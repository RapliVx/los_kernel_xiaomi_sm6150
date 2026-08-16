// SPDX-License-Identifier: GPL-2.0
/*
 * E.V.A (Enhanced Visual-render Affinity) Thread Optimizer
 * Copyright (C) 2026 xMikkkaa
 *
 * E.V.A optimizes game rendering threads by applying nice boosts and
 * big-core affinity. It is activated on-demand via sysctl and operates
 * entirely isolated from the core scheduler (fair.c, core.c, rt.c).
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/sysctl.h>
#include <linux/cpumask.h>
#include <linux/sched/eva.h>
#include <linux/rcupdate.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/capability.h>
#include <linux/ratelimit.h>
#include <linux/cpufreq.h>
#include <linux/seq_file.h>
#include <linux/cpuhotplug.h>
#include <linux/slab.h>
#include <linux/sort.h>
#if defined(CONFIG_UCLAMP_TASK)
#include <uapi/linux/sched/types.h>
#endif
#include <trace/events/sched.h>
#include "sched.h"

/* ======================== Global sysctl variables ======================== */

int __read_mostly sched_eva_enable = 1;
DEFINE_STATIC_KEY_TRUE(sched_eva_enable_key);

int sched_eva_pid;
int sched_eva_nice = -10;
int sched_eva_smart_mode = 2; /* 0: Off, 1: Hard Pin, 2: Smart Logic */
int sched_eva_throttle_freq = 1400000;
int sched_eva_heavy_util = 300;
int sched_eva_light_util = 100;
int sched_eva_cluster_depth = 0;
int sched_eva_uclamp_min = 0;
char sched_eva_thread_patterns[256] = "";

/* ======================== Internal state ================================= */

static struct eva_thread_state eva_threads[EVA_MAX_TRACKED_THREADS];
static int eva_thread_count;
static DEFINE_MUTEX(eva_mutex);
static struct task_struct *eva_target_buffer[EVA_MAX_TRACKED_THREADS];

static cpumask_t eva_clusters[EVA_MAX_CLUSTERS];
static unsigned long eva_cluster_capacity[EVA_MAX_CLUSTERS];
static int eva_cluster_count;
static DEFINE_MUTEX(eva_topo_mutex);

static struct workqueue_struct *eva_wq;

static struct work_struct eva_fork_work;
static struct work_struct eva_exit_work;
static atomic_t eva_exit_pid = ATOMIC_INIT(0);

static struct delayed_work eva_smart_dwork;
static bool eva_smart_running;

/* ======================== Smart worker lifecycle ========================= */

static void start_smart_worker(void)
{
	if (!eva_smart_running && sched_eva_smart_mode == 2) {
		eva_smart_running = true;
		schedule_delayed_work(&eva_smart_dwork, 0);
	}
}

static void stop_smart_worker(void)
{
	if (eva_smart_running) {
		eva_smart_running = false;
		cancel_delayed_work_sync(&eva_smart_dwork);
	}
}

/* ======================== Topology detection ============================= */

static unsigned long eva_get_cpu_capacity(int cpu)
{
	return capacity_orig_of(cpu);
}

struct eva_tier {
	unsigned long cap;
	cpumask_t mask;
};

static int eva_tier_cmp(const void *a, const void *b)
{
	const struct eva_tier *ta = a;
	const struct eva_tier *tb = b;
	return (ta->cap > tb->cap) - (ta->cap < tb->cap);
}

static void eva_detect_clusters(void)
{
	int cpu, i;
	int temp_count = 0;
	struct eva_tier tiers[16];

	mutex_lock(&eva_topo_mutex);
	if (eva_cluster_count > 0) {
		mutex_unlock(&eva_topo_mutex);
		return;
	}

	for (i = 0; i < 16; i++) {
		cpumask_clear(&tiers[i].mask);
		tiers[i].cap = 0;
	}

	for_each_possible_cpu(cpu) {
		unsigned long cap = eva_get_cpu_capacity(cpu);
		bool found = false;

		for (i = 0; i < temp_count; i++) {
			unsigned long max_c = max(cap, tiers[i].cap);
			unsigned long min_c = min(cap, tiers[i].cap);
			if (max_c - min_c <= (max_c * 6) / 100) {
				cpumask_set_cpu(cpu, &tiers[i].mask);
				tiers[i].cap = max_c;
				found = true;
				break;
			}
		}
		if (!found && temp_count < 16) {
			cpumask_set_cpu(cpu, &tiers[temp_count].mask);
			tiers[temp_count].cap = cap;
			temp_count++;
		}
	}

	sort(tiers, temp_count, sizeof(struct eva_tier), eva_tier_cmp, NULL);

	while (temp_count > EVA_MAX_CLUSTERS) {
		cpumask_or(&tiers[1].mask, &tiers[0].mask, &tiers[1].mask);
		for (i = 0; i < temp_count - 1; i++) {
			cpumask_copy(&tiers[i].mask, &tiers[i + 1].mask);
			tiers[i].cap = tiers[i + 1].cap;
		}
		temp_count--;
	}

	if (temp_count == 1 && cpumask_weight(&tiers[0].mask) == num_possible_cpus()) {
		pr_warn_ratelimited("eva: cluster detection unreliable, retrying later\n");
	} else {
		for (i = 0; i < temp_count; i++) {
			cpumask_copy(&eva_clusters[i], &tiers[i].mask);
			eva_cluster_capacity[i] = tiers[i].cap;
			pr_info("eva: tier %d (cap ~%lu) mask: %*pbl\n",
				i, tiers[i].cap, cpumask_pr_args(&eva_clusters[i]));
		}
		eva_cluster_count = temp_count;
	}
	mutex_unlock(&eva_topo_mutex);
}

/* ======================== Thread matching ================================ */

static bool eva_match_thread(struct task_struct *t)
{
	int i;
	char *token, *rest;
	char buf[256];
	bool matched = false;

	/* 1. Heuristic Detection: Thread Policy and Priority */
	if (t->policy == SCHED_FIFO || t->policy == SCHED_RR)
		return true;
		
	if (task_nice(t) < 0)
		return true;

	/* 2. Check static patterns */
	for (i = 0; i < ARRAY_SIZE(eva_comm_patterns); i++) {
		if (strnstr(t->comm, eva_comm_patterns[i], TASK_COMM_LEN))
			return true;
	}

	/* 3. Check dynamic patterns without sleeping allocator */
	if (sched_eva_thread_patterns[0] == '\0')
		return false;

	strlcpy(buf, sched_eva_thread_patterns, sizeof(buf));
	rest = buf;
	
	while ((token = strsep(&rest, ",")) != NULL) {
		token = strim(token);
		if (*token && strnstr(t->comm, token, TASK_COMM_LEN)) {
			matched = true;
			break;
		}
	}
	return matched;
}

/* ======================== Revert logic =================================== */

static bool eva_revert_all_threads(bool clear_state, int expected_pid)
{
	int i;
	struct task_struct *t;

	stop_smart_worker();

	mutex_lock(&eva_mutex);

	if (expected_pid > 0 && READ_ONCE(sched_eva_pid) != expected_pid) {
		mutex_unlock(&eva_mutex);
		return false;
	}

	for (i = 0; i < eva_thread_count; i++) {
		struct eva_thread_state *state = &eva_threads[i];

		rcu_read_lock();
		t = find_task_by_pid_ns(state->tid, &init_pid_ns);
		if (t) {
			if (t->start_time != state->start_time) {
				t = NULL;
			} else {
				get_task_struct(t);
			}
		}
		rcu_read_unlock();

		if (t) {
			int err;
			set_user_nice(t, state->original_nice);
#if defined(CONFIG_UCLAMP_TASK)
			if (sched_eva_uclamp_min > 0) {
				struct sched_attr attr;
				memset(&attr, 0, sizeof(attr));
				attr.size = sizeof(attr);
				attr.sched_util_min = 0;
				attr.sched_flags = SCHED_FLAG_UTIL_CLAMP_MIN;
				sched_setattr(t, &attr);
			}
#endif
			err = set_cpus_allowed_ptr(t, &state->original_mask);
			if (err)
				pr_warn_ratelimited("eva: affinity set failed for tid %d (err=%d)\n",
						    t->pid, err);
			put_task_struct(t);
		}
	}

	if (clear_state)
		eva_thread_count = 0;

	mutex_unlock(&eva_mutex);
	return true;
}

/* ======================== Optimize threads ================================ */

static void eva_optimize_threads(pid_t pid)
{
	struct task_struct *p, *t;
	int target_count = 0;
	int i, start, effective_depth;
	cpumask_t allowed_mask;
	bool has_big_cores;

	if (pid <= 0)
		return;

	eva_detect_clusters();

	mutex_lock(&eva_topo_mutex);
	cpumask_clear(&allowed_mask);
	effective_depth = (sched_eva_cluster_depth == 0)
		? max(1, eva_cluster_count - 1)
		: sched_eva_cluster_depth;
	start = max(0, eva_cluster_count - effective_depth);
	for (i = start; i < eva_cluster_count; i++)
		cpumask_or(&allowed_mask, &allowed_mask, &eva_clusters[i]);
	cpumask_and(&allowed_mask, &allowed_mask, cpu_online_mask);
	mutex_unlock(&eva_topo_mutex);
	has_big_cores = !cpumask_empty(&allowed_mask);

	rcu_read_lock();
	p = find_task_by_pid_ns(pid, &init_pid_ns);
	if (!p) {
		rcu_read_unlock();
		return;
	}
	get_task_struct(p);
	rcu_read_unlock();

	mutex_lock(&eva_mutex);
	read_lock(&tasklist_lock);
	for_each_thread(p, t) {
		if (eva_match_thread(t)) {
			if (target_count < EVA_MAX_TRACKED_THREADS) {
				get_task_struct(t);
				eva_target_buffer[target_count++] = t;
			} else {
				pr_warn_ratelimited("eva: thread cap reached (%d), dropping thread\n",
						    EVA_MAX_TRACKED_THREADS);
			}
		}
	}
	read_unlock(&tasklist_lock);
	put_task_struct(p);

	if (target_count == 0) {
		mutex_unlock(&eva_mutex);
		return;
	}

	for (i = 0; i < eva_thread_count; i++) {
		struct eva_thread_state *state = &eva_threads[i];
		bool dead_or_reused = false;

		rcu_read_lock();
		t = find_task_by_pid_ns(state->tid, &init_pid_ns);
		if (!t || t->tgid != pid || t->start_time != state->start_time)
			dead_or_reused = true;
		rcu_read_unlock();

		if (dead_or_reused) {
			eva_thread_count--;
			if (i < eva_thread_count) {
				eva_threads[i] = eva_threads[eva_thread_count];
				i--;
			}
		}
	}

	for (i = 0; i < target_count; i++) {
		bool tracked = false;
		int j;

		t = eva_target_buffer[i];

		for (j = 0; j < eva_thread_count; j++) {
			if (eva_threads[j].tid == t->pid &&
			    eva_threads[j].start_time == t->start_time) {
				tracked = true;
				break;
			}
		}

		if (!tracked && eva_thread_count < EVA_MAX_TRACKED_THREADS) {
			unsigned long flags;
			struct eva_thread_state *state =
				&eva_threads[eva_thread_count++];
			state->tid = t->pid;
			state->start_time = t->start_time;
			state->original_nice = task_nice(t);

			raw_spin_lock_irqsave(&t->pi_lock, flags);
			cpumask_copy(&state->original_mask,
				     &eva_cpus_allowed(t));
			raw_spin_unlock_irqrestore(&t->pi_lock, flags);
		}

		set_user_nice(t, sched_eva_nice);
#if defined(CONFIG_UCLAMP_TASK)
		if (sched_eva_uclamp_min > 0) {
			struct sched_attr attr;
			memset(&attr, 0, sizeof(attr));
			attr.size = sizeof(attr);
			attr.sched_util_min = sched_eva_uclamp_min;
			attr.sched_flags = SCHED_FLAG_UTIL_CLAMP_MIN;
			sched_setattr(t, &attr);
		}
#endif

		if (sched_eva_smart_mode == 1 && has_big_cores) {
			int err = set_cpus_allowed_ptr(t, &allowed_mask);
			if (err)
				pr_warn_ratelimited("eva: affinity set failed for tid %d (err=%d)\n",
						    t->pid, err);
		}

		put_task_struct(t);
	}
	mutex_unlock(&eva_mutex);

	if (sched_eva_smart_mode == 2)
		start_smart_worker();

	pr_info_ratelimited("eva: optimized game pid %d (%d threads)\n",
			    pid, target_count);
}

/* ======================== Smart worker =================================== */

static void eva_smart_work_fn(struct work_struct *work)
{
	int i, start, effective_depth;
	struct task_struct *t;
	unsigned int big_core_freq = 0;
	bool is_throttled = false;
	cpumask_t allowed_mask;
	int pid = READ_ONCE(sched_eva_pid);

	if (pid <= 0 ||
	    !static_branch_likely(&sched_eva_enable_key))
		goto end;

	if (sched_eva_smart_mode != 2)
		goto end;

	mutex_lock(&eva_topo_mutex);
	if (eva_cluster_count > 0) {
		int cpu = cpumask_first(&eva_clusters[eva_cluster_count - 1]);
		big_core_freq = cpufreq_quick_get(cpu);
	}

	cpumask_clear(&allowed_mask);
	effective_depth = (sched_eva_cluster_depth == 0)
		? max(1, eva_cluster_count - 1)
		: sched_eva_cluster_depth;
	start = max(0, eva_cluster_count - effective_depth);
	for (i = start; i < eva_cluster_count; i++)
		cpumask_or(&allowed_mask, &allowed_mask, &eva_clusters[i]);
	cpumask_and(&allowed_mask, &allowed_mask, cpu_online_mask);
	mutex_unlock(&eva_topo_mutex);

	if (big_core_freq > 0 && big_core_freq < sched_eva_throttle_freq)
		is_throttled = true;

	mutex_lock(&eva_mutex);
	for (i = 0; i < eva_thread_count; i++) {
		struct eva_thread_state *state = &eva_threads[i];
		unsigned long util = 0;
		bool dead_or_reused = false;

		rcu_read_lock();
		t = find_task_by_pid_ns(state->tid, &init_pid_ns);
		if (t) {
			if (t->tgid == pid &&
			    t->start_time == state->start_time) {
				get_task_struct(t);
				util = task_util(t);
			} else {
				t = NULL;
				dead_or_reused = true;
			}
		} else {
			dead_or_reused = true;
		}
		rcu_read_unlock();

		if (dead_or_reused) {
			eva_thread_count--;
			if (i < eva_thread_count) {
				eva_threads[i] = eva_threads[eva_thread_count];
				i--;
			}
			continue;
		}

		if (t) {
			int err = 0;
			if (is_throttled) {
				err = set_cpus_allowed_ptr(t, &state->original_mask);
			} else {
				if (util > sched_eva_heavy_util) {
					if (!cpumask_empty(&allowed_mask))
						err = set_cpus_allowed_ptr(t,
							&allowed_mask);
				} else if (util < sched_eva_light_util) {
					err = set_cpus_allowed_ptr(t,
						&state->original_mask);
				}
			}
			if (err)
				pr_warn_ratelimited("eva: affinity set failed for tid %d (err=%d)\n",
						    t->pid, err);
			put_task_struct(t);
		}
	}
	mutex_unlock(&eva_mutex);

end:
	if (eva_smart_running)
		schedule_delayed_work(&eva_smart_dwork,
				      msecs_to_jiffies(500));
}



/* ======================== Tracepoint work handlers ======================= */

static void eva_fork_work_fn(struct work_struct *work)
{
	int pid = READ_ONCE(sched_eva_pid);

	if (pid > 0 && static_branch_likely(&sched_eva_enable_key))
		eva_optimize_threads(pid);
}

static void eva_exit_work_fn(struct work_struct *work)
{
	int exit_pid = atomic_xchg(&eva_exit_pid, 0);

	if (exit_pid > 0) {
		if (eva_revert_all_threads(true, exit_pid)) {
			if (cmpxchg(&sched_eva_pid, exit_pid, 0) == exit_pid)
				pr_info("eva: auto-reverted on game exit (pid %d)\n",
					exit_pid);
		}
	}
}

/* ======================== Tracepoint probes ============================== */

static void eva_queue_work(struct work_struct *work);

static int eva_cpu_hotplug_callback(unsigned int cpu)
{
	int pid = READ_ONCE(sched_eva_pid);
	if (pid > 0 && static_branch_likely(&sched_eva_enable_key))
		eva_queue_work(&eva_fork_work);
	return 0;
}

static void eva_queue_work(struct work_struct *work)
{
	if (eva_wq)
		queue_work(eva_wq, work);
	else
		schedule_work(work);
}

static void probe_sched_process_fork(void *ignore,
				      struct task_struct *parent,
				      struct task_struct *child)
{
	int pid = READ_ONCE(sched_eva_pid);

	if (pid > 0 && parent->tgid == pid)
		eva_queue_work(&eva_fork_work);
}

static void probe_sched_process_exit(void *ignore, struct task_struct *p)
{
	int pid = READ_ONCE(sched_eva_pid);

	if (pid > 0 && p->tgid == pid && p->pid == pid) {
		atomic_set(&eva_exit_pid, pid);
		eva_queue_work(&eva_exit_work);
	}
}

/* ======================== Sysctl handlers ================================ */

static int sched_eva_pid_handler(struct ctl_table *table, int write,
				   void __user *buffer, size_t *lenp,
				   loff_t *ppos)
{
	int ret;
	int old_pid = READ_ONCE(sched_eva_pid);

	if (write && !capable(CAP_SYS_NICE))
		return -EPERM;

	ret = proc_dointvec(table, write, buffer, lenp, ppos);
	if (ret || !write)
		return ret;

	if (sched_eva_pid != old_pid) {
		pr_info("eva: PID manually changed from %d to %d\n", old_pid, sched_eva_pid);
		if (old_pid > 0)
			eva_revert_all_threads(true, 0);

		if (sched_eva_pid > 0 &&
		    static_branch_likely(&sched_eva_enable_key))
			eva_optimize_threads(sched_eva_pid);
	}

	return 0;
}

static int sched_eva_enable_handler(struct ctl_table *table, int write,
				      void __user *buffer, size_t *lenp,
				      loff_t *ppos)
{
	int ret;
	int old = sched_eva_enable;

	if (write && !capable(CAP_SYS_NICE))
		return -EPERM;

	ret = proc_dointvec(table, write, buffer, lenp, ppos);
	if (ret || !write)
		return ret;

	if (sched_eva_enable != old) {
		pr_info("eva: Master Switch %s\n", sched_eva_enable ? "ENABLED" : "DISABLED");
		if (sched_eva_enable) {
			static_branch_enable(&sched_eva_enable_key);
			if (sched_eva_pid > 0)
				eva_optimize_threads(sched_eva_pid);
		} else {
			static_branch_disable(&sched_eva_enable_key);
			if (sched_eva_pid > 0)
				eva_revert_all_threads(true, 0);
		}
	}
	return 0;
}

static struct ctl_table sched_eva_sysctls[] = {
	{
		.procname	= "sched_eva_enable",
		.data		= &sched_eva_enable,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= sched_eva_enable_handler,
	},

	{
		.procname	= "sched_eva_pid",
		.data		= &sched_eva_pid,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= sched_eva_pid_handler,
	},
	{
		.procname	= "sched_eva_nice",
		.data		= &sched_eva_nice,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "sched_eva_smart_mode",
		.data		= &sched_eva_smart_mode,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "sched_eva_throttle_freq",
		.data		= &sched_eva_throttle_freq,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "sched_eva_heavy_util",
		.data		= &sched_eva_heavy_util,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "sched_eva_light_util",
		.data		= &sched_eva_light_util,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "sched_eva_cluster_depth",
		.data		= &sched_eva_cluster_depth,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "sched_eva_uclamp_min",
		.data		= &sched_eva_uclamp_min,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "sched_eva_thread_patterns",
		.data		= sched_eva_thread_patterns,
		.maxlen		= 256,
		.mode		= 0644,
		.proc_handler	= proc_dostring,
	},
	{}
};

static int __init sched_eva_init(void)
{
	int ret;

	register_sysctl("kernel", sched_eva_sysctls);

	eva_wq = alloc_workqueue("eva_wq", WQ_HIGHPRI | WQ_UNBOUND, 1);
	INIT_WORK(&eva_fork_work, eva_fork_work_fn);
	INIT_WORK(&eva_exit_work, eva_exit_work_fn);
	INIT_DELAYED_WORK(&eva_smart_dwork, eva_smart_work_fn);


	ret = register_trace_sched_process_fork(probe_sched_process_fork, NULL);
	if (ret)
		pr_err("eva: failed to register fork tracepoint\n");

	ret = register_trace_sched_process_exit(probe_sched_process_exit, NULL);
	if (ret)
		pr_err("eva: failed to register exit tracepoint\n");

	cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN, "sched/eva:online",
				  eva_cpu_hotplug_callback, NULL);

	pr_info("E.V.A: Enhanced Visual-render Affinity Initialized\n");
	return 0;
}
late_initcall(sched_eva_init);
