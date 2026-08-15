/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_SCHED_EVA_H
#define _LINUX_SCHED_EVA_H

#include <linux/sched.h>
#include <linux/jump_label.h>
#include <linux/cpumask.h>
#include <linux/version.h>

#define SCHED_EVA_AUTHOR   "RapliVx"
#define SCHED_EVA_PROGNAME "E.V.A (Enhanced Visual-render Affinity)"
#define SCHED_EVA_VERSION  "1.0"

#define EVA_MAX_CLUSTERS 4
#define EVA_MAX_TRACKED_THREADS 64

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
#define eva_cpus_allowed(t) ((t)->cpus_mask)
#else
#define eva_cpus_allowed(t) ((t)->cpus_allowed)
#endif

/* Static Thread Pattern Matching */
static const char *eva_comm_patterns[] = {
	/* Unreal Engine */
	"RenderThread", "GameThread", "TaskGraph", "RHIThread", "FMOD", "AudioThread",
	/* Unity Engine */
	"UnityMain", "UnityGfx", "UnityPreload", "UnityGraphics", "Worker Thread",
	/* Godot & Cocos */
	"Godot_", "godot", "Cocos2dx", "Cocos",
	/* Generic Graphics APIs */
	"glthread", "Vulkan", "ANGLE", "mali", "adreno", "kgsl", "hwuiTask", "FrameWorker",
	/* Emulators (Yuzu, Citra, Dolphin, Skyline) */
	"nvn", "yuzu", "Citra", "EmuThread", "JIT", "GPU", "CPUThread", "MainThread"
};

struct eva_thread_state {
	pid_t tid;
	u64 start_time;
	int original_nice;
	cpumask_t original_mask;
};

extern int __read_mostly sched_eva_enable;
DECLARE_STATIC_KEY_TRUE(sched_eva_enable_key);

extern int sched_eva_pid;
extern int sched_eva_nice;
extern int sched_eva_smart_mode;
extern int sched_eva_throttle_freq;
extern int sched_eva_heavy_util;
extern int sched_eva_light_util;
extern int sched_eva_cluster_depth;
extern int sched_eva_uclamp_min;
extern char sched_eva_thread_patterns[256];

#endif /* _LINUX_SCHED_EVA_H */
