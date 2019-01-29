/*
 * CPUFreq governor based on scheduler-provided CPU utilization data.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *
 * Copyright (C) 2017, Alucard24@XDA
 * Author: Alucard24 <dmbaoh2@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <trace/events/power.h>

#include "sched.h"
#include "tune.h"

unsigned long boosted_cpu_util(int cpu);

/* Stub out fast switch routines present on mainline to reduce the backport
 * overhead. */
#define cpufreq_driver_fast_switch(x, y) 0
#define cpufreq_enable_fast_switch(x)
#define cpufreq_disable_fast_switch(x)
#define LATENCY_MULTIPLIER			(1000)
#define ACGOV_KTHREAD_PRIORITY	50

#define OVERWRITE_RATE_LIMIT_US
#ifdef CONFIG_ARCH_MSM8998
#undef OVERWRITE_RATE_LIMIT_US
#endif

struct acgov_tunables {
	struct gov_attr_set attr_set;
	spinlock_t rate_limit_us_lock;
	unsigned int (*rate_limit_us)[2];
	spinlock_t capacity_lock;
	unsigned int (*capacity)[2];
	int nelements;
	bool iowait_boost_enable;
};

struct acgov_policy {
	struct cpufreq_policy *policy;

	struct acgov_tunables *tunables;
	struct list_head tunables_hook;

	raw_spinlock_t update_lock;  /* For shared policies */
	u64 last_freq_update_time;
#ifndef CONFIG_MSM_TRACK_FREQ_TARGET_INDEX
	int cur_index;
#endif
	unsigned int next_freq;

	/* The next fields are only needed if fast switch cannot be used. */
	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;

	bool need_freq_update;
};

struct acgov_cpu {
	struct update_util_data update_util;
	struct acgov_policy *sg_policy;

	bool iowait_boost_pending;
	unsigned int iowait_boost;
	unsigned int iowait_boost_max;
	u64 last_update;

	/* The fields below are only needed when sharing a policy. */
	unsigned long util;
	unsigned long max;
	unsigned int flags;

	/* The field below is for single-CPU policies only. */
#ifdef CONFIG_NO_HZ_COMMON
	unsigned long saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct acgov_cpu, acgov_cpu);

#define LITTLE_NFREQS		22
#define BIG_NFREQS			31
#define DOWN_INDEX			0
#define UP_INDEX			1

/* Target capacity.  */
static unsigned int little_capacity[LITTLE_NFREQS][2] = {
	{0, 102},
	{102, 123},
	{123, 149},
	{149, 175},
	{175, 201},
	{201, 227},
	{227, 253},
	{253, 278},
	{278, 298},
	{298, 324},
	{324, 350},
	{350, 369},
	{369, 395},
	{395, 421},
	{421, 447},
	{447, 472},
	{472, 498},
	{498, 524},
	{524, 563},
	{563, 589},
	{589, 615},
	{615, 641}
};

static unsigned int big_capacity[BIG_NFREQS][2] = {
	{0, 101},
	{101, 116},
	{116, 141},
	{141, 167},
	{167, 193},
	{193, 218},
	{218, 244},
	{244, 269},
	{269, 301},
	{301, 327},
	{327, 353},
	{353, 378},
	{378, 397},
	{397, 423},
	{423, 449},
	{449, 474},
	{474, 500},
	{500, 525},
	{525, 551},
	{551, 577},
	{577, 602},
	{602, 628},
	{628, 653},
	{653, 679},
	{679, 705},
	{705, 737},
	{737, 756},
	{756, 775},
	{775, 781},
	{781, 788},
	{788, 820}
};

static unsigned int little_rate_limit_us[LITTLE_NFREQS][2] = {
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 500}
};

static unsigned int big_rate_limit_us[BIG_NFREQS][2] = {
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 20000},
	{20000, 500000},
	{500, 500}
};

/************************ Governor internals ***********************/

static bool acgov_should_update_freq(struct acgov_policy *sg_policy, u64 time)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	s64 delta_ns, min_rate_limit_ns;
	int index;
	unsigned long flags;

	if (sg_policy->work_in_progress)
		return false;

#ifdef CONFIG_MSM_TRACK_FREQ_TARGET_INDEX
	index = policy->cur_index;
#else
	index = cpufreq_frequency_table_get_index(policy, policy->cur);
	if (index < 0)
		return false;

	sg_policy->cur_index = index;
#endif
	spin_lock_irqsave(&sg_policy->tunables->rate_limit_us_lock, flags);
	min_rate_limit_ns = min(sg_policy->tunables->rate_limit_us[index][UP_INDEX] * NSEC_PER_USEC,
			sg_policy->tunables->rate_limit_us[index][DOWN_INDEX] * NSEC_PER_USEC);
	spin_unlock_irqrestore(&sg_policy->tunables->rate_limit_us_lock, flags);

	if (unlikely(sg_policy->need_freq_update)) {
		sg_policy->need_freq_update = false;
		/*
		 * This happens when limits change, so forget the previous
		 * next_freq value and force an update.
		 */
		sg_policy->next_freq = UINT_MAX;
		return true;
	}
	/*
	 * next_freq value will be set to the current of cpu policy.
	 */
	sg_policy->next_freq = policy->cur;

	delta_ns = time - sg_policy->last_freq_update_time;

	/* No need to recalculate next freq for min_rate_limit_us at least */
	return delta_ns >= min_rate_limit_ns;
}

static void acgov_update_commit(struct acgov_policy *sg_policy, u64 time,
				unsigned int next_freq)
{
	struct cpufreq_policy *policy = sg_policy->policy;

	if (sg_policy->next_freq == next_freq)
		return;

	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;

	if (policy->fast_switch_enabled) {
		next_freq = cpufreq_driver_fast_switch(policy, next_freq);
		if (next_freq == CPUFREQ_ENTRY_INVALID)
			return;

		policy->cur = next_freq;
		trace_cpu_frequency(next_freq, smp_processor_id());
	} else {
		sg_policy->work_in_progress = true;
		irq_work_queue(&sg_policy->irq_work);
	}
}

/**
 * get_next_freq - Compute a new frequency for a given cpufreq policy.
 * @sg_policy: schedalucard policy object to compute the new frequency for.
 * @util: Current CPU utilization.
 * @max: CPU capacity.
 * @time: Updating time.
 *
 * next_freq = frequency taken from up_capacity/down_capacity table 
 * according to the current CPU utilization and the up_rate_limit/down_rate_limit
 * table value.
 *
 * The lowest driver-supported frequency which is equal or greater than the raw
 * next_freq (as calculated above) is returned, subject to policy min/max and
 * cpufreq driver limitations.
 */
static unsigned int get_next_freq(struct acgov_policy *sg_policy,
				  unsigned long util, unsigned long max, u64 time)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	struct acgov_tunables *tunables = sg_policy->tunables;
	struct cpufreq_frequency_table *table = policy->freq_table;
	unsigned int next_freq = policy->cur;
	s64 delta_ns;
	unsigned long flags, rlflags;
	int i;
#ifdef CONFIG_MSM_TRACK_FREQ_TARGET_INDEX
	int index = policy->cur_index;
#else
	int index = sg_policy->cur_index;
#endif

	delta_ns = time - sg_policy->last_freq_update_time;
	spin_lock_irqsave(&tunables->capacity_lock, flags);
	spin_lock_irqsave(&tunables->rate_limit_us_lock, rlflags);
	if (util >= tunables->capacity[index][UP_INDEX]
		 && policy->cur < policy->max
		 && delta_ns >=
			 tunables->rate_limit_us[index][UP_INDEX] * NSEC_PER_USEC) {
		for (i = index + 1; i < tunables->nelements; i++) {
			if (table[i].frequency == CPUFREQ_ENTRY_INVALID)
				continue;

			next_freq = table[i].frequency;
			if ((util <= tunables->capacity[i][UP_INDEX])
				 || (delta_ns <
					 tunables->rate_limit_us[i][UP_INDEX] * NSEC_PER_USEC))
				break;
		}
	} else if (util < tunables->capacity[index][DOWN_INDEX]
		 && policy->cur > policy->min
		 && delta_ns >=
			 tunables->rate_limit_us[index][DOWN_INDEX] * NSEC_PER_USEC) {
		for (i = index - 1; i >= 0; i--) {
			if (table[i].frequency == CPUFREQ_ENTRY_INVALID)
				continue;

			next_freq = table[i].frequency;
			if ((util >= tunables->capacity[i][DOWN_INDEX])
				 || (delta_ns <
					 tunables->rate_limit_us[i][DOWN_INDEX] * NSEC_PER_USEC))
				break;
		}
	}
	spin_unlock_irqrestore(&tunables->rate_limit_us_lock, rlflags);
	spin_unlock_irqrestore(&tunables->capacity_lock, flags);

	return next_freq;
}

static inline bool use_pelt(void)
{
#ifdef CONFIG_SCHED_WALT
	return (!sysctl_sched_use_walt_cpu_util || walt_disabled);
#else
	return true;
#endif
}

static void acgov_get_util(unsigned long *util, unsigned long *max, u64 time)
{
	int cpu = smp_processor_id();
	struct rq *rq = cpu_rq(cpu);
	unsigned long max_cap, rt;
	s64 delta;

	max_cap = arch_scale_cpu_capacity(NULL, cpu);

	sched_avg_update(rq);
	delta = time - rq->age_stamp;
	if (unlikely(delta < 0))
		delta = 0;
	rt = div64_u64(rq->rt_avg, sched_avg_period() + delta);
	rt = (rt * max_cap) >> SCHED_CAPACITY_SHIFT;

	*util = boosted_cpu_util(cpu);
	if (likely(use_pelt()))
		*util = *util + rt;

	*util = min(*util, max_cap);
	*max = max_cap;
}

static void acgov_set_iowait_boost(struct acgov_cpu *sg_cpu, u64 time,
				   unsigned int flags)
{
	struct acgov_policy *sg_policy = sg_cpu->sg_policy;

	if (!sg_policy->tunables->iowait_boost_enable)
		return;

	if (flags & SCHED_CPUFREQ_IOWAIT) {
		if (sg_cpu->iowait_boost_pending)
			return;

		sg_cpu->iowait_boost_pending = true;

		if (sg_cpu->iowait_boost) {
			sg_cpu->iowait_boost <<= 1;
			if (sg_cpu->iowait_boost > sg_cpu->iowait_boost_max)
				sg_cpu->iowait_boost = sg_cpu->iowait_boost_max;
		} else {
			sg_cpu->iowait_boost = sg_cpu->sg_policy->policy->min;
		}
	} else if (sg_cpu->iowait_boost) {
		s64 delta_ns = time - sg_cpu->last_update;

		/* Clear iowait_boost if the CPU apprears to have been idle. */
		if (delta_ns > TICK_NSEC) {
			sg_cpu->iowait_boost = 0;
			sg_cpu->iowait_boost_pending = false;
		}
	}
}

static void acgov_iowait_boost(struct acgov_cpu *sg_cpu, unsigned long *util,
			       unsigned long *max)
{
	unsigned int boost_util, boost_max;

	if (!sg_cpu->iowait_boost)
		return;

	if (sg_cpu->iowait_boost_pending) {
		sg_cpu->iowait_boost_pending = false;
	} else {
		sg_cpu->iowait_boost >>= 1;
		if (sg_cpu->iowait_boost < sg_cpu->sg_policy->policy->min) {
			sg_cpu->iowait_boost = 0;
			return;
		}
	}

	boost_util = sg_cpu->iowait_boost;
	boost_max = sg_cpu->iowait_boost_max;

	if (*util * boost_max < *max * boost_util) {
		*util = boost_util;
		*max = boost_max;
	}
}

#ifdef CONFIG_NO_HZ_COMMON
static bool acgov_cpu_is_busy(struct acgov_cpu *sg_cpu)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls();
	bool ret = idle_calls == sg_cpu->saved_idle_calls;

	sg_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool acgov_cpu_is_busy(struct acgov_cpu *sg_cpu) { return false; }
#endif /* CONFIG_NO_HZ_COMMON */

static void acgov_update_single(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct acgov_cpu *sg_cpu = container_of(hook, struct acgov_cpu, update_util);
	struct acgov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util, max;
	unsigned int next_f;
	bool busy;

	acgov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	if (!acgov_should_update_freq(sg_policy, time))
		return;

	busy = acgov_cpu_is_busy(sg_cpu);

	if (flags & SCHED_CPUFREQ_DL) {
		next_f = policy->cpuinfo.max_freq;
	} else {
		acgov_get_util(&util, &max, time);
		acgov_iowait_boost(sg_cpu, &util, &max);
		next_f = get_next_freq(sg_policy, util, max, time);
		/*
		 * Do not reduce the frequency if the CPU has not been idle
		 * recently and if the policy limits doesn't change, as 
		 * the reduction is likely to be premature then.
		 */
		if (busy
			 && next_f < sg_policy->next_freq
			 && sg_policy->next_freq != UINT_MAX)
			next_f = sg_policy->next_freq;
	}
	acgov_update_commit(sg_policy, time, next_f);
}

static unsigned int acgov_next_freq_shared(struct acgov_cpu *sg_cpu, u64 time)
{
	struct acgov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;

	for_each_cpu(j, policy->cpus) {
		struct acgov_cpu *j_sg_cpu = &per_cpu(acgov_cpu, j);
		unsigned long j_util, j_max;
		s64 delta_ns;

		/*
		 * If the CPU utilization was last updated before the previous
		 * frequency update and the time elapsed between the last update
		 * of the CPU utilization and the last frequency update is long
		 * enough, don't take the CPU into account as it probably is
		 * idle now (and clear iowait_boost for it).
		 */
		delta_ns = time - j_sg_cpu->last_update;
		if (delta_ns > TICK_NSEC) {
			j_sg_cpu->iowait_boost = 0;
			j_sg_cpu->iowait_boost_pending = false;
			continue;
		}
		if (j_sg_cpu->flags & SCHED_CPUFREQ_DL)
			return policy->cpuinfo.max_freq;

		j_util = j_sg_cpu->util;
		j_max = j_sg_cpu->max;
		if (j_util * max > j_max * util) {
			util = j_util;
			max = j_max;
		}

		acgov_iowait_boost(j_sg_cpu, &util, &max);
	}

	return get_next_freq(sg_policy, util, max, time);
}

static void acgov_update_shared(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct acgov_cpu *sg_cpu = container_of(hook, struct acgov_cpu, update_util);
	struct acgov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long util, max;
	unsigned int next_f;

	acgov_get_util(&util, &max, time);

	raw_spin_lock(&sg_policy->update_lock);

	sg_cpu->util = util;
	sg_cpu->max = max;
	sg_cpu->flags = flags;

	acgov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	if (acgov_should_update_freq(sg_policy, time)) {
		if (flags & SCHED_CPUFREQ_DL)
			next_f = sg_policy->policy->cpuinfo.max_freq;
		else
			next_f = acgov_next_freq_shared(sg_cpu, time);

		acgov_update_commit(sg_policy, time, next_f);
	}

	raw_spin_unlock(&sg_policy->update_lock);
}

static void acgov_work(struct kthread_work *work)
{
	struct acgov_policy *sg_policy = container_of(work, struct acgov_policy, work);

	mutex_lock(&sg_policy->work_lock);
	__cpufreq_driver_target(sg_policy->policy, sg_policy->next_freq,
				CPUFREQ_RELATION_L);
	mutex_unlock(&sg_policy->work_lock);

	sg_policy->work_in_progress = false;
}

static void acgov_irq_work(struct irq_work *irq_work)
{
	struct acgov_policy *sg_policy;

	sg_policy = container_of(irq_work, struct acgov_policy, irq_work);

	/*
	 * For RT and deadline tasks, the schedalucard governor shoots the
	 * frequency to maximum. Special care must be taken to ensure that this
	 * kthread doesn't result in the same behavior.
	 *
	 * This is (mostly) guaranteed by the work_in_progress flag. The flag is
	 * updated only at the end of the acgov_work() function and before that
	 * the schedalucard governor rejects all other frequency scaling requests.
	 *
	 * There is a very rare case though, where the RT thread yields right
	 * after the work_in_progress flag is cleared. The effects of that are
	 * neglected for now.
	 */
	queue_kthread_work(&sg_policy->worker, &sg_policy->work);
}

/************************** sysfs interface ************************/

static struct acgov_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct acgov_tunables *to_acgov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct acgov_tunables, attr_set);
}

/* up_rate_limit_us */
static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct acgov_tunables *tunables = to_acgov_tunables(attr_set);
	int i;
	ssize_t ret = 0;
	unsigned long flags;

	if (!tunables->rate_limit_us)
		return -EINVAL;

	spin_lock_irqsave(&tunables->rate_limit_us_lock, flags);
	for (i = 0; i < tunables->nelements; i++)
		ret += sprintf(buf + ret, "%u%s", tunables->rate_limit_us[i][UP_INDEX],
			       ":");
	spin_unlock_irqrestore(&tunables->rate_limit_us_lock, flags);

	sprintf(buf + ret - 1, "\n");

	return ret;
}

static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct acgov_tunables *tunables = to_acgov_tunables(attr_set);
	const char *cp;
	int i;
	int ntokens = 1;
	unsigned long flags;

	if (!tunables->rate_limit_us)
		return -EINVAL;

	cp = buf;
	while ((cp = strpbrk(cp + 1, ":")))
		ntokens++;

	if (ntokens != tunables->nelements)
		return -EINVAL;

	cp = buf;
	spin_lock_irqsave(&tunables->rate_limit_us_lock, flags);
	for (i = 0; i < ntokens; i++) {
		if (sscanf(cp, "%u", &tunables->rate_limit_us[i][UP_INDEX]) != 1) {
			spin_unlock_irqrestore(&tunables->rate_limit_us_lock, flags);
		} else {
			pr_debug("index[%d], val[%u]\n", i, tunables->rate_limit_us[i][UP_INDEX]);
		}

		cp = strpbrk(cp, ":");
		if (!cp)
			break;
		cp++;
	}
	spin_unlock_irqrestore(&tunables->rate_limit_us_lock, flags);

	return count;
}

/* down_rate_limit_us */
static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct acgov_tunables *tunables = to_acgov_tunables(attr_set);
	int i;
	ssize_t ret = 0;
	unsigned long flags;

	if (!tunables->rate_limit_us)
		return -EINVAL;

	spin_lock_irqsave(&tunables->rate_limit_us_lock, flags);
	for (i = 0; i < tunables->nelements; i++)
		ret += sprintf(buf + ret, "%u%s", tunables->rate_limit_us[i][DOWN_INDEX],
			       ":");
	spin_unlock_irqrestore(&tunables->rate_limit_us_lock, flags);

	sprintf(buf + ret - 1, "\n");

	return ret;
}

static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct acgov_tunables *tunables = to_acgov_tunables(attr_set);
	const char *cp;
	int i;
	int ntokens = 1;
	unsigned long flags;

	if (!tunables->rate_limit_us)
		return -EINVAL;

	cp = buf;
	while ((cp = strpbrk(cp + 1, ":")))
		ntokens++;

	if (ntokens != tunables->nelements)
		return -EINVAL;

	cp = buf;
	spin_lock_irqsave(&tunables->rate_limit_us_lock, flags);
	for (i = 0; i < ntokens; i++) {
		if (sscanf(cp, "%u", &tunables->rate_limit_us[i][DOWN_INDEX]) != 1) {
			spin_unlock_irqrestore(&tunables->rate_limit_us_lock, flags);
		} else {
			pr_debug("index[%d], val[%u]\n", i, tunables->rate_limit_us[i][DOWN_INDEX]);
		}

		cp = strpbrk(cp, ":");
		if (!cp)
			break;
		cp++;
	}
	spin_unlock_irqrestore(&tunables->rate_limit_us_lock, flags);

	return count;
}

/* up_capacity */
static ssize_t up_capacity_show(struct gov_attr_set *attr_set, char *buf)
{
	struct acgov_tunables *tunables = to_acgov_tunables(attr_set);
	int i;
	ssize_t ret = 0;
	unsigned long flags;

	if (!tunables->capacity)
		return -EINVAL;

	spin_lock_irqsave(&tunables->capacity_lock, flags);
	for (i = 0; i < tunables->nelements; i++)
		ret += sprintf(buf + ret, "%u%s", tunables->capacity[i][UP_INDEX],
			       ":");
	spin_unlock_irqrestore(&tunables->capacity_lock, flags);

	sprintf(buf + ret - 1, "\n");

	return ret;
}

static ssize_t up_capacity_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct acgov_tunables *tunables = to_acgov_tunables(attr_set);
	const char *cp;
	int i;
	int ntokens = 1;
	unsigned int value = 0;
	unsigned long flags;

	if (!tunables->capacity)
		return -EINVAL;

	cp = buf;
	while ((cp = strpbrk(cp + 1, ":")))
		ntokens++;

	if (ntokens != tunables->nelements)
		return -EINVAL;

	cp = buf;
	spin_lock_irqsave(&tunables->capacity_lock, flags);
	for (i = 0; i < ntokens; i++) {
		if (sscanf(cp, "%u", &value) != 1) {
			spin_unlock_irqrestore(&tunables->capacity_lock, flags);
			return -EINVAL;
		} else {
			pr_debug("index[%d], val[%u]\n", i, tunables->capacity[i][UP_INDEX]);
		}
		if (!value)
			value = 1;
		tunables->capacity[i][UP_INDEX] = value;
		cp = strpbrk(cp, ":");
		if (!cp)
			break;
		cp++;
	}
	spin_unlock_irqrestore(&tunables->capacity_lock, flags);

	return count;
}

/* down_capacity */
static ssize_t down_capacity_show(struct gov_attr_set *attr_set, char *buf)
{
	struct acgov_tunables *tunables = to_acgov_tunables(attr_set);
	int i;
	ssize_t ret = 0;
	unsigned long flags;

	if (!tunables->capacity)
		return -EINVAL;

	spin_lock_irqsave(&tunables->capacity_lock, flags);
	for (i = 0; i < tunables->nelements; i++)
		ret += sprintf(buf + ret, "%u%s", tunables->capacity[i][DOWN_INDEX],
			       ":");
	spin_unlock_irqrestore(&tunables->capacity_lock, flags);

	sprintf(buf + ret - 1, "\n");

	return ret;
}

static ssize_t down_capacity_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct acgov_tunables *tunables = to_acgov_tunables(attr_set);
	const char *cp;
	int i;
	int ntokens = 1;
	unsigned int value = 0;
	unsigned long flags;

	if (!tunables->capacity)
		return -EINVAL;

	cp = buf;
	while ((cp = strpbrk(cp + 1, ":")))
		ntokens++;

	if (ntokens != tunables->nelements)
		return -EINVAL;

	cp = buf;
	spin_lock_irqsave(&tunables->capacity_lock, flags);
	for (i = 0; i < ntokens; i++) {
		if (sscanf(cp, "%u", &value) != 1) {
			spin_unlock_irqrestore(&tunables->capacity_lock, flags);
			return -EINVAL;
		} else {
			if (!value
				&& i > 0)
				value = 1;
			tunables->capacity[i][DOWN_INDEX] = value;
			pr_debug("index[%d], val[%u]\n", i, tunables->capacity[i][DOWN_INDEX]);
		}

		cp = strpbrk(cp, ":");
		if (!cp)
			break;
		cp++;
	}
	spin_unlock_irqrestore(&tunables->capacity_lock, flags);

	return count;
}

static ssize_t iowait_boost_enable_show(struct gov_attr_set *attr_set,
					char *buf)
{
	struct acgov_tunables *tunables = to_acgov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->iowait_boost_enable);
}

static ssize_t iowait_boost_enable_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct acgov_tunables *tunables = to_acgov_tunables(attr_set);
	bool enable;

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	tunables->iowait_boost_enable = enable;

	return count;
}

static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);
static struct governor_attr up_capacity = __ATTR_RW(up_capacity);
static struct governor_attr down_capacity = __ATTR_RW(down_capacity);
static struct governor_attr iowait_boost_enable = __ATTR_RW(iowait_boost_enable);

static struct attribute *acgov_attributes[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&up_capacity.attr,
	&down_capacity.attr,
	&iowait_boost_enable.attr,
	NULL
};

static struct kobj_type acgov_tunables_ktype = {
	.default_attrs = acgov_attributes,
	.sysfs_ops = &governor_sysfs_ops,
};

/********************** cpufreq governor interface *********************/
#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDALUCARD
static
#endif
struct cpufreq_governor cpufreq_gov_schedalucard;

static struct acgov_policy *acgov_policy_alloc(struct cpufreq_policy *policy)
{
	struct acgov_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	raw_spin_lock_init(&sg_policy->update_lock);
	return sg_policy;
}

static void acgov_policy_free(struct acgov_policy *sg_policy)
{
	kfree(sg_policy);
}

static int acgov_kthread_create(struct acgov_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_param param = { .sched_priority = MAX_USER_RT_PRIO / 2 };
	struct cpufreq_policy *policy = sg_policy->policy;
	int ret;

	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;

	init_kthread_work(&sg_policy->work, acgov_work);
	init_kthread_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"acgov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create acgov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		return ret;
	}

	sg_policy->thread = thread;
	kthread_bind_mask(thread, policy->related_cpus);
	init_irq_work(&sg_policy->irq_work, acgov_irq_work);
	mutex_init(&sg_policy->work_lock);

	wake_up_process(thread);

	return 0;
}

static void acgov_kthread_stop(struct acgov_policy *sg_policy)
{
	/* kthread only required for slow path */
	if (sg_policy->policy->fast_switch_enabled)
		return;

	flush_kthread_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}

static struct acgov_tunables *acgov_tunables_alloc(struct acgov_policy *sg_policy)
{
	struct acgov_tunables *tunables;
	struct cpufreq_policy *policy = sg_policy->policy;
	struct cpufreq_frequency_table *table = policy->freq_table;
	unsigned int freq;
	unsigned long max_cap;
	unsigned long flags;
	int i;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
		spin_lock_init(&tunables->rate_limit_us_lock);
		spin_lock_init(&tunables->capacity_lock);
		if (policy->cpu < 4) {
			tunables->rate_limit_us = little_rate_limit_us;
			tunables->capacity = little_capacity;
			tunables->nelements = LITTLE_NFREQS;
		} else {
			tunables->rate_limit_us = big_rate_limit_us;
			tunables->capacity = big_capacity;
			tunables->nelements = BIG_NFREQS;
		}
		/* Auto-populate up_capacity/down_capacity */
		max_cap = arch_scale_cpu_capacity(NULL, policy->cpu);
		spin_lock_irqsave(&tunables->capacity_lock, flags);
		for (i = 0; i < tunables->nelements; i++) {
			if (table[i].frequency == CPUFREQ_ENTRY_INVALID)
				continue;
			freq = arch_scale_freq_invariant() ?
				policy->cpuinfo.max_freq : table[i].frequency;
			freq = freq + (freq >> 2);
			tunables->capacity[i][UP_INDEX] = ((max_cap * table[i].frequency) / freq) + 1;
			if (i == 0) {
				tunables->capacity[i][DOWN_INDEX] = 0;
			} else {
				tunables->capacity[i][DOWN_INDEX] = tunables->capacity[i - 1][UP_INDEX];
			}	
		}
		spin_unlock_irqrestore(&tunables->capacity_lock, flags);
#ifdef OVERWRITE_RATE_LIMIT_US
		spin_lock_irqsave(&tunables->rate_limit_us_lock, flags);		
		if (policy->up_transition_delay_us && policy->down_transition_delay_us) {
			for (i = 0; i < tunables->nelements; i++) {
				tunables->rate_limit_us[i][UP_INDEX] = policy->up_transition_delay_us;
				tunables->rate_limit_us[i][DOWN_INDEX] = policy->down_transition_delay_us;
			}
		} else {
			unsigned int lat;

			lat = policy->cpuinfo.transition_latency / NSEC_PER_USEC;
			for (i = 0; i < tunables->nelements; i++) {
				tunables->rate_limit_us[i][UP_INDEX] = LATENCY_MULTIPLIER;
				tunables->rate_limit_us[i][DOWN_INDEX] = LATENCY_MULTIPLIER;
				if (lat) {
					tunables->rate_limit_us[i][UP_INDEX] *= lat;
					tunables->rate_limit_us[i][DOWN_INDEX] *= lat;
				}
			}
		}
		spin_unlock_irqrestore(&tunables->rate_limit_us_lock, flags);
#endif
		if (!have_governor_per_policy())
			global_tunables = tunables;
	}
	return tunables;
}

static void acgov_tunables_free(struct acgov_tunables *tunables)
{
	if (!have_governor_per_policy())
		global_tunables = NULL;

	tunables->rate_limit_us = NULL;
	tunables->capacity = NULL;

	kfree(tunables);
}

static int acgov_init(struct cpufreq_policy *policy)
{
	struct acgov_policy *sg_policy;
	struct acgov_tunables *tunables;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	sg_policy = acgov_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = acgov_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;

	mutex_lock(&global_tunables_lock);

	if (global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = sg_policy;
		sg_policy->tunables = global_tunables;

		gov_attr_set_get(&global_tunables->attr_set, &sg_policy->tunables_hook);
		goto out;
	}

	tunables = acgov_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;

	ret = kobject_init_and_add(&tunables->attr_set.kobj, &acgov_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   cpufreq_gov_schedalucard.name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&global_tunables_lock);
	return 0;

fail:
	policy->governor_data = NULL;
	acgov_tunables_free(tunables);

stop_kthread:
	acgov_kthread_stop(sg_policy);

free_sg_policy:
	mutex_unlock(&global_tunables_lock);

	acgov_policy_free(sg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);

	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static int acgov_exit(struct cpufreq_policy *policy)
{
	struct acgov_policy *sg_policy = policy->governor_data;
	struct acgov_tunables *tunables = sg_policy->tunables;
	unsigned int count;

	mutex_lock(&global_tunables_lock);

	count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		acgov_tunables_free(tunables);

	mutex_unlock(&global_tunables_lock);

	acgov_kthread_stop(sg_policy);
	acgov_policy_free(sg_policy);

	cpufreq_disable_fast_switch(policy);
	return 0;
}

static int acgov_start(struct cpufreq_policy *policy)
{
	struct acgov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	sg_policy->last_freq_update_time = 0;
	sg_policy->next_freq = UINT_MAX;
	sg_policy->work_in_progress = false;
	sg_policy->need_freq_update = false;

	for_each_cpu(cpu, policy->cpus) {
		struct acgov_cpu *sg_cpu = &per_cpu(acgov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->sg_policy = sg_policy;
		sg_cpu->flags = SCHED_CPUFREQ_DL;
		sg_cpu->iowait_boost_max = policy->cpuinfo.max_freq;
		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util,
					     policy_is_shared(policy) ?
							acgov_update_shared :
							acgov_update_single);
	}
	return 0;
}

static int acgov_stop(struct cpufreq_policy *policy)
{
	struct acgov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_sched();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&sg_policy->irq_work);
		kthread_cancel_work_sync(&sg_policy->work);
	}
	return 0;
}

static int acgov_limits(struct cpufreq_policy *policy)
{
	struct acgov_policy *sg_policy = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&sg_policy->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&sg_policy->work_lock);
	}

	sg_policy->need_freq_update = true;

	return 0;
}

static int cpufreq_schedalucard_cb(struct cpufreq_policy *policy,
				unsigned int event)
{
	switch(event) {
	case CPUFREQ_GOV_POLICY_INIT:
		return acgov_init(policy);
	case CPUFREQ_GOV_POLICY_EXIT:
		return acgov_exit(policy);
	case CPUFREQ_GOV_START:
		return acgov_start(policy);
	case CPUFREQ_GOV_STOP:
		return acgov_stop(policy);
	case CPUFREQ_GOV_LIMITS:
		return acgov_limits(policy);
	default:
		BUG();
	}
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDALUCARD
static
#endif
struct cpufreq_governor cpufreq_gov_schedalucard = {
	.name = "schedalucard",
	.governor = cpufreq_schedalucard_cb,
	.owner = THIS_MODULE,
};

static int __init acgov_register(void)
{
	return cpufreq_register_governor(&cpufreq_gov_schedalucard);
}
fs_initcall(acgov_register);
