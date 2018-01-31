/*
 * Copyright © 2014-2017 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include <linux/debugfs.h>
#include <linux/relay.h>

#include "intel_guc_log.h"
#include "i915_drv.h"

static void guc_log_capture_logs(struct intel_guc *guc);

/**
 * DOC: GuC firmware log
 *
 * Firmware log is enabled by setting i915.guc_log_level to the positive level.
 * Log data is printed out via reading debugfs i915_guc_log_dump. Reading from
 * i915_guc_load_status will print out firmware loading status and scratch
 * registers value.
 */

static int guc_log_flush_complete(struct intel_guc *guc)
{
	u32 action[] = {
		INTEL_GUC_ACTION_LOG_BUFFER_FILE_FLUSH_COMPLETE
	};

	return intel_guc_send(guc, action, ARRAY_SIZE(action));
}

static int guc_log_flush(struct intel_guc *guc)
{
	u32 action[] = {
		INTEL_GUC_ACTION_FORCE_LOG_BUFFER_FLUSH,
		0
	};

	return intel_guc_send(guc, action, ARRAY_SIZE(action));
}

static int guc_log_control(struct intel_guc *guc, bool enable, u32 verbosity)
{
	union guc_log_control control_val = {
		.logging_enabled = enable,
		.verbosity = verbosity,
	};
	u32 action[] = {
		INTEL_GUC_ACTION_UK_LOG_ENABLE_LOGGING,
		control_val.value
	};

	return intel_guc_send(guc, action, ARRAY_SIZE(action));
}

/*
 * Sub buffer switch callback. Called whenever relay has to switch to a new
 * sub buffer, relay stays on the same sub buffer if 0 is returned.
 */
static int subbuf_start_callback(struct rchan_buf *buf,
				 void *subbuf,
				 void *prev_subbuf,
				 size_t prev_padding)
{
	/*
	 * Use no-overwrite mode by default, where relay will stop accepting
	 * new data if there are no empty sub buffers left.
	 * There is no strict synchronization enforced by relay between Consumer
	 * and Producer. In overwrite mode, there is a possibility of getting
	 * inconsistent/garbled data, the producer could be writing on to the
	 * same sub buffer from which Consumer is reading. This can't be avoided
	 * unless Consumer is fast enough and can always run in tandem with
	 * Producer.
	 */
	if (relay_buf_full(buf))
		return 0;

	return 1;
}

/*
 * file_create() callback. Creates relay file in debugfs.
 */
static struct dentry *create_buf_file_callback(const char *filename,
					       struct dentry *parent,
					       umode_t mode,
					       struct rchan_buf *buf,
					       int *is_global)
{
	struct dentry *buf_file;

	/*
	 * This to enable the use of a single buffer for the relay channel and
	 * correspondingly have a single file exposed to User, through which
	 * it can collect the logs in order without any post-processing.
	 * Need to set 'is_global' even if parent is NULL for early logging.
	 */
	*is_global = 1;

	if (!parent)
		return NULL;

	/*
	 * Not using the channel filename passed as an argument, since for each
	 * channel relay appends the corresponding CPU number to the filename
	 * passed in relay_open(). This should be fine as relay just needs a
	 * dentry of the file associated with the channel buffer and that file's
	 * name need not be same as the filename passed as an argument.
	 */
	buf_file = debugfs_create_file(filename, mode,
				       parent, buf, &relay_file_operations);
	return buf_file;
}

/*
 * file_remove() default callback. Removes relay file in debugfs.
 */
static int remove_buf_file_callback(struct dentry *dentry)
{
	debugfs_remove(dentry);
	return 0;
}

/* relay channel callbacks */
static struct rchan_callbacks relay_callbacks = {
	.subbuf_start = subbuf_start_callback,
	.create_buf_file = create_buf_file_callback,
	.remove_buf_file = remove_buf_file_callback,
};

static void guc_move_to_next_buf(struct intel_guc *guc)
{
	/*
	 * Make sure the updates made in the sub buffer are visible when
	 * Consumer sees the following update to offset inside the sub buffer.
	 */
	smp_wmb();

	/* All data has been written, so now move the offset of sub buffer. */
	relay_reserve(guc->log.relay_chan, guc->log.vma->obj->base.size);

	/* Switch to the next sub buffer */
	relay_flush(guc->log.relay_chan);
}

static void *guc_get_write_buffer(struct intel_guc *guc)
{
	/*
	 * Just get the base address of a new sub buffer and copy data into it
	 * ourselves. NULL will be returned in no-overwrite mode, if all sub
	 * buffers are full. Could have used the relay_write() to indirectly
	 * copy the data, but that would have been bit convoluted, as we need to
	 * write to only certain locations inside a sub buffer which cannot be
	 * done without using relay_reserve() along with relay_write(). So its
	 * better to use relay_reserve() alone.
	 */
	return relay_reserve(guc->log.relay_chan, 0);
}

static bool guc_check_log_buf_overflow(struct intel_guc *guc,
				       enum guc_log_buffer_type type,
				       unsigned int full_cnt)
{
	unsigned int prev_full_cnt = guc->log.prev_overflow_count[type];
	bool overflow = false;

	if (full_cnt != prev_full_cnt) {
		overflow = true;

		guc->log.prev_overflow_count[type] = full_cnt;
		guc->log.total_overflow_count[type] += full_cnt - prev_full_cnt;

		if (full_cnt < prev_full_cnt) {
			/* buffer_full_cnt is a 4 bit counter */
			guc->log.total_overflow_count[type] += 16;
		}
		DRM_ERROR_RATELIMITED("GuC log buffer overflow\n");
	}

	return overflow;
}

static unsigned int guc_get_log_buffer_size(enum guc_log_buffer_type type)
{
	switch (type) {
	case GUC_ISR_LOG_BUFFER:
		return (GUC_LOG_ISR_PAGES + 1) * PAGE_SIZE;
	case GUC_DPC_LOG_BUFFER:
		return (GUC_LOG_DPC_PAGES + 1) * PAGE_SIZE;
	case GUC_CRASH_DUMP_LOG_BUFFER:
		return (GUC_LOG_CRASH_PAGES + 1) * PAGE_SIZE;
	default:
		MISSING_CASE(type);
	}

	return 0;
}

static void guc_read_update_log_buffer(struct intel_guc *guc)
{
	unsigned int buffer_size, read_offset, write_offset, bytes_to_copy, full_cnt;
	struct guc_log_buffer_state *log_buf_state, *log_buf_snapshot_state;
	struct guc_log_buffer_state log_buf_state_local;
	enum guc_log_buffer_type type;
	void *src_data, *dst_data;
	bool new_overflow;

	if (WARN_ON(!guc->log.buf_addr))
		return;

	/* Get the pointer to shared GuC log buffer */
	log_buf_state = src_data = guc->log.buf_addr;

	/* Get the pointer to local buffer to store the logs */
	log_buf_snapshot_state = dst_data = guc_get_write_buffer(guc);

	if (unlikely(!log_buf_snapshot_state)) {
		/*
		 * Used rate limited to avoid deluge of messages, logs might be
		 * getting consumed by User at a slow rate.
		 */
		DRM_ERROR_RATELIMITED("no sub-buffer to capture logs\n");
		guc->log.capture_miss_count++;

		return;
	}

	/* Actual logs are present from the 2nd page */
	src_data += PAGE_SIZE;
	dst_data += PAGE_SIZE;

	for (type = GUC_ISR_LOG_BUFFER; type < GUC_MAX_LOG_BUFFER; type++) {
		/*
		 * Make a copy of the state structure, inside GuC log buffer
		 * (which is uncached mapped), on the stack to avoid reading
		 * from it multiple times.
		 */
		memcpy(&log_buf_state_local, log_buf_state,
		       sizeof(struct guc_log_buffer_state));
		buffer_size = guc_get_log_buffer_size(type);
		read_offset = log_buf_state_local.read_ptr;
		write_offset = log_buf_state_local.sampled_write_ptr;
		full_cnt = log_buf_state_local.buffer_full_cnt;

		/* Bookkeeping stuff */
		guc->log.flush_count[type] += log_buf_state_local.flush_to_file;
		new_overflow = guc_check_log_buf_overflow(guc, type, full_cnt);

		/* Update the state of shared log buffer */
		log_buf_state->read_ptr = write_offset;
		log_buf_state->flush_to_file = 0;
		log_buf_state++;

		/* First copy the state structure in snapshot buffer */
		memcpy(log_buf_snapshot_state, &log_buf_state_local,
		       sizeof(struct guc_log_buffer_state));

		/*
		 * The write pointer could have been updated by GuC firmware,
		 * after sending the flush interrupt to Host, for consistency
		 * set write pointer value to same value of sampled_write_ptr
		 * in the snapshot buffer.
		 */
		log_buf_snapshot_state->write_ptr = write_offset;
		log_buf_snapshot_state++;

		/* Now copy the actual logs. */
		if (unlikely(new_overflow)) {
			/* copy the whole buffer in case of overflow */
			read_offset = 0;
			write_offset = buffer_size;
		} else if (unlikely((read_offset > buffer_size) ||
				    (write_offset > buffer_size))) {
			DRM_ERROR("invalid log buffer state\n");
			/* copy whole buffer as offsets are unreliable */
			read_offset = 0;
			write_offset = buffer_size;
		}

		/* Just copy the newly written data */
		if (read_offset > write_offset) {
			i915_memcpy_from_wc(dst_data, src_data, write_offset);
			bytes_to_copy = buffer_size - read_offset;
		} else {
			bytes_to_copy = write_offset - read_offset;
		}
		i915_memcpy_from_wc(dst_data + read_offset,
				    src_data + read_offset, bytes_to_copy);

		src_data += buffer_size;
		dst_data += buffer_size;
	}

	guc_move_to_next_buf(guc);
}

static void capture_logs_work(struct work_struct *work)
{
	struct intel_guc *guc =
		container_of(work, struct intel_guc, log.flush_work);

	mutex_lock(&guc->log.lock);
	guc_log_capture_logs(guc);
	mutex_unlock(&guc->log.lock);
}

static bool guc_log_relay_enabled(struct intel_guc *guc)
{
	return guc->log.buf_addr != NULL;
}

static int guc_log_map(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	void *vaddr;
	int ret;

	if (!guc->log.vma)
		return -ENODEV;

	mutex_lock(&dev_priv->drm.struct_mutex);
	ret = i915_gem_object_set_to_wc_domain(guc->log.vma->obj, true);
	if (ret)
		return ret;

	/*
	 * Create a WC (Uncached for read) vmalloc mapping of log
	 * buffer pages, so that we can directly get the data
	 * (up-to-date) from memory.
	 */
	vaddr = i915_gem_object_pin_map(guc->log.vma->obj, I915_MAP_WC);
	if (IS_ERR(vaddr)) {
		DRM_ERROR("Couldn't map log buffer pages %d\n", ret);
		return PTR_ERR(vaddr);
	}
	mutex_unlock(&dev_priv->drm.struct_mutex);

	guc->log.buf_addr = vaddr;

	return 0;
}

static void guc_log_unmap(struct intel_guc *guc)
{
	lockdep_assert_held(&guc->log.lock);

	i915_gem_object_unpin_map(guc->log.vma->obj);
	guc->log.buf_addr = NULL;
}

void intel_guc_log_init_early(struct intel_guc *guc)
{
	mutex_init(&guc->log.lock);
	INIT_WORK(&guc->log.flush_work, capture_logs_work);
}

int guc_log_relay_create(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	struct rchan *guc_log_relay_chan;
	size_t n_subbufs, subbuf_size;
	int ret;

	lockdep_assert_held(&guc->log.lock);

	 /* Keep the size of sub buffers same as shared log buffer */
	subbuf_size = GUC_LOG_SIZE;

	/*
	 * Store up to 8 snapshots, which is large enough to buffer sufficient
	 * boot time logs and provides enough leeway to User, in terms of
	 * latency, for consuming the logs from relay. Also doesn't take
	 * up too much memory.
	 */
	n_subbufs = 8;

	/*
	 * Create a relay channel, so that we have buffers for storing
	 * the GuC firmware logs, the channel will be linked with a file
	 * later on when debugfs is registered.
	 */
	guc_log_relay_chan = relay_open("guc_log",
					dev_priv->drm.primary->debugfs_root,
					subbuf_size, n_subbufs,
					&relay_callbacks, dev_priv);
	if (!guc_log_relay_chan) {
		DRM_ERROR("Couldn't create relay chan for GuC logging\n");

		ret = -ENOMEM;
		goto err;
	}

	GEM_BUG_ON(guc_log_relay_chan->subbuf_size < subbuf_size);
	guc->log.relay_chan = guc_log_relay_chan;

	return 0;

err:
	/* logging will be off */
	i915_modparams.guc_log_level = 0;
	return ret;
}

void guc_log_relay_destroy(struct intel_guc *guc)
{
	lockdep_assert_held(&guc->log.lock);

	relay_close(guc->log.relay_chan);
	guc->log.relay_chan = NULL;
}

static void guc_log_capture_logs(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);

	guc_read_update_log_buffer(guc);

	/*
	 * Generally device is expected to be active only at this
	 * time, so get/put should be really quick.
	 */
	intel_runtime_pm_get(dev_priv);
	guc_log_flush_complete(guc);
	intel_runtime_pm_put(dev_priv);
}

int intel_guc_log_create(struct intel_guc *guc)
{
	struct i915_vma *vma;
	unsigned long offset;
	u32 flags;
	int ret;

	GEM_BUG_ON(guc->log.vma);

	vma = intel_guc_allocate_vma(guc, GUC_LOG_SIZE);
	if (IS_ERR(vma)) {
		ret = PTR_ERR(vma);
		goto err;
	}

	guc->log.vma = vma;

	/* each allocated unit is a page */
	flags = GUC_LOG_VALID | GUC_LOG_NOTIFY_ON_HALF_FULL |
		(GUC_LOG_DPC_PAGES << GUC_LOG_DPC_SHIFT) |
		(GUC_LOG_ISR_PAGES << GUC_LOG_ISR_SHIFT) |
		(GUC_LOG_CRASH_PAGES << GUC_LOG_CRASH_SHIFT);

	offset = guc_ggtt_offset(vma) >> PAGE_SHIFT; /* in pages */
	guc->log.flags = (offset << GUC_LOG_BUF_ADDR_SHIFT) | flags;

	return 0;

err:
	/* logging will be off */
	i915_modparams.guc_log_level = 0;
	return ret;
}

static void guc_log_flush_irq_enable(struct intel_guc *guc)
{
	spin_lock_irq(&guc->irq_lock);
	guc->msg_enabled_mask |= INTEL_GUC_RECV_MSG_FLUSH_LOG_BUFFER |
				 INTEL_GUC_RECV_MSG_CRASH_DUMP_POSTED;
	spin_unlock_irq(&guc->irq_lock);
}

static void guc_log_flush_irq_disable(struct intel_guc *guc)
{
	spin_lock_irq(&guc->irq_lock);
	guc->msg_enabled_mask &= ~(INTEL_GUC_RECV_MSG_FLUSH_LOG_BUFFER |
				   INTEL_GUC_RECV_MSG_CRASH_DUMP_POSTED);
	spin_unlock_irq(&guc->irq_lock);
}

void intel_guc_log_destroy(struct intel_guc *guc)
{
	i915_vma_unpin_and_release(&guc->log.vma);
}

int intel_guc_log_level_get(struct intel_guc *guc)
{
	GEM_BUG_ON(!guc->log.vma);
	GEM_BUG_ON(i915_modparams.guc_log_level < 0);

	return i915_modparams.guc_log_level;
}

#define GUC_LOG_IS_ENABLED(x)		(x > 0)
#define GUC_LOG_LEVEL_TO_VERBOSITY(x)	(GUC_LOG_IS_ENABLED(x) ? x - 1 : 0)
int intel_guc_log_level_set(struct intel_guc *guc, u64 val)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);
	int ret;

	BUILD_BUG_ON(GUC_LOG_VERBOSITY_MIN != 0);
	GEM_BUG_ON(!guc->log.vma);
	GEM_BUG_ON(i915_modparams.guc_log_level < 0);

	/*
	 * GuC is recognizing log levels starting from 0 to max, we're using 0
	 * as indication that logging should be disablded.
	 */
	if (GUC_LOG_LEVEL_TO_VERBOSITY(val) < GUC_LOG_VERBOSITY_MIN ||
	    GUC_LOG_LEVEL_TO_VERBOSITY(val) > GUC_LOG_VERBOSITY_MAX)
		return -EINVAL;

	mutex_lock(&guc->log.lock);

	if (i915_modparams.guc_log_level == val) {
		ret = 0;
		goto out_unlock;
	}

	intel_runtime_pm_get(dev_priv);
	ret = guc_log_control(guc, GUC_LOG_IS_ENABLED(val),
			      GUC_LOG_LEVEL_TO_VERBOSITY(val));
	intel_runtime_pm_put(dev_priv);
	if (ret)
		goto out_unlock;

	i915_modparams.guc_log_level = val;

out_unlock:
	mutex_unlock(&guc->log.lock);

	return ret;
}

int intel_guc_log_relay_open(struct intel_guc *guc)
{
	int ret;

	mutex_lock(&guc->log.lock);

	if (guc_log_relay_enabled(guc)) {
		ret = -EEXIST;
		goto out_unlock;
	}

	/*
	 * We require SSE 4.1 for fast reads from the GuC log buffer and
	 * it should be present on the chipsets supporting GuC based
	 * submisssions.
	 */
	if (!i915_has_memcpy_from_wc()) {
		ret = -EINVAL;
		goto out_unlock;
	}

	ret = guc_log_relay_create(guc);
	if (ret)
		goto out_unlock;

	ret = guc_log_map(guc);
	if (ret)
		goto out_relay;

	mutex_unlock(&guc->log.lock);

	guc_log_flush_irq_enable(guc);

	/*
	 * When GuC is logging without us relaying to userspace, we're ignoring
	 * the flush notification. This means that we need to unconditionally
	 * flush on relay enabling, since GuC only notifies us once.
	 */
	queue_work(guc->log.flush_wq, &guc->log.flush_work);

	return 0;

out_relay:
	guc_log_relay_destroy(guc);
out_unlock:
	mutex_unlock(&guc->log.lock);

	return ret;
}

void intel_guc_log_relay_flush(struct intel_guc *guc)
{
	struct drm_i915_private *dev_priv = guc_to_i915(guc);

	/*
	 * Before initiating the forceful flush, wait for any pending/ongoing
	 * flush to complete otherwise forceful flush may not actually happen.
	 */
	flush_work(&guc->log.flush_work);

	intel_runtime_pm_get(dev_priv);
	guc_log_flush(guc);
	intel_runtime_pm_put(dev_priv);

	mutex_lock(&guc->log.lock);
	/* GuC would have updated log buffer by now, so capture it */
	guc_log_capture_logs(guc);
	mutex_unlock(&guc->log.lock);
}

void intel_guc_log_relay_close(struct intel_guc *guc)
{
	GEM_BUG_ON(!guc_log_relay_enabled(guc));

	guc_log_flush_irq_disable(guc);
	flush_work(&guc->log.flush_work);

	mutex_lock(&guc->log.lock);
	guc_log_unmap(guc);
	guc_log_relay_destroy(guc);
	mutex_unlock(&guc->log.lock);
}
