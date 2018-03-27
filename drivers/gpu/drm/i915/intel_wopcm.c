/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2017-2018 Intel Corporation
 */

#include "intel_wopcm.h"
#include "i915_drv.h"

/**
 * DOC: WOPCM Layout
 *
 * The layout of the WOPCM will be fixed after writing to GuC WOPCM size and
 * offset registers whose values are calculated and determined by HuC/GuC
 * firmware size and set of hardware requirements/restrictions as shown below:
 *
 * ::
 *
 *    +=========> +====================+ <== WOPCM Top
 *    ^           |  HW contexts RSVD  |
 *    |     +===> +====================+ <== GuC WOPCM Top
 *    |     ^     |                    |
 *    |     |     |                    |
 *    |     |     |                    |
 *    |    GuC    |                    |
 *    |   WOPCM   |                    |
 *    |    Size   +--------------------+
 *  WOPCM   |     |    GuC FW RSVD     |
 *    |     |     +--------------------+
 *    |     |     |   GuC Stack RSVD   |
 *    |     |     +------------------- +
 *    |     v     |   GuC WOPCM RSVD   |
 *    |     +===> +====================+ <== GuC WOPCM base
 *    |           |     WOPCM RSVD     |
 *    |           +------------------- + <== HuC Firmware Top
 *    v           |      HuC FW        |
 *    +=========> +====================+ <== WOPCM Base
 *
 * GuC accessible WOPCM starts at GuC WOPCM base and ends at GuC WOPCM top.
 * The top part of the WOPCM is reserved for hardware contexts (e.g. RC6
 * context).
 */

/* Default WOPCM size 1MB. */
#define GEN9_WOPCM_SIZE			(1024 * 1024)
/* 16KB WOPCM (RSVD WOPCM) is reserved from HuC firmware top. */
#define WOPCM_RESERVED_SIZE		(16 * 1024)

/* 16KB reserved at the beginning of GuC WOPCM. */
#define GUC_WOPCM_RESERVED		(16 * 1024)
/* 8KB from GUC_WOPCM_RESERVED is reserved for GuC stack. */
#define GUC_WOPCM_STACK_RESERVED	(8 * 1024)

/* GuC WOPCM Offset value needs to be aligned to 16KB. */
#define GUC_WOPCM_OFFSET_ALIGNMENT	(1UL << GUC_WOPCM_OFFSET_SHIFT)

/* 24KB at the end of WOPCM is reserved for RC6 CTX on BXT. */
#define BXT_WOPCM_RC6_CTX_RESERVED	(24 * 1024)
/* 36KB WOPCM reserved at the end of WOPCM on CNL. */
#define CNL_WOPCM_HW_CTX_RESERVED	(36 * 1024)

/* 128KB from GUC_WOPCM_RESERVED is reserved for FW on Gen9. */
#define GEN9_GUC_FW_RESERVED	(128 * 1024)
#define GEN9_GUC_WOPCM_OFFSET	(GUC_WOPCM_RESERVED + GEN9_GUC_FW_RESERVED)
/* 256KB from GUC_WOPCM_RESERVED is reserved for FW on Gen10. */
#define GEN10_GUC_FW_RESERVED	(256 * 1024)
#define GEN10_GUC_WOPCM_OFFSET	(GUC_WOPCM_RESERVED + GEN10_GUC_FW_RESERVED)

static inline u32 context_reserved_size(struct drm_i915_private *i915)
{
	if (IS_GEN9_LP(i915))
		return BXT_WOPCM_RC6_CTX_RESERVED;
	else if (INTEL_GEN(i915) >= 10)
		return CNL_WOPCM_HW_CTX_RESERVED;
	else
		return 0;
}

static inline u32 guc_fw_size_in_wopcm(u32 guc_fw_size)
{
	return ALIGN(guc_fw_size + GUC_WOPCM_RESERVED +
		     GUC_WOPCM_STACK_RESERVED, PAGE_SIZE);
}

static inline u32 huc_fw_size_in_wopcm(u32 huc_fw_size)
{
	return huc_fw_size + WOPCM_RESERVED_SIZE;
}

static u32
__additional_size_for_dword_gap(struct drm_i915_private *i915,
				u32 guc_wopcm_base,
				u32 guc_wopcm_size)
{
	s32 additional_size = 0;
	u32 wopcm_offset;

	if (INTEL_GEN(i915) >= 10)
		wopcm_offset = GEN10_GUC_WOPCM_OFFSET;
	else
		wopcm_offset = GEN9_GUC_WOPCM_OFFSET;

	/*
	 * GuC WOPCM size shall be at least a dword larger than the offset from
	 * WOPCM base (GuC WOPCM offset from WOPCM base + GUC_WOPCM_OFFSET)
	 * due to hardware limitation on Gen9.
	 */
	if (IS_GEN9(i915) || IS_CNL_REVID(i915, CNL_REVID_A0, CNL_REVID_A0))
		additional_size = guc_wopcm_base + wopcm_offset +
				  sizeof(u32) - guc_wopcm_size;

	if (additional_size < 0)
		additional_size = 0;

	return additional_size;
}

static u32
__additional_size_for_huc(struct drm_i915_private *i915,
			  u32 guc_wopcm_size, u32 huc_fw_size)
{
	s32 additional_size = 0;

	/*
	 * On Gen9 & CNL A0, hardware requires the total available GuC WOPCM
	 * size to be larger than or equal to HuC firmware size. Otherwise,
	 * firmware uploading would fail.
	 */
	if (IS_GEN9(i915) || IS_CNL_REVID(i915, CNL_REVID_A0, CNL_REVID_A0))
		additional_size = huc_fw_size -
				  (guc_wopcm_size - GUC_WOPCM_RESERVED);

	if (additional_size < 0)
		additional_size = 0;

	return additional_size;
}

static u32
additional_size_for_hw_restrictions(struct drm_i915_private *i915,
				    u32 guc_wopcm_base, u32 guc_wopcm_size,
				    u32 huc_fw_size)
{
	u32 gap_size, huc_size;

	gap_size = __additional_size_for_dword_gap(i915, guc_wopcm_base,
						   guc_wopcm_size);

	huc_size = __additional_size_for_huc(i915, guc_wopcm_size, huc_fw_size);

	return max(gap_size, huc_size);
}

static inline void
__guc_region_grow(struct intel_wopcm *wopcm, u32 size)
{
	/*
	 * We're growing guc region in the direction of lower addresses.
	 * We need to use multiples of base alignment, because it has more
	 * strict alignment rules.
	 */
	size = DIV_ROUND_UP(size, 2);
	size = ALIGN(size, GUC_WOPCM_OFFSET_ALIGNMENT);

	wopcm->guc.base -= size;
	wopcm->guc.size += size;
}

static void wopcm_adjust_for_hw_restrictions(struct intel_wopcm *wopcm)
{
	struct drm_i915_private *i915 = wopcm_to_i915(wopcm);
	u32 huc_fw_size = intel_uc_fw_get_upload_size(&i915->huc.fw);
	u32 size;

	GEM_BUG_ON(!wopcm->guc.base);
	GEM_BUG_ON(!wopcm->guc.size);

	size = additional_size_for_hw_restrictions(i915, wopcm->guc.base,
						   wopcm->guc.size,
						   huc_fw_size);

	__guc_region_grow(wopcm, size);
}

static void wopcm_guc_region_init(struct intel_wopcm *wopcm)
{
	struct drm_i915_private *dev_priv = wopcm_to_i915(wopcm);
	u32 guc_fw_size = intel_uc_fw_get_upload_size(&dev_priv->guc.fw);
	u32 ctx_rsvd = context_reserved_size(dev_priv);

	GEM_BUG_ON(!wopcm->size);

	wopcm->guc.size = guc_fw_size_in_wopcm(guc_fw_size);

	wopcm->guc.base = ALIGN_DOWN(wopcm->size - wopcm->guc.size - ctx_rsvd,
				     GUC_WOPCM_OFFSET_ALIGNMENT);

	wopcm_adjust_for_hw_restrictions(wopcm);
}

/**
 * intel_wopcm_init_early() - Early initialization of the WOPCM.
 * @wopcm: pointer to intel_wopcm.
 *
 * Setup the size of WOPCM and partition the GuC region.
 */
void intel_wopcm_init_early(struct intel_wopcm *wopcm)
{
	struct drm_i915_private *i915 = wopcm_to_i915(wopcm);

	if (!HAS_GUC(i915) || !USES_GUC(i915))
		return;

	wopcm->size = GEN9_WOPCM_SIZE;

	wopcm_guc_region_init(wopcm);

	DRM_DEBUG_DRIVER("WOPCM size: %uKiB\n", wopcm->size / 1024);
	DRM_DEBUG_DRIVER("GuC WOPCM Region: [%uKiB, %uKiB)\n",
			 wopcm->guc.base / 1024,
			 (wopcm->guc.base + wopcm->guc.size) / 1024);
}

static int check_huc_fw_fits(struct intel_wopcm *wopcm, u32 huc_fw_size)
{
	if (huc_fw_size_in_wopcm(huc_fw_size) > wopcm->guc.base) {
		DRM_ERROR("Need %uKiB WOPCM for HuC, %uKiB available.\n",
			  huc_fw_size_in_wopcm(huc_fw_size) / 1024,
			  wopcm->guc.base / 1024);
		return -E2BIG;
	}

	return 0;
}

static int check_guc_fw_fits(struct intel_wopcm *wopcm, u32 guc_fw_size)
{
	if (guc_fw_size_in_wopcm(guc_fw_size) > wopcm->guc.size) {
		DRM_ERROR("Need %uKiB WOPCM for GuC, %uKiB available.\n",
			  huc_fw_size_in_wopcm(guc_fw_size) / 1024,
			  wopcm->guc.size / 1024);
		return -E2BIG;
	}

	return 0;
}

static int check_ctx_rsvd_fits(struct intel_wopcm *wopcm, u32 ctx_rsvd)
{
	if ((wopcm->guc.base + wopcm->guc.size + ctx_rsvd) > wopcm->size) {
		DRM_ERROR("GuC WOPCM base (%uKiB) is too big.\n",
			  wopcm->guc.base / 1024);
		return -E2BIG;
	}

	return 0;
}

static bool wopcm_check_components_fit(struct intel_wopcm *wopcm)
{
	struct drm_i915_private *i915 = wopcm_to_i915(wopcm);
	u32 huc_fw_size = intel_uc_fw_get_upload_size(&i915->huc.fw);
	u32 guc_fw_size = intel_uc_fw_get_upload_size(&i915->guc.fw);
	u32 ctx_rsvd = context_reserved_size(i915);
	int err;

	err = check_huc_fw_fits(wopcm, huc_fw_size);
	if (err)
		return err;

	err = check_guc_fw_fits(wopcm, guc_fw_size);
	if (err)
		return err;

	err = check_ctx_rsvd_fits(wopcm, ctx_rsvd);
	if (err)
		return err;

	return 0;
}

/**
 * intel_wopcm_init() - Initialize the WOPCM structure.
 * @wopcm: pointer to intel_wopcm.
 *
 * This function will partition WOPCM space based on GuC and HuC firmware sizes
 * and will allocate max remaining for use by GuC. This function will also
 * enforce platform dependent hardware restrictions on GuC WOPCM offset and
 * size. It will fail the WOPCM init if any of these checks were failed, so that
 * the following GuC firmware uploading would be aborted.
 *
 * Return: 0 on success, non-zero error code on failure.
 */
int intel_wopcm_init(struct intel_wopcm *wopcm)
{
	int err;

	if (!wopcm->size)
		return 0;

	err = wopcm_check_components_fit(wopcm);
	if (err)
		return err;

	return 0;
}

static inline int write_and_verify(struct drm_i915_private *dev_priv,
				   i915_reg_t reg, u32 val, u32 mask,
				   u32 locked_bit)
{
	u32 reg_val;

	GEM_BUG_ON(val & ~mask);

	I915_WRITE(reg, val);

	reg_val = I915_READ(reg);

	return (reg_val & mask) != (val | locked_bit) ? -EIO : 0;
}

/**
 * intel_wopcm_init_hw() - Setup GuC WOPCM registers.
 * @wopcm: pointer to intel_wopcm.
 *
 * Setup the GuC WOPCM size and offset registers with the calculated values. It
 * will verify the register values to make sure the registers are locked with
 * correct values.
 *
 * Return: 0 on success. -EIO if registers were locked with incorrect values.
 */
int intel_wopcm_init_hw(struct intel_wopcm *wopcm)
{
	struct drm_i915_private *dev_priv = wopcm_to_i915(wopcm);
	u32 huc_agent;
	u32 mask;
	int err;

	if (!wopcm->size)
		return 0;

	GEM_BUG_ON(!wopcm->guc.size);
	GEM_BUG_ON(!wopcm->guc.base);

	err = write_and_verify(dev_priv, GUC_WOPCM_SIZE, wopcm->guc.size,
			       GUC_WOPCM_SIZE_MASK | GUC_WOPCM_SIZE_LOCKED,
			       GUC_WOPCM_SIZE_LOCKED);
	if (err)
		goto err_out;

	huc_agent = USES_HUC(dev_priv) ? HUC_LOADING_AGENT_GUC : 0;
	mask = GUC_WOPCM_OFFSET_MASK | GUC_WOPCM_OFFSET_VALID | huc_agent;
	err = write_and_verify(dev_priv, DMA_GUC_WOPCM_OFFSET,
			       wopcm->guc.base | huc_agent, mask,
			       GUC_WOPCM_OFFSET_VALID);
	if (err)
		goto err_out;

	return 0;

err_out:
	DRM_ERROR("Failed to init WOPCM registers:\n");
	DRM_ERROR("DMA_GUC_WOPCM_OFFSET=%#x\n",
		  I915_READ(DMA_GUC_WOPCM_OFFSET));
	DRM_ERROR("GUC_WOPCM_SIZE=%#x\n", I915_READ(GUC_WOPCM_SIZE));

	return err;
}
