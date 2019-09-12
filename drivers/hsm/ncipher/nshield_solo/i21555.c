/*
 *
 * i21555.c: nCipher PCI HSM intel 21555 command driver
 *
 * (c) nCipher Security Limited 2019
 *
 * history
 *
 * 09/10/2001 jsh  Original
 *
 */

#include "nfp_common.h"
#include "nfp_error.h"
#include "nfp_hostif.h"
#include "nfp_osif.h"
#include "i21555.h"
#include "nfp_cmd.h"
#include "nfpci.h"

/* started ------------------------------------------------------
 *
 * Check that device is ready to talk, by checking that
 * the i21555 has master enabled on its secondary interface
 */

static nfp_err i21555_started(nfp_cdev *pdev)
{
	unsigned int tmp32;
#ifdef CONFIGSPACE_DEBUG
	unsigned int reg32[64];
	int i;
#endif
	nfp_err ne;

	nfp_log(NFP_DBG2, "%s: entered", __func__);

#ifdef CONFIGSPACE_DEBUG
	/* Suck up all the registers */
	for (i = 0; i < 64; i++)
		ne = nfp_config_inl(pdev, i * 4, &reg32[i]);

	for (i = 0; i < 16; i++) {
		int j = i * 4;

		nfp_log(NFP_DBG3, "i21555 config reg %2x: %08x %08x %08x %08x",
			j * 4, reg32[j], reg32[j + 1], reg32[j + 2],
			reg32[j + 3]);
	}
#endif

	ne = nfp_config_inl(pdev, I21555_CFG_SEC_CMD_STATUS, &tmp32);
	if (ne) {
		/* succeed if PCI config reads are not implemented */
		if (ne == NFP_EUNKNOWN)
			return NFP_SUCCESS;
		nfp_log(NFP_DBG1, "%s: nfp_config_inl failed", __func__);
		return ne;
	}

	tmp32 = FROM_LE32_CONFIG(&tmp32) & 0xffff;

	if (tmp32 & CFG_CMD_MASTER) {
		nfp_log(NFP_DBG3, "%s: Yes %x", __func__, tmp32);
		return NFP_SUCCESS;
	} else {
		nfp_log(NFP_DBG1, "%s: device not started yet %x", __func__,
			tmp32);
		return NFP_ESTARTING;
	}
}

/* create ------------------------------------------------------- */

static nfp_err i21555_create(nfp_cdev *pdev)
{
	unsigned int tmp32;

	nfp_log(NFP_DBG2, "%s: entered", __func__);
	pdev->cmdctx =
		pdev; /* set our context to just be a pointer to our nfp_cdev */

	if (!pdev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}
	nfp_log(NFP_DBG2, "%s: enable doorbell", __func__);
	TO_LE32_MEM(&tmp32, I21555_DOORBELL_PRI_ENABLE);
	nfp_outl(pdev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_SET_MASK, tmp32);
	nfp_outl(pdev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_CLEAR_MASK, tmp32);
	return NFP_SUCCESS;
}

/* stop ------------------------------------------------------- */

static nfp_err i21555_destroy(void *ctx)
{
	nfp_cdev *pdev;
	unsigned int tmp32;

	nfp_log(NFP_DBG2, "%s: entered", __func__);

	pdev = (nfp_cdev *)ctx;
	if (!pdev) {
		nfp_log(NFP_DBG1, "%s: NULL pdev", __func__);
		return NFP_ENODEV;
	}
	if (!pdev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}
	TO_LE32_MEM(&tmp32, I21555_DOORBELL_PRI_DISABLE);
	nfp_outl(pdev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_SET_MASK, tmp32);
	nfp_outl(pdev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_CLEAR_MASK, tmp32);

	return NFP_SUCCESS;
}

/* open ------------------------------------------------------- */

static nfp_err i21555_open(void *ctx)
{
	(void)ctx;

	nfp_log(NFP_DBG2, "%s: entered", __func__);

	return NFP_SUCCESS;
}

/* close ------------------------------------------------------- */

static nfp_err i21555_close(void *ctx)
{
	(void)ctx;
	nfp_log(NFP_DBG2, "%s: entered", __func__);

	return NFP_SUCCESS;
}

/* isr ------------------------------------------------------- */

static nfp_err i21555_isr(void *ctx, int *handled)
{
	nfp_cdev *pdev;
	unsigned short doorbell;
	unsigned short tmp16;

	nfp_log(NFP_DBG3, "%s: entered", __func__);

	*handled = 0;
	pdev = (nfp_cdev *)ctx;
	if (!pdev) {
		nfp_log(NFP_DBG1, "%s: NULL pdev", __func__);
		return NFP_ENODEV;
	}

	pdev->stats.isr++;

	if (!pdev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}

	/* This interrupt may not be from our module, so check that it
	 * actually is us before handling it.
	 */
	doorbell = nfp_inw(pdev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_SET);
	doorbell = FROM_LE16_MEM(&doorbell);
	while (doorbell && doorbell != 0xffff) {
		*handled = 1;
		/* service interrupts */
		if (doorbell & (NFAST_INT_DEVICE_WRITE_OK |
				NFAST_INT_DEVICE_WRITE_FAILED)) {
			pdev->stats.isr_write++;
			TO_LE16_MEM(&tmp16,
				    NFAST_INT_DEVICE_WRITE_OK |
				    NFAST_INT_DEVICE_WRITE_FAILED);
			nfp_outw(pdev, CSR_BAR,
				 I21555_OFFSET_DOORBELL_PRI_CLEAR, tmp16);

			nfp_log(NFP_DBG2,
				"%s: write done interrupt, ok = %d.", __func__,
				doorbell & NFAST_INT_DEVICE_WRITE_OK ? 1 : 0);

			nfp_write_complete(pdev->dev, doorbell &
					   NFAST_INT_DEVICE_WRITE_OK ? 1 : 0);
		}

		if (doorbell &
		    (NFAST_INT_DEVICE_READ_OK |
		     NFAST_INT_DEVICE_READ_FAILED)) {
			pdev->stats.isr_read++;
			TO_LE16_MEM(&tmp16,
				    NFAST_INT_DEVICE_READ_OK |
				    NFAST_INT_DEVICE_READ_FAILED);
			nfp_outw(pdev, CSR_BAR,
				 I21555_OFFSET_DOORBELL_PRI_CLEAR, tmp16);

			nfp_log(NFP_DBG2,
				"%s: read ack interrupt, ok = %d.", __func__,
				doorbell & NFAST_INT_DEVICE_READ_OK ? 1 : 0);
			nfp_read_complete(pdev->dev, doorbell &
					  NFAST_INT_DEVICE_READ_OK ? 1 : 0);
		}

		if (doorbell &
		    ~(NFAST_INT_DEVICE_READ_OK | NFAST_INT_DEVICE_READ_FAILED |
		      NFAST_INT_DEVICE_WRITE_OK |
		      NFAST_INT_DEVICE_WRITE_FAILED)) {
			TO_LE16_MEM(&tmp16, doorbell);
			nfp_outw(pdev, CSR_BAR,
				 I21555_OFFSET_DOORBELL_PRI_CLEAR, tmp16);
			nfp_log(NFP_DBG1, "%s: unexpected interrupt %x",
				__func__, doorbell);
		}
		doorbell = nfp_inw(pdev, CSR_BAR,
				   I21555_OFFSET_DOORBELL_PRI_SET);
		doorbell = FROM_LE16_MEM(&doorbell);
	}
	nfp_log(NFP_DBG3, "%s: exiting", __func__);
	return 0;
}

/* write ------------------------------------------------------- */

static nfp_err i21555_write(unsigned int addr, const char *block, int len,
			    void *ctx)
{
	nfp_cdev *cdev;
	unsigned int hdr[2];
	nfp_err ne;
	unsigned short tmp16;
	unsigned int tmp32;

	/* Note: 'addr' not used in this implementation,
	 * and this is to silence the Windows build
	 */
	(void)addr;

	nfp_log(NFP_DBG2, "%s: entered", __func__);

	cdev = (nfp_cdev *)ctx;
	if (!cdev) {
		nfp_log(NFP_DBG1, "%s: NULL cdev", __func__);
		return NFP_ENODEV;
	}

	cdev->stats.write_fail++;

	if (!cdev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}

	ne = i21555_started(cdev);
	if (ne) {
		if (ne != NFP_ESTARTING) {
			nfp_log(NFP_DBG1,
				"%s: i21555_started failed", __func__);
		}
		return ne;
	}

	nfp_log(NFP_DBG3, "%s: cdev->bar[ MEMBAR2 ]= %p", __func__,
		cdev->bar[MEMBAR2]);
	nfp_log(NFP_DBG3, "%s: cdev->bar[ CSR_BAR ]= %p", __func__,
		cdev->bar[CSR_BAR]);
	nfp_log(NFP_DBG3, "%s: block len %d", __func__, len);
	ne = nfp_copy_from_user_to_dev(cdev, MEMBAR2, NFPCI_JOBS_WR_DATA,
				       block, len);
	if (ne) {
		nfp_log(NFP_DBG1,
			"%s: nfp_copy_from_user_to_dev failed", __func__);
		return ne;
	}
	TO_LE32_MEM(&hdr[0], NFPCI_JOB_CONTROL);
	TO_LE32_MEM(&hdr[1], len);
	ne = nfp_copy_to_dev(cdev, MEMBAR2, NFPCI_JOBS_WR_CONTROL,
			     (const char *)hdr, 8);
	if (ne) {
		nfp_log(NFP_DBG1, "%s: nfp_copy_to_dev failed", __func__);
		return ne;
	}
	ne = nfp_copy_from_dev(cdev, MEMBAR2, NFPCI_JOBS_WR_LENGTH,
			       (char *)hdr, 4);
	if (ne) {
		nfp_log(NFP_DBG1, "%s: nfp_copy_from_dev failed", __func__);
		return ne;
	}

	TO_LE32_MEM(&tmp32, len);
	if (hdr[0] != tmp32) {
		nfp_log(NFP_DBG1, "%s: length not written", __func__);
		return NFP_EIO;
	}
	TO_LE16_MEM(&tmp16, NFAST_INT_HOST_WRITE_REQUEST >> 16);
	nfp_outw(cdev, CSR_BAR, I21555_OFFSET_DOORBELL_SEC_SET, tmp16);

	cdev->stats.write_fail--;
	cdev->stats.write_block++;
	cdev->stats.write_byte += len;

	nfp_log(NFP_DBG2, "%s: done", __func__);
	return NFP_SUCCESS;
}

/* read ------------------------------------------------------- */

static nfp_err i21555_read(char *block, int len, void *ctx, int *rcount)
{
	nfp_cdev *cdev;
	nfp_err ne;
	int count;

	nfp_log(NFP_DBG2, "%s: entered", __func__);
	*rcount = 0;

	cdev = (nfp_cdev *)ctx;
	if (!cdev) {
		nfp_log(NFP_DBG1, "%s: NULL pdev", __func__);
		return NFP_ENODEV;
	}

	cdev->stats.read_fail++;

	if (!cdev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}

	ne = i21555_started(cdev);
	if (ne) {
		if (ne != NFP_ESTARTING)
			nfp_log(NFP_DBG1, "%s: i21555_started failed",
				__func__);

		return ne;
	}

	ne = nfp_copy_from_dev(cdev, MEMBAR2, NFPCI_JOBS_RD_LENGTH,
			       (char *)&count, 4);
	if (ne) {
		nfp_log(NFP_DBG1, "%s: nfp_copy_from_dev failed.", __func__);
		return ne;
	}
	count = FROM_LE32_MEM(&count);
	if (count < 0 || count > len) {
		nfp_log(NFP_DBG1,
			"%s: bad byte count (%d) from device", __func__,
			count);
		return NFP_EIO;
	}
	ne = nfp_copy_to_user_from_dev(cdev, MEMBAR2, NFPCI_JOBS_RD_DATA,
				       block, count);
	if (ne) {
		nfp_log(NFP_DBG1, "%s: nfp_copy_to_user failed.", __func__);
		return ne;
	}
	nfp_log(NFP_DBG2, "%s: done", __func__);
	*rcount = count;
	cdev->stats.read_fail--;
	cdev->stats.read_block++;
	cdev->stats.read_byte += len;
	return NFP_SUCCESS;
}

/* chupdate  ------------------------------------------------------- */

static nfp_err i21555_chupdate(char *data, int len, void *ctx)
{
	(void)ctx;
	(void)len;
	(void)data;
	nfp_log(NFP_DBG1, "%s: NYI", __func__);
	return NFP_SUCCESS;
}

/* ensure reading -------------------------------------------------- */

static nfp_err i21555_ensure_reading(unsigned int addr, int len, void *ctx,
				     int lock_flag)
{
	nfp_cdev *cdev;
	unsigned int hdr[3];
	unsigned short tmp16;
	unsigned int tmp32;
	nfp_err ne;
	int hdr_len;

	/* not used with i21555, mentioned to defeat build warnings
	 */
	(void)lock_flag;

	nfp_log(NFP_DBG2, "%s: entered", __func__);

	cdev = (nfp_cdev *)ctx;
	if (!cdev) {
		nfp_log(NFP_DBG1, "%s: NULL cdev", __func__);
		return NFP_ENODEV;
	}

	cdev->stats.ensure_fail++;

	if (!cdev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__,
			CSR_BAR);
		return NFP_ENOMEM;
	}

	ne = i21555_started(cdev);
	if (ne) {
		if (ne != NFP_ESTARTING) {
			nfp_log(NFP_DBG1,
				"%s: i21555_started failed", __func__
				);
		}
		return ne;
	}

	nfp_log(NFP_DBG3, "%s: pdev->bar[ MEMBAR2 ]= %p", __func__,
		cdev->bar[MEMBAR2]);
	nfp_log(NFP_DBG3, "%s: pdev->bar[ CSR_BAR ]= %p", __func__,
		cdev->bar[CSR_BAR]);
	if (addr) {
		nfp_log(NFP_DBG3, "%s: new format, addr %p", __func__,
			addr);
		TO_LE32_MEM(&hdr[0], NFPCI_JOB_CONTROL_PCI_PUSH);
		TO_LE32_MEM(&hdr[1], len);
		TO_LE32_MEM(&hdr[2], addr);
		hdr_len = 12;
	} else {
		TO_LE32_MEM(&hdr[0], NFPCI_JOB_CONTROL);
		TO_LE32_MEM(&hdr[1], len);
		hdr_len = 8;
	}

	ne = nfp_copy_to_dev(cdev, MEMBAR2, NFPCI_JOBS_RD_CONTROL,
			     (const char *)hdr, hdr_len);
	if (ne) {
		nfp_log(NFP_DBG1,
			"%s: nfp_copy_to_dev failed", __func__);
		return ne;
	}

	ne = nfp_copy_from_dev(cdev, MEMBAR2, NFPCI_JOBS_RD_LENGTH,
			       (char *)hdr, 4);
	if (ne) {
		nfp_log(NFP_DBG1,
			"%s: nfp_copy_from_dev failed", __func__);
		return ne;
	}

	TO_LE32_MEM(&tmp32, len);

	if (hdr[0] != tmp32) {
		nfp_log(NFP_DBG1, "%s: len not written", __func__);
		return NFP_EIO;
	}
	TO_LE16_MEM(&tmp16, NFAST_INT_HOST_READ_REQUEST >> 16);
	nfp_outw(cdev, CSR_BAR, I21555_OFFSET_DOORBELL_SEC_SET, tmp16);

	cdev->stats.ensure_fail--;
	cdev->stats.ensure++;

	return NFP_SUCCESS;
}

/* set control register ----------------------------------------- */

static nfp_err i21555_set_control(const struct nfdev_control_str *control,
				  void *ctx)
{
	nfp_cdev *cdev = (nfp_cdev *)ctx;
	u32 control_flipped;

	nfp_log(NFP_DBG3, "%s: entered", __func__);
	if (!cdev) {
		nfp_log(NFP_DBG1, "%s: NULL pdev", __func__);
		return NFP_ENODEV;
	}
	if (!cdev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}
	TO_LE32_MEM(&control_flipped, control->control);
	nfp_outl(cdev, CSR_BAR, I21555_SCRATCHPAD_REGISTER_CONTROL,
		 control_flipped);
	return NFP_SUCCESS;
}

/* get status/error registers ----------------------------------- */

static nfp_err i21555_get_status(struct nfdev_status_str *status, void *ctx)
{
	nfp_cdev *cdev = (nfp_cdev *)ctx;
	u32 status_flipped;
	u32 *error = (u32 *)status->error;

	nfp_log(NFP_DBG3, "%s: entered", __func__);
	if (!cdev) {
		nfp_log(NFP_DBG1, "%s: NULL cdev", __func__);
		return NFP_ENODEV;
	}
	if (!cdev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}
	status_flipped =
		nfp_inl(cdev, CSR_BAR, I21555_SCRATCHPAD_REGISTER_STATUS);
	status->status = FROM_LE32_MEM(&status_flipped);
	error[0] = nfp_inl(cdev, CSR_BAR, I21555_SCRATCHPAD_REGISTER_ERROR_LO);
	error[1] = nfp_inl(cdev, CSR_BAR, I21555_SCRATCHPAD_REGISTER_ERROR_HI);
	return NFP_SUCCESS;
}

/* command device structure ------------------------------------- */

const struct nfpcmd_dev i21555_cmddev = {
	.name = "nCipher Gen 2 PCI",
	.vendorid = PCI_VENDOR_ID_INTEL,
	.deviceid = PCI_DEVICE_ID_INTEL_21555,
	.sub_vendorid = PCI_VENDOR_ID_NCIPHER,
	.sub_deviceid = PCI_SUBSYSTEM_ID_NFAST_REV1,
	.bar_sizes = BAR_SIZES,
	.flags = NFP_CMD_FLG_NEED_IOBUF,
	.max_ifvers = NFDEV_IF_PCI_PUSH,
	.create = i21555_create,
	.destroy = i21555_destroy,
	.open = i21555_open,
	.close = i21555_close,
	.isr = i21555_isr,
	.write_block = i21555_write,
	.read_block = i21555_read,
	.channel_update = i21555_chupdate,
	.ensure_reading = i21555_ensure_reading,
	.debug = NULL,
	.setcontrol = i21555_set_control,
	.getstatus = i21555_get_status,
};
