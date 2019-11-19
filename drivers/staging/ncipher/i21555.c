// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * i21555.c: nCipher PCI HSM intel 21555 command driver
 *
 */

#include "solo.h"
#include "i21555.h"

/* pci fixed length accessors. ----------------------------------
 * The below functions are used predominantly
 * to access CSR registers in pci memory space.
 */
u32 nfp_inl(struct nfp_dev *ndev, int bar, int offset)
{
	nfp_log(NFP_DBG3, "%s: addr %p", __func__, ndev->bar[bar] + offset);
	return ioread32(ndev->bar[bar] + offset);
}

u16 nfp_inw(struct nfp_dev *ndev, int bar, int offset)
{
	nfp_log(NFP_DBG3, "%s: addr %p", __func__, ndev->bar[bar] + offset);
	return ioread16(ndev->bar[bar] + offset);
}

void nfp_outl(struct nfp_dev *ndev, int bar, int offset, u32 data)
{
	nfp_log(NFP_DBG3, "%s: addr %p, data %x", __func__,
		ndev->bar[bar] + offset, data);
	iowrite32(data, ndev->bar[bar] + offset);
}

void nfp_outw(struct nfp_dev *ndev, int bar, int offset, u16 data)
{
	nfp_log(NFP_DBG3, "%s: addr %p, data %x", __func__,
		ndev->bar[bar] + offset, data);
	iowrite16(data, ndev->bar[bar] + offset);
}

int nfp_config_inl(struct nfp_dev *ndev, int offset, u32 *res)
{
	if (!ndev || !ndev->pcidev)
		return NFP_ENODEV;
	pci_read_config_dword(ndev->pcidev, offset, res);
	return 0;
}

/* started ------------------------------------------------------
 *
 * Check that device is ready to talk, by checking that
 * the i21555 has master enabled on its secondary interface
 */

static int i21555_started(struct nfp_dev *ndev)
{
	u32 tmp32;
#ifdef CONFIGSPACE_DEBUG
	u32 reg32[64];
	int i;
#endif
	int ne;

	nfp_log(NFP_DBG2, "%s: entered", __func__);

#ifdef CONFIGSPACE_DEBUG
	/* Suck up all the registers */
	for (i = 0; i < 64; i++)
		ne = nfp_config_inl(ndev, i * 4, &reg32[i]);

	for (i = 0; i < 16; i++) {
		int j = i * 4;

		nfp_log(NFP_DBG3, "i21555 config reg %2x: %08x %08x %08x %08x",
			j * 4, reg32[j], reg32[j + 1], reg32[j + 2],
			reg32[j + 3]);
	}
#endif

	ne = nfp_config_inl(ndev, I21555_CFG_SEC_CMD_STATUS, &tmp32);
	if (ne) {
		/* succeed if PCI config reads are not implemented */
		if (ne == NFP_EUNKNOWN)
			return NFP_SUCCESS;
		nfp_log(NFP_DBG1, "%s: nfp_config_inl failed", __func__);
		return ne;
	}

	tmp32 = le32_to_cpu(tmp32) & 0xffff;

	ne = NFP_SUCCESS;
	if (tmp32 & CFG_CMD_MASTER) {
		nfp_log(NFP_DBG3, "%s: Yes %x", __func__, tmp32);
	} else {
		nfp_log(NFP_DBG1, "%s: device not started yet %x", __func__,
			tmp32);
		ne = NFP_ESTARTING;
	}
	return ne;
}

/* create ------------------------------------------------------- */

static int i21555_create(struct nfp_dev *ndev)
{
	u32 tmp32;

	nfp_log(NFP_DBG2, "%s: entered", __func__);

	/* set our context to just be a pointer to our struct nfp_dev */
	ndev->cmdctx = ndev;

	if (!ndev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}
	nfp_log(NFP_DBG2, "%s: enable doorbell", __func__);
	tmp32 = cpu_to_le32(I21555_DOORBELL_PRI_ENABLE);
	nfp_outl(ndev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_SET_MASK, tmp32);
	nfp_outl(ndev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_CLEAR_MASK, tmp32);
	return NFP_SUCCESS;
}

/* stop ------------------------------------------------------- */

static int i21555_destroy(void *ctx)
{
	struct nfp_dev *ndev;
	u32 tmp32;

	nfp_log(NFP_DBG2, "%s: entered", __func__);

	ndev = (struct nfp_dev *)ctx;
	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: NULL ndev", __func__);
		return NFP_ENODEV;
	}
	if (!ndev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}
	tmp32 = cpu_to_le32(I21555_DOORBELL_PRI_DISABLE);
	nfp_outl(ndev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_SET_MASK, tmp32);
	nfp_outl(ndev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_CLEAR_MASK, tmp32);

	return NFP_SUCCESS;
}

/* open ------------------------------------------------------- */

static int i21555_open(void *ctx)
{
	nfp_log(NFP_DBG2, "%s: entered", __func__);

	return NFP_SUCCESS;
}

/* close ------------------------------------------------------- */

static int i21555_close(void *ctx)
{
	nfp_log(NFP_DBG2, "%s: entered", __func__);

	return NFP_SUCCESS;
}

/* isr ------------------------------------------------------- */

static int i21555_isr(void *ctx, int *handled)
{
	struct nfp_dev *ndev;
	u16 doorbell;
	u16 tmp16;

	nfp_log(NFP_DBG3, "%s: entered", __func__);

	*handled = 0;
	ndev = (struct nfp_dev *)ctx;
	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: NULL ndev", __func__);
		return NFP_ENODEV;
	}

	ndev->stats.isr++;

	if (!ndev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}

	/* This interrupt may not be from our module, so check that it
	 * actually is us before handling it.
	 */
	doorbell = nfp_inw(ndev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_SET);
	doorbell = le16_to_cpu(doorbell);
	while (doorbell && doorbell != 0xffff) {
		*handled = 1;
		/* service interrupts */
		if (doorbell & (NFAST_INT_DEVICE_WRITE_OK |
				NFAST_INT_DEVICE_WRITE_FAILED)) {
			ndev->stats.isr_write++;
			tmp16 = cpu_to_le16(NFAST_INT_DEVICE_WRITE_OK |
				    NFAST_INT_DEVICE_WRITE_FAILED);
			nfp_outw(ndev, CSR_BAR,
				 I21555_OFFSET_DOORBELL_PRI_CLEAR, tmp16);

			nfp_log(NFP_DBG2,
				"%s: write done interrupt, ok = %d.", __func__,
				doorbell & NFAST_INT_DEVICE_WRITE_OK ? 1 : 0);

			nfp_write_complete(ndev, doorbell &
					   NFAST_INT_DEVICE_WRITE_OK ? 1 : 0);
		}

		if (doorbell &
		    (NFAST_INT_DEVICE_READ_OK |
		     NFAST_INT_DEVICE_READ_FAILED)) {
			ndev->stats.isr_read++;
			tmp16 = cpu_to_le16(NFAST_INT_DEVICE_READ_OK |
				    NFAST_INT_DEVICE_READ_FAILED);
			nfp_outw(ndev, CSR_BAR,
				 I21555_OFFSET_DOORBELL_PRI_CLEAR, tmp16);

			nfp_log(NFP_DBG2,
				"%s: read ack interrupt, ok = %d.", __func__,
				doorbell & NFAST_INT_DEVICE_READ_OK ? 1 : 0);
			nfp_read_complete(ndev, doorbell &
					  NFAST_INT_DEVICE_READ_OK ? 1 : 0);
		}

		if (doorbell &
		    ~(NFAST_INT_DEVICE_READ_OK | NFAST_INT_DEVICE_READ_FAILED |
		      NFAST_INT_DEVICE_WRITE_OK |
		      NFAST_INT_DEVICE_WRITE_FAILED)) {
			tmp16 = cpu_to_le16(doorbell);
			nfp_outw(ndev, CSR_BAR,
				 I21555_OFFSET_DOORBELL_PRI_CLEAR, tmp16);
			nfp_log(NFP_DBG1, "%s: unexpected interrupt %x",
				__func__, doorbell);
		}
		doorbell = nfp_inw(ndev, CSR_BAR,
				   I21555_OFFSET_DOORBELL_PRI_SET);
		doorbell = le16_to_cpu(doorbell);
	}
	nfp_log(NFP_DBG3, "%s: exiting", __func__);
	return 0;
}

/* write ------------------------------------------------------- */

static int i21555_write(u32 addr, const char *block, int len, void *ctx)
{
	struct nfp_dev *ndev;
	u32 hdr[2];
	int ne;
	u16 tmp16;
	u32 tmp32;

	nfp_log(NFP_DBG2, "%s: entered", __func__);

	ndev = (struct nfp_dev *)ctx;
	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: NULL ndev", __func__);
		return NFP_ENODEV;
	}

	ndev->stats.write_fail++;

	if (!ndev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}

	ne = i21555_started(ndev);
	if (ne) {
		if (ne != NFP_ESTARTING) {
			nfp_log(NFP_DBG1,
				"%s: i21555_started failed", __func__);
		}
		return ne;
	}

	nfp_log(NFP_DBG3, "%s: ndev->bar[ MEMBAR2 ]= %p", __func__,
		ndev->bar[MEMBAR2]);
	nfp_log(NFP_DBG3, "%s: ndev->bar[ CSR_BAR ]= %p", __func__,
		ndev->bar[CSR_BAR]);
	nfp_log(NFP_DBG3, "%s: block len %d", __func__, len);
	ne = nfp_copy_from_user_to_dev(ndev, MEMBAR2, NFPCI_JOBS_WR_DATA,
				       block, len);
	if (ne) {
		nfp_log(NFP_DBG1,
			"%s: nfp_copy_from_user_to_dev failed", __func__);
		return ne;
	}
	hdr[0] = cpu_to_le32(NFPCI_JOB_CONTROL);
	hdr[1] = cpu_to_le32(len);
	ne = nfp_copy_to_dev(ndev, MEMBAR2, NFPCI_JOBS_WR_CONTROL,
			     (const char *)hdr, 8);
	if (ne) {
		nfp_log(NFP_DBG1, "%s: nfp_copy_to_dev failed", __func__);
		return ne;
	}
	ne = nfp_copy_from_dev(ndev, MEMBAR2, NFPCI_JOBS_WR_LENGTH,
			       (char *)hdr, 4);
	if (ne) {
		nfp_log(NFP_DBG1, "%s: nfp_copy_from_dev failed", __func__);
		return ne;
	}

	tmp32 = cpu_to_le32(len);
	if (hdr[0] != tmp32) {
		nfp_log(NFP_DBG1, "%s: length not written", __func__);
		return NFP_EIO;
	}
	tmp16 = cpu_to_le16(NFAST_INT_HOST_WRITE_REQUEST >> 16);
	nfp_outw(ndev, CSR_BAR, I21555_OFFSET_DOORBELL_SEC_SET, tmp16);

	ndev->stats.write_fail--;
	ndev->stats.write_block++;
	ndev->stats.write_byte += len;

	nfp_log(NFP_DBG2, "%s: done", __func__);
	return NFP_SUCCESS;
}

/* read ------------------------------------------------------- */

static int i21555_read(char *block, int len, void *ctx, int *rcount)
{
	struct nfp_dev *ndev;
	int ne;
	int count;

	nfp_log(NFP_DBG2, "%s: entered", __func__);
	*rcount = 0;

	ndev = (struct nfp_dev *)ctx;
	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: NULL ndev", __func__);
		return NFP_ENODEV;
	}

	ndev->stats.read_fail++;

	if (!ndev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}

	ne = i21555_started(ndev);
	if (ne) {
		if (ne != NFP_ESTARTING)
			nfp_log(NFP_DBG1, "%s: i21555_started failed",
				__func__);

		return ne;
	}

	ne = nfp_copy_from_dev(ndev, MEMBAR2, NFPCI_JOBS_RD_LENGTH,
			       (char *)&count, 4);
	if (ne) {
		nfp_log(NFP_DBG1, "%s: nfp_copy_from_dev failed.", __func__);
		return ne;
	}
	count = le32_to_cpu(count);
	if (count < 0 || count > len) {
		nfp_log(NFP_DBG1,
			"%s: bad byte count (%d) from device", __func__,
			count);
		return NFP_EIO;
	}
	ne = nfp_copy_to_user_from_dev(ndev, MEMBAR2, NFPCI_JOBS_RD_DATA,
				       block, count);
	if (ne) {
		nfp_log(NFP_DBG1, "%s: nfp_copy_to_user_from_dev failed",
			__func__);
		return ne;
	}
	nfp_log(NFP_DBG2, "%s: done", __func__);
	*rcount = count;
	ndev->stats.read_fail--;
	ndev->stats.read_block++;
	ndev->stats.read_byte += len;
	return NFP_SUCCESS;
}

/* chupdate  ------------------------------------------------------- */

static int i21555_chupdate(char *data, int len, void *ctx)
{
	nfp_log(NFP_DBG1, "%s: NYI", __func__);
	return NFP_SUCCESS;
}

/* ensure reading -------------------------------------------------- */

static int i21555_ensure_reading(u32 addr, int len, void *ctx, int lock_flag)
{
	struct nfp_dev *ndev;
	u32 hdr[3];
	u16 tmp16;
	u32 tmp32;
	int ne;
	int hdr_len;

	nfp_log(NFP_DBG2, "%s: entered", __func__);

	ndev = (struct nfp_dev *)ctx;
	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: NULL ndev", __func__);
		return NFP_ENODEV;
	}

	ndev->stats.ensure_fail++;

	if (!ndev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__,
			CSR_BAR);
		return NFP_ENOMEM;
	}

	ne = i21555_started(ndev);
	if (ne) {
		if (ne != NFP_ESTARTING) {
			nfp_log(NFP_DBG1,
				"%s: i21555_started failed", __func__
				);
		}
		return ne;
	}

	nfp_log(NFP_DBG3, "%s: ndev->bar[ MEMBAR2 ]= %p", __func__,
		ndev->bar[MEMBAR2]);
	nfp_log(NFP_DBG3, "%s: ndev->bar[ CSR_BAR ]= %p", __func__,
		ndev->bar[CSR_BAR]);
	if (addr) {
		nfp_log(NFP_DBG3, "%s: new format, addr %p", __func__,
			addr);
		hdr[0] = cpu_to_le32(NFPCI_JOB_CONTROL_PCI_PUSH);
		hdr[1] = cpu_to_le32(len);
		hdr[2] = cpu_to_le32(addr);
		hdr_len = 12;
	} else {
		hdr[0] = cpu_to_le32(NFPCI_JOB_CONTROL);
		hdr[1] = cpu_to_le32(len);
		hdr_len = 8;
	}

	ne = nfp_copy_to_dev(ndev, MEMBAR2, NFPCI_JOBS_RD_CONTROL,
			     (const char *)hdr, hdr_len);
	if (ne) {
		nfp_log(NFP_DBG1,
			"%s: nfp_copy_to_dev failed", __func__);
		return ne;
	}

	ne = nfp_copy_from_dev(ndev, MEMBAR2, NFPCI_JOBS_RD_LENGTH,
			       (char *)hdr, 4);
	if (ne) {
		nfp_log(NFP_DBG1,
			"%s: nfp_copy_from_dev failed", __func__);
		return ne;
	}

	tmp32 = cpu_to_le32(len);

	if (hdr[0] != tmp32) {
		nfp_log(NFP_DBG1, "%s: len not written", __func__);
		return NFP_EIO;
	}
	tmp16 = cpu_to_le16(NFAST_INT_HOST_READ_REQUEST >> 16);
	nfp_outw(ndev, CSR_BAR, I21555_OFFSET_DOORBELL_SEC_SET, tmp16);

	ndev->stats.ensure_fail--;
	ndev->stats.ensure++;

	return NFP_SUCCESS;
}

/* set control register ----------------------------------------- */

static int i21555_set_control(const struct nfdev_control_str *control,
			      void *ctx)
{
	struct nfp_dev *ndev = (struct nfp_dev *)ctx;
	u32 control_flipped;

	nfp_log(NFP_DBG3, "%s: entered", __func__);
	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: NULL ndev", __func__);
		return NFP_ENODEV;
	}
	if (!ndev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}
	control_flipped = cpu_to_le32(control->control);
	nfp_outl(ndev, CSR_BAR, I21555_SCRATCHPAD_REGISTER_CONTROL,
		 control_flipped);
	return NFP_SUCCESS;
}

/* get status/error registers ----------------------------------- */

static int i21555_get_status(struct nfdev_status_str *status, void *ctx)
{
	struct nfp_dev *ndev = (struct nfp_dev *)ctx;
	u32 status_flipped;
	u32 *error = (u32 *)status->error;

	nfp_log(NFP_DBG3, "%s: entered", __func__);
	if (!ndev) {
		nfp_log(NFP_DBG1, "%s: NULL ndev", __func__);
		return NFP_ENODEV;
	}
	if (!ndev->bar[CSR_BAR]) {
		nfp_log(NFP_DBG1, "%s: null BAR[%d]", __func__, CSR_BAR);
		return NFP_ENOMEM;
	}
	status_flipped =
		nfp_inl(ndev, CSR_BAR, I21555_SCRATCHPAD_REGISTER_STATUS);
	status->status = le32_to_cpu(status_flipped);
	error[0] = nfp_inl(ndev, CSR_BAR, I21555_SCRATCHPAD_REGISTER_ERROR_LO);
	error[1] = nfp_inl(ndev, CSR_BAR, I21555_SCRATCHPAD_REGISTER_ERROR_HI);
	return NFP_SUCCESS;
}

/* command device structure ------------------------------------- */

const struct nfpcmd_dev i21555_cmddev = {
	.name = "nCipher nShield Solo",
	.vendorid = PCI_VENDOR_ID_INTEL,
	.deviceid = PCI_DEVICE_ID_INTEL_21555,
	.sub_vendorid = PCI_VENDOR_ID_NCIPHER,
	.sub_deviceid = PCI_SUBSYSTEM_ID_NFAST_REV1,
	.bar_sizes = BAR_SIZES,
	.flags = 0,
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
