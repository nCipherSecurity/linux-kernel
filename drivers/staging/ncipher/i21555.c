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
	dev_info(&ndev->pcidev->dev,
		 "%s: addr %p", __func__, ndev->bar[bar] + offset);
	return ioread32(ndev->bar[bar] + offset);
}

u16 nfp_inw(struct nfp_dev *ndev, int bar, int offset)
{
	dev_info(&ndev->pcidev->dev,
		 "%s: addr %p", __func__, ndev->bar[bar] + offset);
	return ioread16(ndev->bar[bar] + offset);
}

void nfp_outl(struct nfp_dev *ndev, int bar, int offset, u32 data)
{
	dev_info(&ndev->pcidev->dev,
		 "%s: addr %p, data %x", __func__,
		 ndev->bar[bar] + offset, data);
	iowrite32(data, ndev->bar[bar] + offset);
}

void nfp_outw(struct nfp_dev *ndev, int bar, int offset, u16 data)
{
	dev_info(&ndev->pcidev->dev,
		 "%s: addr %p, data %x", __func__,
		 ndev->bar[bar] + offset, data);
	iowrite16(data, ndev->bar[bar] + offset);
}

int nfp_config_inl(struct nfp_dev *ndev, int offset, u32 *res)
{
	if (!ndev || !ndev->pcidev)
		return -ENODEV;
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
	int ne;

	/* check for device */
	if (!ndev) {
		pr_err("%s: error: no device", __func__);
		return -ENODEV;
	}

	dev_info(&ndev->pcidev->dev, "%s: entered", __func__);

	ne = nfp_config_inl(ndev, I21555_CFG_SEC_CMD_STATUS, &tmp32);
	if (ne != 0) {
		dev_err(&ndev->pcidev->dev,
			"%s: nfp_config_inl failed",
			__func__);
		return ne;
	}

	tmp32 = le32_to_cpu(tmp32) & 0xffff;

	if (tmp32 & CFG_CMD_MASTER) {
		dev_notice(&ndev->pcidev->dev,
			   "%s: Yes %x", __func__, tmp32);
	} else {
		dev_err(&ndev->pcidev->dev,
			"%s: device not started yet %x", __func__,
			tmp32);
		ne = -EAGAIN;
	}
	return ne;
}

/* create ------------------------------------------------------- */

static int i21555_create(struct nfp_dev *ndev)
{
	u32 tmp32;

	/* check for device */
	if (!ndev) {
		pr_err("%s: error: no device", __func__);
		return -ENODEV;
	}

	dev_info(&ndev->pcidev->dev, "%s: entered", __func__);

	/* set our context to just be a pointer to our struct nfp_dev */
	ndev->cmdctx = ndev;

	if (!ndev->bar[CSR_BAR]) {
		dev_err(&ndev->pcidev->dev,
			"%s: null BAR[%d]", __func__, CSR_BAR);
		return -ENOMEM;
	}
	dev_warn(&ndev->pcidev->dev, "%s: enable doorbell", __func__);
	tmp32 = cpu_to_le32(I21555_DOORBELL_PRI_ENABLE);
	nfp_outl(ndev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_SET_MASK, tmp32);
	nfp_outl(ndev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_CLEAR_MASK, tmp32);
	return 0;
}

/* stop ------------------------------------------------------- */

static int i21555_destroy(struct nfp_dev *ctx)
{
	struct nfp_dev *ndev = ctx;
	u32 tmp32;

	/* check for device */
	if (!ndev) {
		pr_err("%s: error: no device", __func__);
		return -ENODEV;
	}

	dev_info(&ndev->pcidev->dev, "%s: entered", __func__);

	if (!ndev) {
		dev_err(&ndev->pcidev->dev, "%s: NULL ndev", __func__);
		return -ENODEV;
	}
	if (!ndev->bar[CSR_BAR]) {
		dev_err(&ndev->pcidev->dev,
			"%s: null BAR[%d]", __func__, CSR_BAR);
		return -ENOMEM;
	}
	tmp32 = cpu_to_le32(I21555_DOORBELL_PRI_DISABLE);
	nfp_outl(ndev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_SET_MASK, tmp32);
	nfp_outl(ndev, CSR_BAR, I21555_OFFSET_DOORBELL_PRI_CLEAR_MASK, tmp32);

	return 0;
}

/* open ------------------------------------------------------- */

static int i21555_open(struct nfp_dev *ctx)
{
	struct nfp_dev *ndev = ctx;

	/* check for device */
	if (!ndev) {
		pr_err("%s: error: no device", __func__);
		return -ENODEV;
	}

	dev_info(&ndev->pcidev->dev, "%s: entered", __func__);

	return 0;
}

/* close ------------------------------------------------------- */

static int i21555_close(struct nfp_dev *ctx)
{
	struct nfp_dev *ndev = ctx;

	/* check for device */
	if (!ndev) {
		pr_err("%s: error: no device", __func__);
		return -ENODEV;
	}

	dev_info(&ndev->pcidev->dev, "%s: entered", __func__);

	return 0;
}

/* isr ------------------------------------------------------- */

static int i21555_isr(struct nfp_dev *ctx, int *handled)
{
	struct nfp_dev *ndev = ctx;
	u16 doorbell;
	u16 tmp16;

	/* check for device */
	if (!ndev) {
		pr_err("%s: error: no device", __func__);
		return -ENODEV;
	}

	dev_info(&ndev->pcidev->dev, "%s: entered", __func__);

	*handled = 0;

	ndev->stats.isr++;

	if (!ndev->bar[CSR_BAR]) {
		dev_err(&ndev->pcidev->dev,
			"%s: null BAR[%d]", __func__, CSR_BAR);
		return -ENOMEM;
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

			dev_warn(&ndev->pcidev->dev,
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

			dev_warn(&ndev->pcidev->dev,
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
			dev_err(&ndev->pcidev->dev, "%s: unexpected interrupt %x",
				__func__, doorbell);
		}
		doorbell = nfp_inw(ndev, CSR_BAR,
				   I21555_OFFSET_DOORBELL_PRI_SET);
		doorbell = le16_to_cpu(doorbell);
	}
	dev_notice(&ndev->pcidev->dev, "%s: exiting", __func__);
	return 0;
}

/* write ------------------------------------------------------- */

static int i21555_write(u32 addr, const char *block, int len,
			struct nfp_dev *ctx)
{
	struct nfp_dev *ndev = ctx;
	u32 hdr[2];
	int ne;
	u16 tmp16;
	u32 tmp32;

	/* check for device */
	if (!ndev) {
		pr_err("%s: error: no device", __func__);
		return -ENODEV;
	}

	dev_info(&ndev->pcidev->dev, "%s: entered", __func__);

	ndev->stats.write_fail++;

	if (!ndev->bar[CSR_BAR]) {
		dev_err(&ndev->pcidev->dev,
			"%s: null BAR[%d]", __func__, CSR_BAR);
		return -ENOMEM;
	}

	ne = i21555_started(ndev);
	if (ne != 0) {
		if (ne != -EAGAIN) {
			dev_err(&ndev->pcidev->dev,
				"%s: i21555_started failed", __func__);
		}
		return ne;
	}

	dev_notice(&ndev->pcidev->dev, "%s: ndev->bar[ MEMBAR2 ]= %p", __func__,
		   ndev->bar[MEMBAR2]);
	dev_notice(&ndev->pcidev->dev, "%s: ndev->bar[ CSR_BAR ]= %p", __func__,
		   ndev->bar[CSR_BAR]);
	dev_notice(&ndev->pcidev->dev, "%s: block len %d", __func__, len);
	ne = nfp_copy_from_user_to_dev(ndev, MEMBAR2, NFPCI_JOBS_WR_DATA,
				       block, len);
	if (ne != 0) {
		dev_err(&ndev->pcidev->dev,
			"%s: nfp_copy_from_user_to_dev failed", __func__);
		return ne;
	}
	hdr[0] = cpu_to_le32(NFPCI_JOB_CONTROL);
	hdr[1] = cpu_to_le32(len);
	ne = nfp_copy_to_dev(ndev, MEMBAR2, NFPCI_JOBS_WR_CONTROL,
			     (const char *)hdr, 8);
	if (ne != 0) {
		dev_err(&ndev->pcidev->dev,
			"%s: nfp_copy_to_dev failed", __func__);
		return ne;
	}
	ne = nfp_copy_from_dev(ndev, MEMBAR2, NFPCI_JOBS_WR_LENGTH,
			       (char *)hdr, 4);
	if (ne != 0) {
		dev_err(&ndev->pcidev->dev,
			"%s: nfp_copy_from_dev failed", __func__);
		return ne;
	}

	tmp32 = cpu_to_le32(len);
	if (hdr[0] != tmp32) {
		dev_err(&ndev->pcidev->dev,
			"%s: length not written", __func__);
		return -EIO;
	}
	tmp16 = cpu_to_le16(NFAST_INT_HOST_WRITE_REQUEST >> 16);
	nfp_outw(ndev, CSR_BAR, I21555_OFFSET_DOORBELL_SEC_SET, tmp16);

	ndev->stats.write_fail--;
	ndev->stats.write_block++;
	ndev->stats.write_byte += len;

	dev_warn(&ndev->pcidev->dev, "%s: done", __func__);
	return 0;
}

/* read ------------------------------------------------------- */

static int i21555_read(char *block, int len, struct nfp_dev *ctx, int *rcount)
{
	struct nfp_dev *ndev = ctx;
	int ne;
	int count;

	/* check for device */
	if (!ndev) {
		pr_err("%s: error: no device", __func__);
		return -ENODEV;
	}

	dev_info(&ndev->pcidev->dev, "%s: entered", __func__);
	*rcount = 0;

	ndev->stats.read_fail++;

	if (!ndev->bar[CSR_BAR]) {
		dev_err(&ndev->pcidev->dev,
			"%s: null BAR[%d]", __func__, CSR_BAR);
		return -ENOMEM;
	}

	ne = i21555_started(ndev);
	if (ne != 0) {
		if (ne != -EAGAIN)
			dev_err(&ndev->pcidev->dev,
				"%s: i21555_started failed",
				__func__);

		return ne;
	}

	ne = nfp_copy_from_dev(ndev, MEMBAR2, NFPCI_JOBS_RD_LENGTH,
			       (char *)&count, 4);
	if (ne != 0) {
		dev_err(&ndev->pcidev->dev,
			"%s: nfp_copy_from_dev failed.", __func__);
		return ne;
	}
	count = le32_to_cpu(count);
	if (count < 0 || count > len) {
		dev_err(&ndev->pcidev->dev,
			"%s: bad byte count (%d) from device", __func__,
			count);
		return -EIO;
	}
	ne = nfp_copy_to_user_from_dev(ndev, MEMBAR2, NFPCI_JOBS_RD_DATA,
				       block, count);
	if (ne != 0) {
		dev_err(&ndev->pcidev->dev,
			"%s: nfp_copy_to_user_from_dev failed",
			__func__);
		return ne;
	}
	dev_warn(&ndev->pcidev->dev, "%s: done", __func__);
	*rcount = count;
	ndev->stats.read_fail--;
	ndev->stats.read_block++;
	ndev->stats.read_byte += len;
	return 0;
}

/* ensure reading -------------------------------------------------- */

static int i21555_ensure_reading(dma_addr_t addr,
				 int len, struct nfp_dev *ctx, int lock_flag)
{
	struct nfp_dev *ndev = ctx;
	u32 hdr[3];
	u16 tmp16;
	u32 tmp32;
	int ne;
	int hdr_len;

	/* check for device */
	if (!ndev) {
		pr_err("%s: error: no device", __func__);
		return -ENODEV;
	}

	dev_info(&ndev->pcidev->dev, "%s: entered", __func__);

	ndev->stats.ensure_fail++;

	if (!ndev->bar[CSR_BAR]) {
		dev_err(&ndev->pcidev->dev, "%s: null BAR[%d]", __func__,
			CSR_BAR);
		return -ENOMEM;
	}

	ne = i21555_started(ndev);
	if (ne != 0) {
		if (ne != -EAGAIN) {
			dev_err(&ndev->pcidev->dev,
				"%s: i21555_started failed", __func__
				);
		}
		return ne;
	}

	dev_notice(&ndev->pcidev->dev, "%s: ndev->bar[ MEMBAR2 ]= %p", __func__,
		   ndev->bar[MEMBAR2]);
	dev_notice(&ndev->pcidev->dev, "%s: ndev->bar[ CSR_BAR ]= %p", __func__,
		   ndev->bar[CSR_BAR]);
	if (addr) {
		dev_notice(&ndev->pcidev->dev,
			   "%s: new format, addr %p",
			   __func__, (void *)addr);
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
	if (ne != 0) {
		dev_err(&ndev->pcidev->dev,
			"%s: nfp_copy_to_dev failed", __func__);
		return ne;
	}

	ne = nfp_copy_from_dev(ndev, MEMBAR2, NFPCI_JOBS_RD_LENGTH,
			       (char *)hdr, 4);
	if (ne != 0) {
		dev_err(&ndev->pcidev->dev,
			"%s: nfp_copy_from_dev failed", __func__);
		return ne;
	}

	tmp32 = cpu_to_le32(len);

	if (hdr[0] != tmp32) {
		dev_err(&ndev->pcidev->dev, "%s: len not written", __func__);
		return -EIO;
	}
	tmp16 = cpu_to_le16(NFAST_INT_HOST_READ_REQUEST >> 16);
	nfp_outw(ndev, CSR_BAR, I21555_OFFSET_DOORBELL_SEC_SET, tmp16);

	ndev->stats.ensure_fail--;
	ndev->stats.ensure++;

	return 0;
}

/* set control register ----------------------------------------- */

static int i21555_set_control(const struct nfdev_control_str *control,
			      struct nfp_dev *ctx)
{
	struct nfp_dev *ndev = ctx;
	u32 control_flipped;

	/* check for device */
	if (!ndev) {
		pr_err("%s: error: no device", __func__);
		return -ENODEV;
	}

	dev_info(&ndev->pcidev->dev, "%s: entered", __func__);
	if (!ndev->bar[CSR_BAR]) {
		dev_err(&ndev->pcidev->dev,
			"%s: null BAR[%d]", __func__, CSR_BAR);
		return -ENOMEM;
	}
	control_flipped = cpu_to_le32(control->control);
	nfp_outl(ndev, CSR_BAR, I21555_SCRATCHPAD_REGISTER_CONTROL,
		 control_flipped);
	return 0;
}

/* get status/error registers ----------------------------------- */

static int i21555_get_status(struct nfdev_status_str *status,
			     struct nfp_dev *ctx)
{
	struct nfp_dev *ndev = ctx;
	u32 status_flipped;
	u32 *error = (u32 *)status->error;

	/* check for device */
	if (!ndev) {
		pr_err("%s: error: no device", __func__);
		return -ENODEV;
	}

	dev_info(&ndev->pcidev->dev, "%s: entered", __func__);
	if (!ndev->bar[CSR_BAR]) {
		dev_err(&ndev->pcidev->dev,
			"%s: null BAR[%d]", __func__, CSR_BAR);
		return -ENOMEM;
	}
	status_flipped =
		nfp_inl(ndev, CSR_BAR, I21555_SCRATCHPAD_REGISTER_STATUS);
	status->status = le32_to_cpu(status_flipped);
	error[0] = nfp_inl(ndev, CSR_BAR, I21555_SCRATCHPAD_REGISTER_ERROR_LO);
	error[1] = nfp_inl(ndev, CSR_BAR, I21555_SCRATCHPAD_REGISTER_ERROR_HI);
	return 0;
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
	.ensure_reading = i21555_ensure_reading,
	.setcontrol = i21555_set_control,
	.getstatus = i21555_get_status,
};
