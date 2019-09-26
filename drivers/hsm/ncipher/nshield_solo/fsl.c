// SPDX-License-Identifier: GPL-2.0+
/*
 *
 * fsl.c: nCipher PCI HSM FSL command driver
 *
 */

#include "osif.h"
#include "fsl.h"

/* If defined, use aggressive checking for errors. */
#define FSL_AGRESSIVE_CHECKING 0

/**
 * Resets FSL device.
 *
 * Extra device info is initialized the first time created.
 *
 * @param cdev common device.
 * @returns NFP_SUCCESS if successful, other value if error.
 */
static int fsl_create(struct nfp_cdev *cdev)
{
	u32 clr = 0;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	if (!cdev) {
		nfp_log(NFP_DBG1, "%s: error: no device", __func__);
		return NFP_ENODEV;
	}

	if (cdev->created) {
		nfp_log(NFP_DBG3, "%s: device already created", __func__);
		return NFP_SUCCESS;
	}

	cdev->active_bar = -1;
	cdev->detection_type = NFP_EPOLLING;
	cdev->conn_status = NFP_ESTARTING;

	/* try to reset check doorbell registers
	 * (don't read back in case they hang)
	 */
	TO_LE32_MEM(&clr, NFAST_INT_DEVICE_CLR);

	cdev->active_bar = FSL_MEMBAR;
	fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_CS_STATUS, clr);

	if (!cdev->bar[cdev->active_bar]) {
		nfp_log(NFP_DBG1, "%s: error: null FSL memory BAR[%d]",
			__func__, cdev->active_bar);
		return NFP_ENOMEM;
	}

	/* set our context to just be a pointer to ourself */
	cdev->cmdctx = cdev;

	/* try to reset read/write doorbell registers
	 * (don't read back in case they hang)
	 */
	nfp_log(NFP_DBG3, "%s: clearing read/write doorbell registers",
		__func__);
	TO_LE32_MEM(&clr, NFAST_INT_HOST_CLR);
	fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_CMD, clr);
	fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_CMD, clr);

	nfp_log(NFP_DBG3, "%s: exiting %s active_bar: %d.",
		__func__, __func__, cdev->active_bar);

	cdev->created = 1;

	return NFP_SUCCESS;
}

/**
 * Destroys an FSL device.
 *
 * @param ctx device context (always the device itself).
 * @returns NFP_SUCCESS if successful, other value if error.
 */
static int fsl_destroy(void *ctx)
{
	struct nfp_cdev *cdev;
	unsigned int tmp32;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* check for device */
	cdev = (struct nfp_cdev *)ctx;
	if (!cdev) {
		nfp_log(NFP_DBG1, "%s: warning: no device", __func__);
		return NFP_ENODEV;
	}

	/* clear doorbell registers */
	if (cdev->bar[cdev->active_bar]) {
		nfp_log(NFP_DBG3, "%s: clearing doorbell registers", __func__);
		TO_LE32_MEM(&tmp32, NFAST_INT_DEVICE_CLR);
		fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_STATUS,
			 tmp32);
		fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_STATUS,
			 tmp32);
		fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_CS_STATUS,
			 tmp32);
	} else {
		nfp_log(NFP_DBG1, "%s: warning: no FSL BAR[%d] memory",
			__func__, cdev->active_bar);
	}

	return NFP_SUCCESS;
}

/**
 * Returns fsl_created status.
 *
 * @param cdev common device.
 * @returns NFP_SUCCESS if created or other value if error.
 */
static int fsl_created(struct nfp_cdev *cdev)
{
	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* check for device */

	if (!cdev) {
		nfp_log(NFP_DBG1, "%s: error: no device", __func__);
		return NFP_ENODEV;
	}

	if (!cdev->created) {
		nfp_log(NFP_DBG1, "%s: error: device not created", __func__);
		return NFP_ENODEV;
	}

	if (!cdev->bar[cdev->active_bar]) {
		nfp_log(NFP_DBG1, "%s: error: no FSL BAR[%d] memory", __func__,
			cdev->active_bar);
		return NFP_ENOMEM;
	}

	return NFP_SUCCESS;
}

/* ----------------------------------------------------- */
/* This call needs to be in synch with the ISR or the ISR will come in the
 *  middle of a write/read op and cause problems.
 */
static int fsl_connection_status(struct nfp_cdev *cdev, int lock_flag,
				 int epd_status)
{
	int status = NFP_ESTARTING;
	(void)lock_flag;

	if (!cdev)
		return NFP_ENODEV;

	/* this code is mainly to support backwards compatibility with the
	 * interrupt driven approach to detection.
	 */
	if (epd_status == NFP_EPOLLING) {
		cdev->detection_type = NFP_EPOLLING;
		cdev->conn_status = NFP_SUCCESS;
	} else if (epd_status == NFAST_INT_DEVICE_PCI_DOWN) {
		cdev->conn_status = NFP_ESTARTING;
	}

	status = cdev->conn_status;
	return status;
}

/**
 * Returns connection check status.
 *
 * @param cdev common device.
 * @returns NFP_SUCCESS if started, NFP_ESTARTING if not ready,
 * or other value if error.
 */
static int fsl_started(struct nfp_cdev *cdev, int lock_flag)
{
	int status = NFP_ESTARTING;
	int epd_status = NFP_ESTARTING;
	u32 doorbell_cs = 0x0;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* check for device */

	if (!cdev) {
		nfp_log(NFP_DBG1, "%s: error: no device", __func__);
		return NFP_ENODEV;
	}

	if (!cdev->bar[cdev->active_bar]) {
		nfp_log(NFP_DBG1, "%s: error: no FSL BAR[%d] memory", __func__,
			cdev->active_bar);
		return NFP_ENOMEM;
	}

	/* check the status register to see if epd has started */
	doorbell_cs =
		fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_POLLING);
	doorbell_cs = FROM_LE32_MEM(&doorbell_cs);
	nfp_log(NFP_DBG3, "%s: doorbell_polling is: %x", __func__, doorbell_cs);

	if (doorbell_cs == NFAST_INT_DEVICE_POLL) {
		epd_status = NFP_EPOLLING;
		nfp_log(NFP_DBG3, "%s: EPD in polling mode", __func__);
	} else if (doorbell_cs == NFAST_INT_DEVICE_PCI_DOWN) {
		epd_status = NFAST_INT_DEVICE_PCI_DOWN;
	}
	/* check current connection status */
	status = fsl_connection_status(cdev, lock_flag, epd_status);

	if (status == NFP_SUCCESS) {
		nfp_log(NFP_DBG3, "%s: device started", __func__);
	} else if (status == NFP_ESTARTING) {
		nfp_log(NFP_DBG3, "%s: device starting", __func__);
	} else {
		nfp_log(NFP_DBG1, "%s: error: device failure: code 0x%x",
			__func__, status);
	}

	return status;
}

/**
 * Resets connection check status.
 *
 * This can be called by the device driver when an I/O operation times out.
 * This causes any following I/O operations to fail quickly until fsl_started()
 * has been set successfully by a periodic check interrupt.
 *
 * @param cdev common device.
 * @returns NFP_SUCCESS if stopped or other value if error.
 */
static int fsl_stopped(struct nfp_cdev *cdev)
{
	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* check for device */

	if (!cdev) {
		nfp_log(NFP_DBG1, "%s: error: no device", __func__);
		return NFP_ENODEV;
	}

	if (!cdev->bar[cdev->active_bar]) {
		nfp_log(NFP_DBG1, "%s: error: no FSL BAR[%d] memory", __func__,
			cdev->active_bar);
		return NFP_ENOMEM;
	}

	/* reset current connection status */

	cdev->conn_status = NFP_ESTARTING;

	nfp_log(NFP_DBG2, "%s: device stopped", __func__);

	return NFP_SUCCESS;
}

static int fsl_update_connection_status(struct nfp_cdev *cdev, int status)
{
	int current_status;

	if (!cdev)
		return NFP_ENODEV;

	current_status = cdev->conn_status;
	cdev->conn_status = status;
	return current_status;
}

/**
 * Completes a connection check status interrupt.
 *
 * @param cdev common device.
 * @param status device status.
 */
static void fsl_check_complete(struct nfp_cdev *cdev, int status)
{
	int ne;
	u32 clr, chk;
	int started;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* check for device */

	ne = fsl_created(cdev);
	if (ne != NFP_SUCCESS) {
		nfp_log(NFP_DBG1, "%s: error: check not completed", __func__);
		return;
	}

	/* started becomes true after fsl_create and the first cs interrupt is
	 * successful. It switches to false right afer since cs_status is set to
	 * NFP_SUCCESS right after this check. A fsl_close or a fsl_create can
	 * reset the cs_status to NFP_ESTARTING again.
	 */
	started =
		(fsl_update_connection_status(cdev, status) == NFP_ESTARTING) &&
		(status == NFP_SUCCESS);

	/* reset read/write doorbell registers if just started */

	if (started) {
		nfp_log(NFP_DBG3,
			"fsl_create: clearing read/write doorbell registers");
		TO_LE32_MEM(&clr, NFAST_INT_HOST_CLR);
		fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_CMD,
			 clr);
		chk = fsl_inl(cdev, cdev->active_bar,
			      FSL_OFFSET_DOORBELL_WR_CMD);
		fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_CMD,
			 clr);
		chk = fsl_inl(cdev, cdev->active_bar,
			      FSL_OFFSET_DOORBELL_RD_CMD);
		TO_LE32_MEM(&clr, NFAST_INT_DEVICE_CLR);
		fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_STATUS,
			 clr);
		chk = fsl_inl(cdev, cdev->active_bar,
			      FSL_OFFSET_DOORBELL_WR_STATUS);
		fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_STATUS,
			 clr);
		chk = fsl_inl(cdev, cdev->active_bar,
			      FSL_OFFSET_DOORBELL_RD_STATUS);
	}

	if (status == NFP_SUCCESS) {
		nfp_log(NFP_DBG3, "%s: device started", __func__);
	} else if (status == NFP_ESTARTING) {
		nfp_log(NFP_DBG3, "%s: device not started yet", __func__);
	} else {
		nfp_log(NFP_DBG1, "%s: device check failed with code: 0x%x",
			__func__, status);
	}
}

/**
 * Handles an interrupt from the FSL device.
 *
 * @param ctx device context (always the device itself).
 * @param handled set non-zero by this routine if interrupt considered handled
 * @returns NFP_SUCCESS if successful, other value if error.
 */
static int fsl_isr(void *ctx, int *handled)
{
	struct nfp_cdev *cdev;
	int ne;
	u32 doorbell_rd, doorbell_wr, doorbell_cs;
#ifdef FSL_AGRESSIVE_CHECKING
	int x_wr = 0, x_rd = 0, x_cs = 0;
#endif

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* mark not yet handled */

	*handled = 0;

	/* check for device */

	cdev = (struct nfp_cdev *)ctx;
	ne = fsl_created(cdev);
	if (ne != NFP_SUCCESS) {
		nfp_log(NFP_DBG1, "%s: error: interrupt not handled", __func__);
		return ne;
	}

	++cdev->stats.isr;

	doorbell_wr =
		fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_STATUS);
	doorbell_wr = FROM_LE32_MEM(&doorbell_wr);
	doorbell_rd =
		fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_STATUS);
	doorbell_rd = FROM_LE32_MEM(&doorbell_rd);
	doorbell_cs =
		fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_CS_STATUS);
	doorbell_cs = FROM_LE32_MEM(&doorbell_cs);
	nfp_log(NFP_DBG3, "%s: cs:= %x,rd:=%x,wr:=%x",
		__func__, doorbell_cs, doorbell_rd, doorbell_wr);

	while (doorbell_rd || doorbell_wr || doorbell_cs) {
		/* prevent any illegal combination of set bits from triggering
		 * processing. Note that if anyone of these registers have an
		 * incorrect bit set, it would prevent the other operations from
		 * being processed since we return from the ISR, even if they
		 * have legal values. This is an unlikely scenario since these
		 * registers are written either to 0 or one of the legal values
		 * by the software on the card.
		 */
		if ((doorbell_cs) &&
		    ((doorbell_cs & ~NFAST_INT_DEVICE_CHECK_OK) &&
		     (doorbell_cs & ~NFAST_INT_DEVICE_CHECK_FAILED))) {
			nfp_log(NFP_DBG1, "%s: illegal bits in doorbell_cs %x",
				__func__, doorbell_cs);
			*handled = 1;
			/* clear the register*/
			fsl_outl(cdev, cdev->active_bar,
				 FSL_OFFSET_DOORBELL_CS_STATUS,
				 NFAST_INT_DEVICE_CLR);
			return 0;
		}
		if ((doorbell_rd) &&
		    ((doorbell_rd & ~NFAST_INT_DEVICE_READ_OK) &&
		     (doorbell_rd & ~NFAST_INT_DEVICE_READ_FAILED))) {
			nfp_log(NFP_DBG1, "%s: illegal bits in doorbell_rd %x",
				__func__, doorbell_rd);
			*handled = 1;
			/* clear the register*/
			fsl_outl(cdev, cdev->active_bar,
				 FSL_OFFSET_DOORBELL_RD_STATUS,
				 NFAST_INT_DEVICE_CLR);
			return 0;
		}
		if ((doorbell_wr) &&
		    ((doorbell_wr & ~NFAST_INT_DEVICE_WRITE_OK) &&
		     (doorbell_wr & ~NFAST_INT_DEVICE_WRITE_FAILED))) {
			nfp_log(NFP_DBG1, "%s: illegal bits in doorbell_wr %x",
				__func__, doorbell_wr);
			/* clear the register*/
			fsl_outl(cdev, cdev->active_bar,
				 FSL_OFFSET_DOORBELL_WR_STATUS,
				 NFAST_INT_DEVICE_CLR);
			*handled = 1;
			return 0;
		}

		/* service interrupts.
		 * if we made it here, the doorbell registers are all valid,
		 * so no need to check for their validity anymore.
		 */

		if (doorbell_wr) {
			fsl_outl(cdev, cdev->active_bar,
				 FSL_OFFSET_DOORBELL_WR_STATUS,
				 NFAST_INT_DEVICE_CLR);
			cdev->stats.isr_write++;
			nfp_write_complete(cdev->dev,
					   doorbell_wr &
					     NFAST_INT_DEVICE_WRITE_OK ?
						1 : 0);
			nfp_log(NFP_DBG3,
				"%s: acknowledging write interrupt: ok = %d",
				__func__,
				doorbell_wr & NFAST_INT_DEVICE_WRITE_OK ? 1 :
				0);

#ifdef FSL_AGRESSIVE_CHECKING
			if (fsl_inl(cdev, cdev->active_bar,
				    FSL_OFFSET_DOORBELL_WR_STATUS) !=
			    NFAST_INT_DEVICE_CLR) {
				nfp_log(NFP_DBG1,
					"%s: failed to clear doorbell write status",
					__func__);
			}
			++x_wr;
#endif
		}

		if (doorbell_rd) {
			fsl_outl(cdev, cdev->active_bar,
				 FSL_OFFSET_DOORBELL_RD_STATUS,
				 NFAST_INT_DEVICE_CLR);
			cdev->stats.isr_read++;
			nfp_read_complete(cdev->dev,
					  doorbell_rd &
					    NFAST_INT_DEVICE_READ_OK ? 1 : 0);
			nfp_log(NFP_DBG3,
				"%s: acknowledging read interrupt: ok = %d",
				__func__,
				doorbell_rd & NFAST_INT_DEVICE_READ_OK ? 1 : 0);

#ifdef FSL_AGRESSIVE_CHECKING
			if (fsl_inl(cdev, cdev->active_bar,
				    FSL_OFFSET_DOORBELL_RD_STATUS) !=
			    NFAST_INT_DEVICE_CLR) {
				nfp_log(NFP_DBG1,
					"%s: failed to clear doorbell read status",
					__func__);
			}
			++x_rd;
#endif
		}
		/* the doorbell_cs is being phased out in favor of polling since
		 * there were issues caused by this interrupt being issued from
		 * the card on its own when the driver was not even present.
		 * To maintain backwards compatibility, this code is being kept,
		 * but might be removed in the future.
		 */
		nfp_log(NFP_DBG3, "%s: doorbell_cs is: %x",
			__func__, doorbell_cs);
		if (doorbell_cs) {
			fsl_outl(cdev, cdev->active_bar,
				 FSL_OFFSET_DOORBELL_CS_STATUS,
				 NFAST_INT_DEVICE_CLR);
			fsl_check_complete(cdev,
					   doorbell_cs &
						NFAST_INT_DEVICE_CHECK_OK ?
						NFP_SUCCESS :
						NFP_ESTARTING);
			nfp_log(NFP_DBG3,
				"%s: acknowledging check interrupt: status:0x%x",
				__func__,
				doorbell_cs & NFAST_INT_DEVICE_CHECK_OK ?
					NFP_SUCCESS :
					NFP_ESTARTING);

#ifdef FSL_AGRESSIVE_CHECKING
			if (fsl_inl(cdev, cdev->active_bar,
				    FSL_OFFSET_DOORBELL_CS_STATUS) !=
			    NFAST_INT_DEVICE_CLR) {
				nfp_log(NFP_DBG1,
					"%s: warning: failed to clear doorbell check status",
					__func__);
			}
			++x_cs;
#endif
		}

		doorbell_wr = fsl_inl(cdev, cdev->active_bar,
				      FSL_OFFSET_DOORBELL_WR_STATUS);
		doorbell_wr = FROM_LE32_MEM(&doorbell_wr);
		doorbell_rd = fsl_inl(cdev, cdev->active_bar,
				      FSL_OFFSET_DOORBELL_RD_STATUS);
		doorbell_rd = FROM_LE32_MEM(&doorbell_rd);
		doorbell_cs = fsl_inl(cdev, cdev->active_bar,
				      FSL_OFFSET_DOORBELL_CS_STATUS);
		doorbell_cs = FROM_LE32_MEM(&doorbell_cs);

		nfp_log(NFP_DBG3, "%s: cs status in isr is: %x",
			__func__, doorbell_cs);
	}

	/* always report the interrupt as handled */

	*handled = 1;
#ifdef FSL_AGRESSIVE_CHECKING
	if (x_wr + x_rd + x_cs == 0) {
		nfp_log(NFP_DBG2,
			"%s: no operations handled by this ISR call",
			__func__);
	} else {
		if (x_wr + x_rd + x_cs > 1) {
			nfp_log(NFP_DBG3,
				"%s: DEBUG: multiple operations handled by this ISR call: wr=%d, rd=%d, cs=%d",
				__func__, x_wr, x_rd, x_cs);
		} else {
			nfp_log(NFP_DBG3,
				"%s: DEBUG: one operation handled by this ISR call: wr=%d, rd=%d, cs=%d",
				__func__, x_wr, x_rd, x_cs);
		}
	}
#endif
	nfp_log(NFP_DBG3, "%s: exiting", __func__);

	return 0;
}

/**
 * Performs additional FSL-specific actions when opening a device.
 *
 * This routine returns an error if the device has not properly started.
 *
 * @param ctx device context (always the device itself).
 * @returns NFP_SUCCESS if successful, other value if error.
 */
static int fsl_open(void *ctx)
{
	struct nfp_cdev *cdev;
	int ne;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* check for device */

	cdev = (struct nfp_cdev *)ctx;
	ne = fsl_started(cdev, NFP_WITH_LOCK);
	if (ne != NFP_SUCCESS) {
		cdev->stats.ensure_fail++;
		nfp_log(NFP_DBG1, "%s: error: device not started", __func__);
		return ne;
	}

	return NFP_SUCCESS;
}

/**
 * Performs additional FSL-specific actions when closing a device.
 *
 * @param ctx device context (always the device itself).
 * @returns NFP_SUCCESS if successful, other value if error.
 */
static int fsl_close(void *ctx)
{
	(void)ctx;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	return NFP_SUCCESS;
}

/**
 * Updates the device channel (not implemented).
 *
 * @param data data buffer.
 * @param len length.
 * @param ctx device context (always the device itself).
 * @returns NFP_SUCCESS.
 */
static int fsl_chupdate(char *data, int len, void *ctx)
{
	(void)data;
	(void)len;
	(void)ctx;

	nfp_log(NFP_DBG1, "%s: warning: not implemented", __func__);

	return NFP_SUCCESS;
}

/**
 * Sets control data.
 *
 * The device control register is writen directly. No doorbell style handshake
 * is used.
 *
 * @param control control string to copy from.
 * @param ctx device context (always the device itself).
 * @returns NFP_SUCCESS if successful, other value if error.
 */
static int fsl_set_control(const struct nfdev_control_str *control, void *ctx)
{
	u32 control_data;
	struct nfp_cdev *cdev;
	int ne;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* check for device */

	cdev = (struct nfp_cdev *)ctx;
	ne = fsl_started(cdev, NFP_WITH_LOCK);
	if (ne != NFP_SUCCESS) {
		nfp_log(NFP_DBG1, "%s: error: unable to set control", __func__);
		return ne;
	}

	/* set control (written immediately with no explicit
	 * synchronization with the firmware)
	 */

	TO_LE32_MEM(&control_data, control->control);
	fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_REGISTER_CONTROL,
		 control_data);

	return NFP_SUCCESS;
}

/**
 * Returns status data.
 *
 * The device status registers are read immediately. No doorbell style
 * handshake is used. Without explicit synchronization, it is possible
 * that an inconsistent state may be returned if the status is being
 * updated by the firmware while simultaneously being read by the host.
 * For example, the call could return an updated status word with a not
 * as yet updated error string. This is likely a degenerate case.
 *
 * @param status string to copy into.
 * @param ctx device context (always the device itself).
 * @returns NFP_SUCCESS if successful, other value if error.
 */
static int fsl_get_status(struct nfdev_status_str *status, void *ctx)
{
	struct nfp_cdev *cdev;
	int ne;
	u32 status_data;
	u32 *error = (uint32_t *)status->error;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* check for device */

	cdev = (struct nfp_cdev *)ctx;
	ne = fsl_started(cdev, NFP_WITH_LOCK);
	if (ne != NFP_SUCCESS) {
		nfp_log(NFP_DBG1, "%s: error: unable to get status", __func__);
		return ne;
	}

	/* get status (read immediately with no explicit synchronization
	 * with the firmware)
	 */

	status_data =
		fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_REGISTER_STATUS);
	status->status = FROM_LE32_MEM(&status_data);
	error[0] =
		fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_REGISTER_ERROR_LO);
	error[1] =
		fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_REGISTER_ERROR_HI);

	return NFP_SUCCESS;
}

/**
 * Initiates a device read request.
 *
 * @param addr 32-bit bus address used by DMA to push reply from device.
 * @param len maximum length data to return.
 * @param ctx device context (always the device itself).
 * @returns NFP_SUCCESS if read initiated, NFP_ESTARTING if device not ready,
 * or other value if error.
 */
static int fsl_ensure_reading(unsigned int addr, int len, void *ctx,
			      int lock_flag)
{
	struct nfp_cdev *cdev;
	u32 hdr[3];
	u32 tmp32;
	int ne;
	int hdr_len;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* check for device */

	cdev = (struct nfp_cdev *)ctx;
	ne = fsl_started(cdev, lock_flag);
	if (ne != NFP_SUCCESS) {
		cdev->stats.ensure_fail++;
		nfp_log(NFP_DBG1, "%s: error: unable to initiate read",
			__func__);
		return ne;
	}

	nfp_log(NFP_DBG3, "%s: cdev->bar[cdev->active_bar]= %x",
		__func__, cdev->bar[cdev->active_bar]);

	/* send read request */

	if (addr) {
		nfp_log(NFP_DBG3,
			"%s: requesting DMA reply to bus address %x",
			__func__, addr);
		TO_LE32_MEM(&hdr[0], NFPCI_JOB_CONTROL_PCI_PUSH);
		TO_LE32_MEM(&hdr[1], len);
		TO_LE32_MEM(&hdr[2], addr);
		hdr_len = 12;
	} else {
		TO_LE32_MEM(&hdr[0], NFPCI_JOB_CONTROL);
		TO_LE32_MEM(&hdr[1], len);
		hdr_len = 8;
	}
	ne = nfp_copy_to_dev(cdev, cdev->active_bar,
			     NFPCI_JOBS_RD_CONTROL, (char const *)hdr, hdr_len);
	if (ne != NFP_SUCCESS) {
		nfp_log(NFP_DBG1,
			"%s: error: nfp_copy_to_dev failed", __func__);
		cdev->stats.ensure_fail++;
		return ne;
	}

	/* confirm read request */

	ne = nfp_copy_from_dev(cdev, cdev->active_bar, NFPCI_JOBS_RD_LENGTH,
			       (char *)hdr, 4);
	if (ne != NFP_SUCCESS) {
		nfp_log(NFP_DBG1, "%s: error: nfp_copy_from_dev failed",
			__func__);
		cdev->stats.ensure_fail++;
		return ne;
	}
	TO_LE32_MEM(&tmp32, len);
	if (hdr[0] != tmp32) {
		nfp_log(NFP_DBG1,
			"%s: error: expected length not written (%08x != %08x)",
			__func__, hdr[0], tmp32);
		cdev->stats.ensure_fail++;
		return NFP_EIO;
	}

	/* trigger read request */

	TO_LE32_MEM(&tmp32, NFAST_INT_HOST_READ_REQUEST);
	fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_CMD, tmp32);

	cdev->stats.ensure++;

	nfp_log(NFP_DBG3, "%s: requesting max %d bytes",
		__func__, len);

	return NFP_SUCCESS;
}

/**
 * Reads a device read reply.
 *
 * @param block data buffer to copy into.
 * @param len maximum length of data to copy.
 * @param ctx device context (always the device itself).
 * @param rcnt returned actual # of bytes copied
 * @returns NFP_SUCCESS if read initiated, NFP_ESTARTING if device not ready,
 * or other value if error.
 */
static int fsl_read(char *block, int len, void *ctx, int *rcnt)
{
	struct nfp_cdev *cdev;
	int ne;
	int cnt;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	*rcnt = 0;

	/* check for device */

	cdev = (struct nfp_cdev *)ctx;
	ne = fsl_started(cdev, NFP_WITH_LOCK);
	if (ne != NFP_SUCCESS) {
		cdev->stats.read_fail++;
		nfp_log(NFP_DBG1, "%s: error: unable to complete read",
			__func__);
		return ne;
	}

	/* receive reply length */

	ne = nfp_copy_from_dev(cdev, cdev->active_bar, NFPCI_JOBS_RD_LENGTH,
			       (char *)&cnt, 4);
	if (ne != NFP_SUCCESS) {
		cdev->stats.read_fail++;
		nfp_log(NFP_DBG1, "%s: error: nfp_copy_from_dev failed.",
			__func__);
		return ne;
	}
	cnt = FROM_LE32_MEM(&cnt);
	nfp_log(NFP_DBG3, "%s: cnt=%u.", __func__, cnt);
	if (cnt < 0 || cnt > len) {
		cdev->stats.read_fail++;
		nfp_log(NFP_DBG1,
			"%s: error: bad byte count (%d) from device",
			__func__, cnt);
		return NFP_EIO;
	}

	/* receive data */

	ne = nfp_copy_to_user_from_dev(cdev, cdev->active_bar,
				       NFPCI_JOBS_RD_DATA, block, cnt);
	if (ne != NFP_SUCCESS) {
		cdev->stats.read_fail++;
		nfp_log(NFP_DBG1, "%s: error: nfp_copy_to_user failed.",
			__func__);
		return ne;
	}

	*rcnt = cnt;
	cdev->stats.read_block++;
	cdev->stats.read_byte += cnt;
	nfp_log(NFP_DBG2, "%s: read %d bytes (std)", __func__, cnt);

	return NFP_SUCCESS;
}

/**
 * Initiates a device write request.
 *
 * @param addr 32-bit bus address used by DMA to pull request to device.
 * @param block data buffer to copy from.
 * @param len length of data to copy.
 * @param ctx device context (always the device itself).
 * @returns NFP_SUCCESS if write successful, NFP_ESTARTING if device not
 * ready, or other value if error.
 */
static int fsl_write(unsigned int addr, char const *block, int len,
		     void *ctx)
{
	struct nfp_cdev *cdev;
	unsigned int hdr[3];
	int ne;
	unsigned int tmp32;

	nfp_log(NFP_DBG4, "%s: entered", __func__);

	/* check for device */

	cdev = (struct nfp_cdev *)ctx;
	ne = fsl_started(cdev, NFP_WITH_LOCK);
	if (ne != NFP_SUCCESS) {
		cdev->stats.write_fail++;
		nfp_log(NFP_DBG1, "%s: error: unable to initiate write",
			__func__);
		return ne;
	}

	if (addr == 0) {
		/* std write */

		nfp_log(NFP_DBG3, "%s: cdev->bar[cdev->active_bar]= %x",
			__func__, cdev->bar[cdev->active_bar]);
		nfp_log(NFP_DBG3, "%s: block len %d", __func__, len);

		/* send write request */

		ne = nfp_copy_from_user_to_dev(cdev, cdev->active_bar,
					       NFPCI_JOBS_WR_DATA, block, len);
		if (ne != NFP_SUCCESS) {
			cdev->stats.write_fail++;
			nfp_log(NFP_DBG1,
				"%s: error: nfp_copy_from_user_to_dev failed",
				__func__);
			return ne;
		}

		TO_LE32_MEM(&hdr[0], NFPCI_JOB_CONTROL);
		TO_LE32_MEM(&hdr[1], len);
		ne = nfp_copy_to_dev(cdev, cdev->active_bar,
				     NFPCI_JOBS_WR_CONTROL,
					 (char const *)hdr, 8);
		if (ne != NFP_SUCCESS) {
			cdev->stats.write_fail++;
			nfp_log(NFP_DBG1,
				"%s: error: nfp_copy_to_dev failed", __func__);
			return ne;
		}

		/* confirm write request */

		ne = nfp_copy_from_dev(cdev, cdev->active_bar,
				       NFPCI_JOBS_WR_LENGTH, (char *)hdr, 4);
		if (ne != NFP_SUCCESS) {
			cdev->stats.write_fail++;
			nfp_log(NFP_DBG1,
				"%s: nfp_copy_from_dev failed", __func__);
			return ne;
		}

		TO_LE32_MEM(&tmp32, len);
		if (hdr[0] != tmp32) {
			cdev->stats.write_fail++;
			nfp_log(NFP_DBG1,
				"%s: length not written (%08x != %08x)",
				__func__, hdr[0], tmp32);
			return NFP_EIO;
		}
	} else {
		/* dma write */

		nfp_log(NFP_DBG3, "%s: cdev->bar[cdev->active_bar]= %x",
			__func__, cdev->bar[cdev->active_bar]);
		nfp_log(NFP_DBG3, "%s: block len %d", __func__, len);
		nfp_log(NFP_DBG3, "%s: pull from 0x%016x using DMA",
			__func__, addr);

		/* submit write request */

		TO_LE32_MEM(&hdr[0], NFPCI_JOB_CONTROL_PCI_PULL);
		TO_LE32_MEM(&hdr[1], len);
		TO_LE32_MEM(&hdr[2], addr);
		ne = nfp_copy_to_dev(cdev, cdev->active_bar,
				     NFPCI_JOBS_WR_CONTROL, (char const *)hdr,
					 12);
		if (ne != NFP_SUCCESS) {
			cdev->stats.write_fail++;
			nfp_log(NFP_DBG1, "%s: nfp_copy_to_dev failed",
				__func__);
			return ne;
		}

		/* confirm request */

		ne = nfp_copy_from_dev(cdev, cdev->active_bar,
				       NFPCI_JOBS_WR_LENGTH, (char *)hdr, 4);
		if (ne != NFP_SUCCESS) {
			cdev->stats.write_fail++;
			nfp_log(NFP_DBG1,
				"%s: nfp_copy_from_dev failed", __func__);
			return ne;
		}

		TO_LE32_MEM(&tmp32, len);
		if (hdr[0] != tmp32) {
			cdev->stats.write_fail++;
			nfp_log(NFP_DBG1,
				"%s: length not written (%08x != %08x)",
				__func__, tmp32, hdr[0]);
			return NFP_EIO;
		}
	}

	/* trigger write */

	TO_LE32_MEM(&tmp32, NFAST_INT_HOST_WRITE_REQUEST);
	fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_CMD, tmp32);

	cdev->stats.write_block++;
	cdev->stats.write_byte += len;

	nfp_log(NFP_DBG3, "%s: done", __func__);
	return NFP_SUCCESS;
}

/** FSL Sawshark T1022 device configuration. */
const struct nfpcmd_dev fsl_t1022_cmddev = { "nCipher Next Gen PCI",
				      PCI_VENDOR_ID_FREESCALE,
				      PCI_DEVICE_ID_FREESCALE_T1022,
				      PCI_VENDOR_ID_NCIPHER,
				      PCI_SUBSYSTEM_ID_NFAST_REV1,
				      { 0, FSL_MEMSIZE, 0, 0, 0, 0 },
				      NFP_CMD_FLG_NEED_MSI,
				      NFDEV_IF_PCI_PULL,
				      fsl_create,
				      fsl_destroy,
				      fsl_started,
				      fsl_stopped,
				      fsl_open,
				      fsl_close,
				      fsl_isr,
				      fsl_write,
				      fsl_read,
				      fsl_chupdate,
				      fsl_ensure_reading,
				      0,
				      fsl_set_control,
				      fsl_get_status };
/* end of file */
