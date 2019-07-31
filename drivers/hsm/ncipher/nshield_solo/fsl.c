/*
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this source file; if not, write to the Free Software
 *      Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*

 fsl.c: COMPANY PCI HSM FSL command driver

 (C) Copyright COMPANY Corporation Ltd 2002 All rights reserved

 history

 09/10/2001 jsh  Original

 */

#include "fsl.h"
#include "fsl_osif.h"
#include "nfp_common.h"
#include "nfp_error.h"
#include "nfp_hostif.h"
#include "nfp_osif.h"
#include "nfp_cmd.h"
#include "nfpci.h"


#ifdef _MSC_VER
#define __func__ __FUNCTION__
#endif


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
static nfp_err fsl_create(nfp_cdev *cdev) {
    uint32_t clr = 0;

    nfp_log(NFP_DBG4, "fsl_create: entered");

    if (cdev == NULL) {
        nfp_log(NFP_DBG1, "fsl_create: error: no device");
        return NFP_ENODEV;
    }

    if (cdev->created) {
        nfp_log(NFP_DBG3, "fsl_create: device already created");
        return NFP_SUCCESS;
    }

    cdev->active_bar = -1;
    cdev->detection_type = NFP_EPOLLING;
    cdev->conn_status = NFP_ESTARTING;

    /* try to reset check doorbell registers (don't read back in case they hang) */
    TO_LE32_MEM(&clr, NFAST_INT_DEVICE_CLR);

#if defined(__unix__) || defined(__sun) || defined(__hpux)
    cdev->active_bar = FSL_MEMBAR;
    fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_CS_STATUS, clr);
#elif defined(WINVER)
    /* perform check to see if Bar0 and Bar1 are open for backwards compatibility.
    If both bars are open Element [1] in the array will be populated */
    if (cdev->bar[FSL_MEMBAR] == NULL) {
        cdev->active_bar = 0;
        fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_CS_STATUS, clr);
    } else {
        cdev->active_bar = 1;
        fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_CS_STATUS, clr);
    }
#endif

    if (cdev->bar[cdev->active_bar] == NULL) {
        nfp_log(NFP_DBG1, "fsl_create: error: null FSL memory BAR[%d]", cdev->active_bar);
        return NFP_ENOMEM;
    }

    /* set our context to just be a pointer to ourself */
    cdev->cmdctx = cdev;

    /* try to reset read/write doorbell registers (don't read back in case they hang) */
    nfp_log(NFP_DBG3, "fsl_create: clearing read/write doorbell registers");
    TO_LE32_MEM(&clr, NFAST_INT_HOST_CLR);
    fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_CMD, clr);
    fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_CMD, clr);
	
    nfp_log(NFP_DBG3, "fsl_create: exiting fsl_create active_bar: %d.", cdev->active_bar);

    cdev->created = 1;
    
    return NFP_SUCCESS;
}

/**
 * Destroys an FSL device.
 *
 * @param ctx device context (always the device itself).
 * @returns NFP_SUCCESS if successful, other value if error.
 */
static nfp_err fsl_destroy(void *ctx) {
    nfp_cdev *cdev;
    unsigned int tmp32;

    nfp_log(NFP_DBG4, "fsl_destroy: entered");

    /* check for device */
    cdev = (nfp_cdev *)ctx;
    if (cdev == NULL) {
        nfp_log(NFP_DBG1, "fsl_destroy: warning: no device");
        return NFP_ENODEV;
    }

    /* clear doorbell registers */
    if (cdev->bar[cdev->active_bar] != NULL) {
        nfp_log(NFP_DBG3, "fsl_destroy: clearing doorbell registers");
        TO_LE32_MEM(&tmp32, NFAST_INT_DEVICE_CLR);
        fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_STATUS, tmp32);
        fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_STATUS, tmp32);
        fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_CS_STATUS, tmp32);
    } else {
        nfp_log(NFP_DBG1, "fsl_destroy: warning: no FSL BAR[%d] memory", cdev->active_bar);
    }

    return NFP_SUCCESS;
}

/**
 * Returns fsl_created status.
 *
 * @param cdev common device.
 * @returns NFP_SUCCESS if created or other value if error.
 */
static nfp_err fsl_created(nfp_cdev *cdev) {
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

    if (cdev->bar[cdev->active_bar] == NULL) {
        nfp_log(NFP_DBG1, "%s: error: no FSL BAR[%d] memory", __func__, cdev->active_bar);
        return NFP_ENOMEM;
    }

    return NFP_SUCCESS;
}

/* ----------------------------------------------------- */
/* This call needs to be in synch with the ISR or the ISR will come in the middle 
*  of a write/read op and cause problems.   
* 
*/
static int fsl_connection_status(nfp_cdev *cdev, int lock_flag, nfp_err epd_status) {
       
    nfp_err status = NFP_ESTARTING;
    (void) lock_flag;

    if (!cdev) {
       return NFP_ENODEV;
    }

    /* this code is mainly to support backwards compatibility with the interrupt driven approach
     * to detection.
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
 * @returns NFP_SUCCESS if started, NFP_ESTARTING if not ready, or other value if error.
 */
static nfp_err fsl_started(nfp_cdev *cdev, int lock_flag) {
    nfp_err status = NFP_ESTARTING;
    nfp_err epd_status = NFP_ESTARTING;
    uint32_t doorbell_cs = 0x0;

    nfp_log(NFP_DBG4, "%s: entered", __func__);

    /* check for device */

    if (!cdev) {
        nfp_log(NFP_DBG1, "%s: error: no device", __func__);
        return NFP_ENODEV;
    }

    if (cdev->bar[cdev->active_bar] == NULL) {
        nfp_log(NFP_DBG1, "%s: error: no FSL BAR[%d] memory", __func__, cdev->active_bar);
        return NFP_ENOMEM;
    }
    
    /* check the status register to see if epd has started */
    doorbell_cs = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_POLLING);
    doorbell_cs = FROM_LE32_MEM(&doorbell_cs);
    nfp_log(NFP_DBG3, "%s: doorbell_polling is: %x", __func__,doorbell_cs);
    
    if(doorbell_cs == NFAST_INT_DEVICE_POLL){
        epd_status = NFP_EPOLLING;
        nfp_log(NFP_DBG3, "%s: EPD in polling mode", __func__);
    }
    else if(doorbell_cs == NFAST_INT_DEVICE_PCI_DOWN){
        epd_status = NFAST_INT_DEVICE_PCI_DOWN;
    }
    /* check current connection status */
    status = fsl_connection_status(cdev, lock_flag,epd_status);

    if (status == NFP_SUCCESS) {
        nfp_log(NFP_DBG3, "%s: device started", __func__);
    } else if (status == NFP_ESTARTING) {
        nfp_log(NFP_DBG3, "%s: device starting", __func__);
    } else {
        nfp_log(NFP_DBG1, "%s: error: device failure: code 0x%x", __func__, status);
    }

    return status;
}

/**
 * Resets connection check status.
 *
 * This can be called by the device driver when an I/O operation times out. This causes any
 * following I/O operations to fail quickly until fsl_started() has been set successfully by a
 * periodic check interrupt.
 *
 * @param cdev common device.
 * @returns NFP_SUCCESS if stopped or other value if error.
 */
static nfp_err fsl_stopped(nfp_cdev *cdev) {

    nfp_log(NFP_DBG4, "%s: entered", __func__);

    /* check for device */

    if (!cdev) {
        nfp_log(NFP_DBG1, "%s: error: no device", __func__);
        return NFP_ENODEV;
    }

    if (cdev->bar[cdev->active_bar] == NULL) {
        nfp_log(NFP_DBG1, "%s: error: no FSL BAR[%d] memory", __func__, cdev->active_bar);
        return NFP_ENOMEM;
    }

    /* reset current connection status */

    cdev->conn_status = NFP_ESTARTING;

    nfp_log(NFP_DBG2, "%s: device stopped", __func__);

    return NFP_SUCCESS;
}

static int fsl_update_connection_status(nfp_cdev *cdev, int status) {

    int current_status;
    if (!cdev) {
        return NFP_ENODEV;
    }

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
static void fsl_check_complete(nfp_cdev *cdev, nfp_err status) {
    nfp_err ne;
    uint32_t clr, chk;
    int started;

    nfp_log(NFP_DBG4, "%s: entered", __func__);

    /* check for device */

    if ((ne = fsl_created(cdev)) != NFP_SUCCESS) {
        nfp_log(NFP_DBG1, "%s: error: check not completed", __func__);
        return;
    }

    /* started becomes true after fsl_create and the first cs interrupt is successful.
	*  It switches to false right afer since cs_status is set to NFP_SUCCESS right after this check.
	*  A fsl_close or a fsl_create can reset the cs_status to NFP_ESTARTING again.
	*/
   started = (fsl_update_connection_status(cdev, status) == NFP_ESTARTING) && (status == NFP_SUCCESS);
    
   /* reset read/write doorbell registers if just started */

    if (started) {
        nfp_log(NFP_DBG3, "fsl_create: clearing read/write doorbell registers");
        TO_LE32_MEM(&clr, NFAST_INT_HOST_CLR);
        fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_CMD, clr);
        chk = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_CMD);
        fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_CMD, clr);
        chk = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_CMD);
        TO_LE32_MEM(&clr, NFAST_INT_DEVICE_CLR);
        fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_STATUS, clr);
        chk = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_STATUS);
        fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_STATUS, clr);
        chk = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_STATUS);
    }

    if (status == NFP_SUCCESS) {
        nfp_log(NFP_DBG3, "fsl_check_complete: device started");
    } else if (status == NFP_ESTARTING) {
        nfp_log(NFP_DBG3, "fsl_check_complete: device not started yet");
    } else {
        nfp_log(NFP_DBG1, "fsl_check_complete: device check failed with code: 0x%x", status);
    }
}

/**
 * Handles an interrupt from the FSL device.
 *
 * @param ctx device context (always the device itself).
 * @param handled set non-zero by this routine if interrupt considered handled
 * @returns NFP_SUCCESS if successful, other value if error.
 */
static nfp_err fsl_isr(void *ctx, int *handled) {
    nfp_cdev *cdev;
    nfp_err ne;
    uint32_t doorbell_rd, doorbell_wr, doorbell_cs;
#ifdef FSL_AGRESSIVE_CHECKING
    int x_wr = 0, x_rd = 0, x_cs = 0;
#endif

    nfp_log(NFP_DBG4, "%s: entered", __func__);
    
    /* mark not yet handled */

    *handled = 0;

    /* check for device */

    cdev = (nfp_cdev *)ctx;
    if ((ne = fsl_created(cdev)) != NFP_SUCCESS) {
        nfp_log(NFP_DBG1, "%s: error: interrupt not handled", __func__);
        return ne;
    }

    ++cdev->stats.isr;

    doorbell_wr = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_STATUS);
    doorbell_wr = FROM_LE32_MEM(&doorbell_wr);
    doorbell_rd = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_STATUS);
    doorbell_rd = FROM_LE32_MEM(&doorbell_rd);
    doorbell_cs = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_CS_STATUS);
    doorbell_cs = FROM_LE32_MEM(&doorbell_cs);
    nfp_log(NFP_DBG3, "%s: cs:= %x,rd:=%x,wr:=%x", __func__,doorbell_cs,doorbell_rd,doorbell_wr);

        
    while (doorbell_rd || doorbell_wr || doorbell_cs ) {
        /* prevent any illegal combination of set bits from triggering processing. Note that if anyone of these registers
         * have an incorrect bit set, it would prevent the other operations from being processed since we return from the ISR,
         * even if they have legal values. This is an unlikely scenario since these registers are written either to 0 or one
         * of the legal values by the software on the card.*/
        if ((doorbell_cs) && ((doorbell_cs & ~NFAST_INT_DEVICE_CHECK_OK) && (doorbell_cs & ~NFAST_INT_DEVICE_CHECK_FAILED))){
            nfp_log(NFP_DBG1, "fsl_isr: illegal bits in doorbell_cs %x",doorbell_cs);
            *handled = 1;
            /* clear the register*/
            fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_CS_STATUS, NFAST_INT_DEVICE_CLR);
            return 0;
        } 
        if ((doorbell_rd) && ((doorbell_rd & ~NFAST_INT_DEVICE_READ_OK) && (doorbell_rd & ~NFAST_INT_DEVICE_READ_FAILED))){
            nfp_log(NFP_DBG1, "fsl_isr: illegal bits in doorbell_rd %x",doorbell_rd);
            *handled = 1;
            /* clear the register*/
            fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_STATUS, NFAST_INT_DEVICE_CLR);
            return 0;
        }
        if ((doorbell_wr) && ((doorbell_wr & ~NFAST_INT_DEVICE_WRITE_OK) && (doorbell_wr & ~NFAST_INT_DEVICE_WRITE_FAILED))){
            nfp_log(NFP_DBG1, "fsl_isr: illegal bits in doorbell_wr %x",doorbell_wr);
            /* clear the register*/
            fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_STATUS, NFAST_INT_DEVICE_CLR);
            *handled = 1;
            return 0;
        }         
        
        /* service interrupts. if we made it here, the doorbell registers are all valid, so no need to check
         * for their validity anymore. */

        if (doorbell_wr) {
            fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_STATUS, NFAST_INT_DEVICE_CLR);
            cdev->stats.isr_write++;
            nfp_write_complete(cdev->dev, doorbell_wr & NFAST_INT_DEVICE_WRITE_OK ? 1 : 0);
            nfp_log(NFP_DBG3, "fsl_isr: acknowledging write interrupt: ok = %d", doorbell_wr & NFAST_INT_DEVICE_WRITE_OK ? 1 : 0);

#ifdef FSL_AGRESSIVE_CHECKING
            if (fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_STATUS) != NFAST_INT_DEVICE_CLR) {
                nfp_log(NFP_DBG1, "fsl_isr: failed to clear doorbell write status");
            } ++x_wr;
#endif
        }

        if (doorbell_rd) {
            fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_STATUS, NFAST_INT_DEVICE_CLR);
            cdev->stats.isr_read++;
            nfp_read_complete(cdev->dev, doorbell_rd & NFAST_INT_DEVICE_READ_OK ? 1 : 0);
            nfp_log(NFP_DBG3, "fsl_isr: acknowledging read interrupt: ok = %d", doorbell_rd & NFAST_INT_DEVICE_READ_OK ? 1 : 0);

#ifdef FSL_AGRESSIVE_CHECKING
            if (fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_STATUS) != NFAST_INT_DEVICE_CLR) {
                nfp_log(NFP_DBG1, "fsl_isr: failed to clear doorbell read status");
            } ++x_rd;
#endif
        }
        /* the doorbell_cs is being phased out in favor of polling since there were issues caused by this interrupt being issued from
         * the card on its own when the driver was not even present. To maintain backwards compatibility, this code is being kept, but
         * might be removed in the future.*/
        nfp_log(NFP_DBG3, "fsl_isr: doorbell_cs is: %x",doorbell_cs);
        if (doorbell_cs) {
            fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_CS_STATUS, NFAST_INT_DEVICE_CLR);
            fsl_check_complete(cdev, doorbell_cs & NFAST_INT_DEVICE_CHECK_OK ? NFP_SUCCESS : NFP_ESTARTING);
            nfp_log(NFP_DBG3, "fsl_isr: acknowledging check interrupt: status:0x%x", doorbell_cs & NFAST_INT_DEVICE_CHECK_OK ? NFP_SUCCESS : NFP_ESTARTING);
 
#ifdef FSL_AGRESSIVE_CHECKING
            if (fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_CS_STATUS) != NFAST_INT_DEVICE_CLR) {
                nfp_log(NFP_DBG1, "fsl_isr: warning: failed to clear doorbell check status");
            } ++x_cs;
#endif
        }

        doorbell_wr = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_STATUS);
        doorbell_wr = FROM_LE32_MEM(&doorbell_wr);
        doorbell_rd = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_STATUS);
        doorbell_rd = FROM_LE32_MEM(&doorbell_rd);
        doorbell_cs = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_CS_STATUS);
        doorbell_cs = FROM_LE32_MEM(&doorbell_cs);
        
        nfp_log(NFP_DBG3, "fsl_isr: cs status in isr is: %x",doorbell_cs);
    }

    /* always report the interrupt as handled */

    *handled = 1;
#ifdef FSL_AGRESSIVE_CHECKING
    if (x_wr + x_rd + x_cs == 0) {
        nfp_log(NFP_DBG2, "fsl_isr: no operations handled by this ISR call");
    } else {
        if (x_wr + x_rd + x_cs > 1) {
            nfp_log(NFP_DBG3, "fsl_isr: DEBUG: multiple operations handled by this ISR call: wr=%d, rd=%d, cs=%d", x_wr, x_rd, x_cs);
        } else {
            nfp_log(NFP_DBG3, "fsl_isr: DEBUG: one operation handled by this ISR call: wr=%d, rd=%d, cs=%d", x_wr, x_rd, x_cs);
        }

    }
#endif
    nfp_log(NFP_DBG3, "fsl_isr: exiting");

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
static nfp_err fsl_open(void *ctx) {
    nfp_cdev *cdev;
    nfp_err ne;
	

    nfp_log(NFP_DBG4, "fsl_open: entered");

    /* check for device */

    cdev = (nfp_cdev *) ctx;
    if ((ne = fsl_started(cdev,NFP_WITH_LOCK)) != NFP_SUCCESS) {
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
static nfp_err fsl_close(void *ctx) {
    (void) ctx;

    nfp_log(NFP_DBG4, "fsl_close: entered");

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
static nfp_err fsl_chupdate(char *data, int len, void *ctx) {
    (void) data; (void) len; (void) ctx;

    nfp_log( NFP_DBG1, "fsl_chupdate: warning: not implemented");

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
static nfp_err fsl_set_control(const nfdev_control_str *control, void *ctx) {
    uint32_t control_data;
    nfp_cdev *cdev;
    nfp_err ne;

    nfp_log(NFP_DBG4, "%s: entered", __func__);

    /* check for device */

    cdev = (nfp_cdev *) ctx;
    if ((ne = fsl_started(cdev,NFP_WITH_LOCK)) != NFP_SUCCESS) {
        nfp_log(NFP_DBG1, "%s: error: unable to set control", __func__);
        return ne;
    }

    /* set control (written immediately with no explicit synchronization with the firmware)*/

    TO_LE32_MEM(&control_data, control->control);
    fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_REGISTER_CONTROL, control_data);

    return NFP_SUCCESS;
}

/**
 * Returns status data.
 *
 * The device status registers are read immediately. No doorbell style handshake
 * is used. Without explicit synchronization, it is possible that an inconsistent state
 * may be returned if the status is being updated by the firmware while simultaneously
 * being read by the host. For example, the call could return an updated status word
 * with a not as yet updated error string. This is likely a degenerate case.
 *
 * @param status string to copy into.
 * @param ctx device context (always the device itself).
 * @returns NFP_SUCCESS if successful, other value if error.
 */
static nfp_err fsl_get_status(nfdev_status_str *status, void *ctx) {
    nfp_cdev *cdev;
    nfp_err ne;
    uint32_t status_data;
    uint32_t *error = (uint32_t *) status->error;

    nfp_log(NFP_DBG4, "%s: entered", __func__);

    /* check for device */

    cdev = (nfp_cdev *) ctx;
    if ((ne = fsl_started(cdev,NFP_WITH_LOCK)) != NFP_SUCCESS) {
        nfp_log(NFP_DBG1, "%s: error: unable to get status", __func__);
        return ne;
    }

    /* get status (read immediately with no explicit synchronization with the firmware) */

    status_data = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_REGISTER_STATUS);
    status->status = FROM_LE32_MEM(&status_data);
    error[0] = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_REGISTER_ERROR_LO);
    error[1] = fsl_inl(cdev, cdev->active_bar, FSL_OFFSET_REGISTER_ERROR_HI);

    return NFP_SUCCESS;
}

/**
 * Initiates a device read request.
 *
 * @param addr 32-bit bus address used by DMA to push reply from device.
 * @param len maximum length data to return.
 * @param ctx device context (always the device itself).
 * @returns NFP_SUCCESS if read initiated, NFP_ESTARTING if device not ready, or other value if error.
 */
static nfp_err fsl_ensure_reading(int unsigned addr, int len, void *ctx, int lock_flag) {
    nfp_cdev *cdev;
    uint32_t hdr[3];
    uint32_t tmp32;
    nfp_err ne;
    int hdr_len;

    nfp_log(NFP_DBG4, "%s: entered", __func__);

    /* check for device */

    cdev = (nfp_cdev *) ctx;
	if ((ne = fsl_started(cdev, lock_flag)) != NFP_SUCCESS) {
        cdev->stats.ensure_fail++;
        nfp_log(NFP_DBG1, "%s: error: unable to initiate read", __func__);
        return ne;
    }

    nfp_log( NFP_DBG3, "fsl_ensure_reading: cdev->bar[cdev->active_bar]= %x",  cdev->bar[cdev->active_bar]);

    /* send read request */

    if (addr) {
        nfp_log(NFP_DBG3, "fsl_ensure_reading: requesting DMA reply to bus address %x", addr);
        TO_LE32_MEM(&hdr[0], NFPCI_JOB_CONTROL_PCI_PUSH);
        TO_LE32_MEM(&hdr[1], len);
        TO_LE32_MEM(&hdr[2], addr);
        hdr_len = 12;
    } else {
        TO_LE32_MEM(&hdr[0], NFPCI_JOB_CONTROL);
        TO_LE32_MEM(&hdr[1], len);
        hdr_len = 8;
    }
    if ((ne = nfp_copy_to_dev(cdev, cdev->active_bar, NFPCI_JOBS_RD_CONTROL, (char const *)hdr, hdr_len)) != NFP_SUCCESS) {
        nfp_log(NFP_DBG1, "fsl_ensure_reading: error: nfp_copy_to_dev failed");
        cdev->stats.ensure_fail++;
        return ne;
    }

    /* confirm read request */

    if ((ne = nfp_copy_from_dev(cdev, cdev->active_bar, NFPCI_JOBS_RD_LENGTH, (char *)hdr, 4)) != NFP_SUCCESS) {
        nfp_log(NFP_DBG1, "fsl_ensure_reading: error: nfp_copy_from_dev failed");
        cdev->stats.ensure_fail++;
        return ne;
    }
    TO_LE32_MEM(&tmp32, len);
    if (hdr[0] != tmp32) {
        nfp_log(NFP_DBG1, "fsl_ensure_reading: error: expected length not written (%08x != %08x)", hdr[0], tmp32);
        cdev->stats.ensure_fail++;
        return NFP_EIO;
    }

    /* trigger read request */

    TO_LE32_MEM(&tmp32, NFAST_INT_HOST_READ_REQUEST);
    fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_RD_CMD, tmp32);

    cdev->stats.ensure++;

    nfp_log(NFP_DBG3, "fsl_ensure_reading: requesting max %d bytes", len);

    return NFP_SUCCESS;
}

/**
 * Reads a device read reply.
 *
 * @param block data buffer to copy into.
 * @param len maximum length of data to copy.
 * @param ctx device context (always the device itself).
 * @param rcnt returned actual # of bytes copied
 * @returns NFP_SUCCESS if read initiated, NFP_ESTARTING if device not ready, or other value if error.
 */
static nfp_err fsl_read(char *block, int len, void *ctx, int *rcnt) {
    nfp_cdev *cdev;
    nfp_err ne;
    int cnt;

    nfp_log(NFP_DBG4, "%s: entered", __func__);

    *rcnt = 0;

    /* check for device */

    cdev = (nfp_cdev *)ctx;
    if ((ne = fsl_started(cdev,NFP_WITH_LOCK)) != NFP_SUCCESS) {
        cdev->stats.read_fail++;
        nfp_log(NFP_DBG1, "%s: error: unable to complete read", __func__);
        return ne;
    }

    /* receive reply length */

    if ((ne = nfp_copy_from_dev(cdev, cdev->active_bar, NFPCI_JOBS_RD_LENGTH, (char *)&cnt, 4)) != NFP_SUCCESS) {
        cdev->stats.read_fail++;
        nfp_log(NFP_DBG1, "fsl_read: error: nfp_copy_from_dev failed.");
        return ne;
    }
    cnt = FROM_LE32_MEM(&cnt);
    nfp_log(NFP_DBG3, "fsl_read: cnt=%u.", cnt);
    if (cnt < 0 || cnt > len) {
        cdev->stats.read_fail++;
        nfp_log(NFP_DBG1, "fsl_read: error: bad byte count (%d) from device", cnt);
        return NFP_EIO;
    }

    /* receive data */

    if ((ne = nfp_copy_to_user_from_dev(cdev, cdev->active_bar, NFPCI_JOBS_RD_DATA, block, cnt)) != NFP_SUCCESS) {
        cdev->stats.read_fail++;
        nfp_log(NFP_DBG1, "fsl_read: error: nfp_copy_to_user failed.");
        return ne;
    }

    *rcnt = cnt;
    cdev->stats.read_block++;
    cdev->stats.read_byte += cnt;
    nfp_log(NFP_DBG2, "fsl_read: read %d bytes (std)", cnt);

    return NFP_SUCCESS;
}

/**
 * Initiates a device write request.
 *
 * @param addr 32-bit bus address used by DMA to pull request to device.
 * @param block data buffer to copy from.
 * @param len length of data to copy.
 * @param ctx device context (always the device itself).
 * @returns NFP_SUCCESS if write successful, NFP_ESTARTING if device not ready, or other value if error.
 */
static nfp_err fsl_write(int unsigned addr, char const *block, int len, void *ctx) {
    nfp_cdev *cdev;
    unsigned int hdr[3];
    nfp_err ne;
    unsigned int tmp32;

    nfp_log(NFP_DBG4, "%s: entered", __func__);

    /* check for device */

    cdev = (nfp_cdev *) ctx;
    if ((ne = fsl_started(cdev,NFP_WITH_LOCK)) != NFP_SUCCESS) {
        cdev->stats.write_fail++;
        nfp_log(NFP_DBG1, "%s: error: unable to initiate write", __func__);
        return ne;
    }

    if (0 == addr) {
        /* std write */

        nfp_log(NFP_DBG3, "fsl_write: cdev->bar[cdev->active_bar]= %x", cdev->bar[cdev->active_bar]);
        nfp_log(NFP_DBG3, "fsl_write: block len %d", len);

        /* send write request */

        if ((ne = nfp_copy_from_user_to_dev(cdev, cdev->active_bar, NFPCI_JOBS_WR_DATA, block, len)) != NFP_SUCCESS) {
            cdev->stats.write_fail++;
            nfp_log(NFP_DBG1, "fsl_write: error: nfp_copy_from_user_to_dev failed");
            return ne;
        }

        TO_LE32_MEM(&hdr[0], NFPCI_JOB_CONTROL);
        TO_LE32_MEM(&hdr[1], len);
        if ((ne = nfp_copy_to_dev(cdev, cdev->active_bar, NFPCI_JOBS_WR_CONTROL, (char const *)hdr, 8)) != NFP_SUCCESS) {
            cdev->stats.write_fail++;
            nfp_log(NFP_DBG1, "fsl_write: error: nfp_copy_to_dev failed");
            return ne;
        }

        /* confirm write request */

        if ((ne = nfp_copy_from_dev(cdev, cdev->active_bar, NFPCI_JOBS_WR_LENGTH, (char *)hdr, 4)) != NFP_SUCCESS) {
            cdev->stats.write_fail++;
            nfp_log(NFP_DBG1, "fsl_write: nfp_copy_from_dev failed");
            return ne;
        }

        TO_LE32_MEM(&tmp32, len);
        if (hdr[0] != tmp32) {
            cdev->stats.write_fail++;
            nfp_log(NFP_DBG1, "fsl_write: length not written (%08x != %08x)", hdr[0], tmp32);
            return NFP_EIO;
        }
    } else {
        /* dma write */

        nfp_log(NFP_DBG3, "fsl_write: cdev->bar[cdev->active_bar]= %x", cdev->bar[cdev->active_bar]);
        nfp_log(NFP_DBG3, "fsl_write: block len %d", len);
        nfp_log(NFP_DBG3, "fsl_write: pull from 0x%016x using DMA", addr);

        /* submit write request */

        TO_LE32_MEM(&hdr[0], NFPCI_JOB_CONTROL_PCI_PULL);
        TO_LE32_MEM(&hdr[1], len);
        TO_LE32_MEM(&hdr[2], addr);
        if ((ne = nfp_copy_to_dev(cdev, cdev->active_bar, NFPCI_JOBS_WR_CONTROL, (char const *)hdr, 12)) != NFP_SUCCESS) {
            cdev->stats.write_fail++;
            nfp_log(NFP_DBG1, "fsl_write: nfp_copy_to_dev failed");
            return ne;
        }

        /* confirm request */

        if ((ne = nfp_copy_from_dev(cdev, cdev->active_bar, NFPCI_JOBS_WR_LENGTH, (char *)hdr, 4)) != NFP_SUCCESS) {
            cdev->stats.write_fail++;
            nfp_log(NFP_DBG1, "fsl_write: nfp_copy_from_dev failed");
            return ne;
        }

        TO_LE32_MEM(&tmp32, len);
        if (hdr[0] != tmp32) {
            cdev->stats.write_fail++;
            nfp_log(NFP_DBG1, "fsl_write: length not written (%08x != %08x)", tmp32, hdr[0]);
            return NFP_EIO;
        }
    }

    /* trigger write */

    TO_LE32_MEM(&tmp32, NFAST_INT_HOST_WRITE_REQUEST);
    fsl_outl(cdev, cdev->active_bar, FSL_OFFSET_DOORBELL_WR_CMD, tmp32);

    cdev->stats.write_block++;
    cdev->stats.write_byte += len;

    nfp_log(NFP_DBG3, "fsl_write: done");
    return NFP_SUCCESS;
}

/** FSL c293 device configuration. */
const nfpcmd_dev fsl_c293_cmddev = { "COMPANY Next Gen PCI",
        PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_FREESCALE_C293,
        PCI_VENDOR_ID_NCIPHER, PCI_SUBSYSTEM_ID_NFAST_REV1,
        { 0, FSL_MEMSIZE, 0, 0, 0, 0 },
        NFP_CMD_FLG_NEED_MSI, NFDEV_IF_PCI_PUSH, fsl_create, fsl_destroy,
        fsl_started, fsl_stopped, fsl_open, fsl_close, fsl_isr, fsl_write, fsl_read,
        fsl_chupdate, fsl_ensure_reading, 0, 0, 0 };

/** FSL Hammerhead p3041 device configuration. */
const nfpcmd_dev fsl_p3041_cmddev = { "COMPANY Next Gen PCI",
        PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_FREESCALE_P3041,
        PCI_VENDOR_ID_NCIPHER, PCI_SUBSYSTEM_ID_NFAST_REV1,
        { 0, FSL_MEMSIZE, 0, 0, 0, 0 },
        NFP_CMD_FLG_NEED_MSI, NFDEV_IF_PCI_PULL, fsl_create, fsl_destroy,
        fsl_started, fsl_stopped, fsl_open, fsl_close, fsl_isr, fsl_write, fsl_read,
        fsl_chupdate, fsl_ensure_reading, 0, fsl_set_control, fsl_get_status };

/** FSL Sawshark p3041 device configuration. */
#if defined(__unix__) || defined(__sun) || defined(__hpux)
const nfpcmd_dev fsl_t1022_cmddev = { "COMPANY Next Gen PCI",
        PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_FREESCALE_T1022,
        PCI_VENDOR_ID_NCIPHER, PCI_SUBSYSTEM_ID_NFAST_REV1,
        { 0, FSL_MEMSIZE, 0, 0, 0, 0 },
        NFP_CMD_FLG_NEED_MSI, NFDEV_IF_PCI_PULL, fsl_create, fsl_destroy,
        fsl_started, fsl_stopped, fsl_open, fsl_close, fsl_isr, fsl_write, fsl_read,
        fsl_chupdate, fsl_ensure_reading, 0, fsl_set_control, fsl_get_status };
#elif defined(WINVER)
/* NOTE setting FSL_MEMSIZE for both Element [0] and Element [1] in order
to accomodate scenarios when both Bar0 and Bar1 Mem Resources are Available
*/
const nfpcmd_dev fsl_t1022_cmddev = { "COMPANY Next Gen PCI",
        PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_FREESCALE_T1022,
        PCI_VENDOR_ID_NCIPHER, PCI_SUBSYSTEM_ID_NFAST_REV1,
        { FSL_MEMSIZE, FSL_MEMSIZE, 0, 0, 0, 0 },
        NFP_CMD_FLG_NEED_MSI, NFDEV_IF_PCI_PULL, fsl_create, fsl_destroy,
        fsl_started, fsl_stopped, fsl_open, fsl_close, fsl_isr, fsl_write, fsl_read,
        fsl_chupdate, fsl_ensure_reading, 0, fsl_set_control, fsl_get_status };
#endif
/* end of file */
