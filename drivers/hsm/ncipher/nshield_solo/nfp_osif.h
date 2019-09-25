/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *
 * nfp_osif.h: nCipher PCI HSM OS interface declarations
 *
 */

#ifndef NFP_OSIF_H
#define NFP_OSIF_H

/* implementation choices ----------------------------------------- */

/** ISR strict locking.
 *
 * Define this to choose strict locking for state variables changed
 * within ISR calls. If not defined, we are relying on the implicit
 * memory barriers in sleeping and waiting functions and the
 * pseudo-barriers of the read and write related function calls
 * themselves.
 */
#define NFP_USE_ISR_LOCKING

/* general typedefs ----------------------------------------------- */

/* timeouts ------------------------------------------------------ */

void nfp_sleep(int ms);

/* config space access ------------------------------------------------ */

/* return Little Endian 32 bit config register */
int nfp_config_inl(struct nfp_cdev *pdev, int offset, unsigned int *res);

/* io space access ------------------------------------------------ */

unsigned int nfp_inl(struct nfp_cdev *pdev, int bar, int offset);
unsigned short nfp_inw(struct nfp_cdev *pdev, int bar, int offset);
void nfp_outl(struct nfp_cdev *pdev, int bar, int offset, unsigned int data);
void nfp_outw(struct nfp_cdev *pdev, int bar, int offset, unsigned short data);

/* user and device memory space access ---------------------------- */

/*
 * NB these 2 functions are not guaranteed to be re-entrant for a given device
 */
int nfp_copy_from_user_to_dev(struct nfp_cdev *cdev, int bar, int offset,
			      const char *ubuf, int len);
int nfp_copy_to_user_from_dev(struct nfp_cdev *cdev, int bar, int offset,
			      char *ubuf, int len);

int nfp_copy_from_user(char *kbuf, const char *ubuf, int len);
int nfp_copy_to_user(char *ubuf, const char *kbuf, int len);

int nfp_copy_from_dev(struct nfp_cdev *cdev, int bar, int offset,
		      char *kbuf, int len);
int nfp_copy_to_dev(struct nfp_cdev *cdev, int bar, int offset,
		    const char *kbuf, int len);

/* debug ------------------------------------------------------------ */

#define NFP_DBG1	1
#define NFP_DBGE	NFP_DBG1
#define NFP_DBG2	2
#define NFP_DBG3	3
#define NFP_DBG4	4

void nfp_log(int severity, const char *format, ...);
extern int nfp_debug;

#endif
