#ifndef _BNX2_H
#define _BNX2_H

/** @file
 *
 * Broadcom NetXtreme II series 10/100/1000 network card driver
 *
 */

FILE_LICENCE ( GPL2_OR_LATER );

#include <ipxe/mii.h>

/** bnx2 BAR size */
#define BNX2_BAR_SIZE 256

/** MISC_ID register */
#define BNX2_MISC_ID			0x00000808UL
#define BNX2_MISC_ID_BOND_ID	0x0000000fUL
#define BNX2_MISC_ID_CHIP_METAL	0x00000ff0UL
#define BNX2_MISC_ID_CHIP_REV	0x0000f000UL
#define BNX2_MISC_ID_CHIP_NUM	0xffff0000UL

/** A bnx2 network card */
struct bnx2_nic {
	/** Registers */
	void *regs;
	/** MII interface */
	struct mii_interface mii;
};

#endif /* _BNX2_H */
