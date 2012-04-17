/*
 * ipg.c: Device Driver for the IP1000 Gigabit Ethernet Adapter
 *
 * Copyright (C) 2003, 2007  IC Plus Corp
 *
 * Original Author:
 *
 *   Craig Rich
 *   Sundance Technology, Inc.
 *   www.sundanceti.com
 *   craig_rich@sundanceti.com
 *
 * Current Maintainer:
 *
 *   Sorbica Shieh.
 *   http://www.icplus.com.tw
 *   sorbica@icplus.com.tw
 *
 *   Jesse Huang
 *   http://www.icplus.com.tw
 *   jesse@icplus.com.tw
 *
 * iPXE Port Copyright (C) 2009 Thomas Miletich <thomas.miletich@gmail.com>
 *
 */

FILE_LICENCE ( GPL2_ONLY );

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <byteswap.h>
#include <errno.h>
#include <mii.h>
#include <ipxe/ethernet.h>
#include <ipxe/if_ether.h>
#include <ipxe/io.h>
#include <ipxe/iobuf.h>
#include <ipxe/malloc.h>
#include <ipxe/netdevice.h>
#include <ipxe/pci.h>
#include <ipxe/timer.h>
#include <ipxe/nvs.h>

#include "ipg.h"

/*
 * Defaults
 */
#define IPG_MAX_RXFRAME_SIZE	0x0600
#define IPG_RXFRAG_SIZE		0x0600
#define IPG_RXSUPPORT_SIZE	0x0600
/*
 * Variable record -- index by leading revision/length
 * Revision/Length(=N*4), Address1, Data1, Address2, Data2,...,AddressN,DataN
 */
static unsigned short DefaultPhyParam[] = {
	/* 11/12/03 IP1000A v1-3 rev=0x40 */
	/*--------------------------------------------------------------------------
	(0x4000|(15*4)), 31, 0x0001, 27, 0x01e0, 31, 0x0002, 22, 0x85bd, 24, 0xfff2,
				 27, 0x0c10, 28, 0x0c10, 29, 0x2c10, 31, 0x0003, 23, 0x92f6,
				 31, 0x0000, 23, 0x003d, 30, 0x00de, 20, 0x20e7,  9, 0x0700,
	  --------------------------------------------------------------------------*/
	/* 12/17/03 IP1000A v1-4 rev=0x40 */
	(0x4000 | (07 * 4)), 31, 0x0001, 27, 0x01e0, 31, 0x0002, 27,
	0xeb8e, 31,
	0x0000,
	30, 0x005e, 9, 0x0700,
	/* 01/09/04 IP1000A v1-5 rev=0x41 */
	(0x4100 | (07 * 4)), 31, 0x0001, 27, 0x01e0, 31, 0x0002, 27,
	0xeb8e, 31,
	0x0000,
	30, 0x005e, 9, 0x0700,
	0x0000
};

static int ipg_hw_init(struct net_device *dev);

static inline void *ipg_ioaddr(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	return sp->ioaddr;
}

static void ipg_write_phy_ctl(void *ioaddr, u8 data)
{
	DBGP("ipg_write_phy_ctl\n");

	ipg_w8(IPG_PC_RSVD_MASK & data, PHY_CTRL);
	udelay(IPG_PC_PHYCTRLWAIT_NS);
}

static void ipg_drive_phy_ctl_low_high(void *ioaddr, u8 data)
{
	DBGP("ipg_drive_phy_ctl_low_high\n");

	ipg_write_phy_ctl(ioaddr, IPG_PC_MGMTCLK_LO | data);
	ipg_write_phy_ctl(ioaddr, IPG_PC_MGMTCLK_HI | data);
}

static void send_three_state(void *ioaddr, u8 phyctrlpolarity)
{
	DBGP("send_three_state\n");

	phyctrlpolarity |= (IPG_PC_MGMTDATA & 0) | IPG_PC_MGMTDIR;

	ipg_drive_phy_ctl_low_high(ioaddr, phyctrlpolarity);
}

static void send_end(void *ioaddr, u8 phyctrlpolarity)
{
	DBGP("send_end\n");

	ipg_w8((IPG_PC_MGMTCLK_LO | (IPG_PC_MGMTDATA & 0) | IPG_PC_MGMTDIR
		| phyctrlpolarity) & IPG_PC_RSVD_MASK, PHY_CTRL);
}

static u16 read_phy_bit(void *ioaddr, u8 phyctrlpolarity)
{
	u16 bit_data;

	DBGP("read_phy_bit\n");

	ipg_write_phy_ctl(ioaddr, IPG_PC_MGMTCLK_LO | phyctrlpolarity);

	bit_data = ((ipg_r8(PHY_CTRL) & IPG_PC_MGMTDATA) >> 1) & 1;

	ipg_write_phy_ctl(ioaddr, IPG_PC_MGMTCLK_HI | phyctrlpolarity);

	return bit_data;
}

/*
 * Read a register from the Physical Layer device located
 * on the IPG NIC, using the IPG PHYCTRL register.
 */
static int mdio_read(struct net_device *dev, int phy_id, int phy_reg)
{
	void *ioaddr = ipg_ioaddr(dev);

	DBGP("mdio_read\n");

	/*
	 * The GMII mangement frame structure for a read is as follows:
	 *
	 * |Preamble|st|op|phyad|regad|ta|      data      |idle|
	 * |< 32 1s>|01|10|AAAAA|RRRRR|z0|DDDDDDDDDDDDDDDD|z   |
	 *
	 * <32 1s> = 32 consecutive logic 1 values
	 * A = bit of Physical Layer device address (MSB first)
	 * R = bit of register address (MSB first)
	 * z = High impedance state
	 * D = bit of read data (MSB first)
	 *
	 * Transmission order is 'Preamble' field first, bits transmitted
	 * left to right (first to last).
	 */
	struct {
		u32 field;
		unsigned int len;
	} p[] = {
		{
		GMII_PREAMBLE, 32},	/* Preamble */
		{
		GMII_ST, 2},	/* ST */
		{
		GMII_READ, 2},	/* OP */
		{
		phy_id, 5},	/* PHYAD */
		{
		phy_reg, 5},	/* REGAD */
		{
		0x0000, 2},	/* TA */
		{
		0x0000, 16},	/* DATA */
		{
		0x0000, 1}	/* IDLE */
	};
	unsigned int i, j;
	u8 polarity, data;

	polarity = ipg_r8(PHY_CTRL);
	polarity &= (IPG_PC_DUPLEX_POLARITY | IPG_PC_LINK_POLARITY);

	/* Create the Preamble, ST, OP, PHYAD, and REGAD field. */
	for (j = 0; j < 5; j++) {
		for (i = 0; i < p[j].len; i++) {
			/* For each variable length field, the MSB must be
			 * transmitted first. Rotate through the field bits,
			 * starting with the MSB, and move each bit into the
			 * the 1st (2^1) bit position (this is the bit position
			 * corresponding to the MgmtData bit of the PhyCtrl
			 * register for the IPG).
			 *
			 * Example: ST = 01;
			 *
			 *          First write a '0' to bit 1 of the PhyCtrl
			 *          register, then write a '1' to bit 1 of the
			 *          PhyCtrl register.
			 *
			 * To do this, right shift the MSB of ST by the value:
			 * [field length - 1 - #ST bits already written]
			 * then left shift this result by 1.
			 */
			data = (p[j].field >> (p[j].len - 1 - i)) << 1;
			data &= IPG_PC_MGMTDATA;
			data |= polarity | IPG_PC_MGMTDIR;

			ipg_drive_phy_ctl_low_high(ioaddr, data);
		}
	}

	send_three_state(ioaddr, polarity);

	read_phy_bit(ioaddr, polarity);

	/*
	 * For a read cycle, the bits for the next two fields (TA and
	 * DATA) are driven by the PHY (the IPG reads these bits).
	 */
	for (i = 0; i < p[6].len; i++) {
		p[6].field |=
		    (read_phy_bit(ioaddr, polarity) << (p[6].len - 1 - i));
	}

	send_three_state(ioaddr, polarity);
	send_three_state(ioaddr, polarity);
	send_three_state(ioaddr, polarity);
	send_end(ioaddr, polarity);

	/* Return the value of the DATA field. */
	return p[6].field;
}

/*
 * Write to a register from the Physical Layer device located
 * on the IPG NIC, using the IPG PHYCTRL register.
 */
static void mdio_write(struct net_device *dev, int phy_id, int phy_reg,
		       int val)
{
	void *ioaddr = ipg_ioaddr(dev);

	DBGP("mdio_write\n");
	/*
	 * The GMII mangement frame structure for a read is as follows:
	 *
	 * |Preamble|st|op|phyad|regad|ta|      data      |idle|
	 * |< 32 1s>|01|10|AAAAA|RRRRR|z0|DDDDDDDDDDDDDDDD|z   |
	 *
	 * <32 1s> = 32 consecutive logic 1 values
	 * A = bit of Physical Layer device address (MSB first)
	 * R = bit of register address (MSB first)
	 * z = High impedance state
	 * D = bit of write data (MSB first)
	 *
	 * Transmission order is 'Preamble' field first, bits transmitted
	 * left to right (first to last).
	 */
	struct {
		u32 field;
		unsigned int len;
	} p[] = {
		{
		GMII_PREAMBLE, 32},	/* Preamble */
		{
		GMII_ST, 2},	/* ST */
		{
		GMII_WRITE, 2},	/* OP */
		{
		phy_id, 5},	/* PHYAD */
		{
		phy_reg, 5},	/* REGAD */
		{
		0x0002, 2},	/* TA */
		{
		val & 0xffff, 16},	/* DATA */
		{
		0x0000, 1}	/* IDLE */
	};
	unsigned int i, j;
	u8 polarity, data;

	polarity = ipg_r8(PHY_CTRL);
	polarity &= (IPG_PC_DUPLEX_POLARITY | IPG_PC_LINK_POLARITY);

	/* Create the Preamble, ST, OP, PHYAD, and REGAD field. */
	for (j = 0; j < 7; j++) {
		for (i = 0; i < p[j].len; i++) {
			/* For each variable length field, the MSB must be
			 * transmitted first. Rotate through the field bits,
			 * starting with the MSB, and move each bit into the
			 * the 1st (2^1) bit position (this is the bit position
			 * corresponding to the MgmtData bit of the PhyCtrl
			 * register for the IPG).
			 *
			 * Example: ST = 01;
			 *
			 *          First write a '0' to bit 1 of the PhyCtrl
			 *          register, then write a '1' to bit 1 of the
			 *          PhyCtrl register.
			 *
			 * To do this, right shift the MSB of ST by the value:
			 * [field length - 1 - #ST bits already written]
			 * then left shift this result by 1.
			 */
			data = (p[j].field >> (p[j].len - 1 - i)) << 1;
			data &= IPG_PC_MGMTDATA;
			data |= polarity | IPG_PC_MGMTDIR;

			ipg_drive_phy_ctl_low_high(ioaddr, data);
		}
	}

	/* The last cycle is a tri-state, so read from the PHY. */
	for (j = 7; j < 8; j++) {
		for (i = 0; i < p[j].len; i++) {
			ipg_write_phy_ctl(ioaddr,
					  IPG_PC_MGMTCLK_LO | polarity);

			p[j].field |= ((ipg_r8(PHY_CTRL) &
					IPG_PC_MGMTDATA) >> 1) << (p[j].
								   len -
								   1 - i);

			ipg_write_phy_ctl(ioaddr,
					  IPG_PC_MGMTCLK_HI | polarity);
		}
	}
}

static void ipg_set_led_mode(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	void *ioaddr = sp->ioaddr;
	u32 mode;

	DBGP("ipg_set_led_mode\n");

	mode = ipg_r32(ASIC_CTRL);
	mode &=
	    ~(IPG_AC_LED_MODE_BIT_1 | IPG_AC_LED_MODE | IPG_AC_LED_SPEED);

	if ((sp->led_mode & 0x03) > 1)
		mode |= IPG_AC_LED_MODE_BIT_1;	/* Write Asic Control Bit 29 */

	if ((sp->led_mode & 0x01) == 1)
		mode |= IPG_AC_LED_MODE;	/* Write Asic Control Bit 14 */

	if ((sp->led_mode & 0x08) == 8)
		mode |= IPG_AC_LED_SPEED;	/* Write Asic Control Bit 27 */

	ipg_w32(mode, ASIC_CTRL);
}

static void ipg_set_phy_set(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	void *ioaddr = sp->ioaddr;
	int physet;

	DBGP("ipg_set_phy_set\n");

	physet = ipg_r8(PHY_SET);
	physet &=
	    ~(IPG_PS_MEM_LENB9B | IPG_PS_MEM_LEN9 | IPG_PS_NON_COMPDET);
	physet |= ((sp->led_mode & 0x70) >> 4);
	ipg_w8(physet, PHY_SET);
}

static int ipg_reset(struct net_device *dev, u32 resetflags)
{
	/* Assert functional resets via the IPG AsicCtrl
	 * register as specified by the 'resetflags' input
	 * parameter.
	 */
	void *ioaddr = ipg_ioaddr(dev);
	unsigned int timeout_count = 0;

	DBGP("ipg_reset\n");

	ipg_w32(ipg_r32(ASIC_CTRL) | resetflags, ASIC_CTRL);

	/* Delay added to account for problem with 10Mbps reset. */
	mdelay(IPG_AC_RESETWAIT);

	while (IPG_AC_RESET_BUSY & ipg_r32(ASIC_CTRL)) {
		mdelay(IPG_AC_RESETWAIT);
		if (++timeout_count > IPG_AC_RESET_TIMEOUT)
			return -ETIME;
	}
	/* Set LED Mode in Asic Control */
	ipg_set_led_mode(dev);

	/* Set PHYSet Register Value */
	ipg_set_phy_set(dev);
	return 0;
}

/* Find the GMII PHY address. */
static int ipg_find_phyaddr(struct net_device *dev)
{
	unsigned int phyaddr, i;

	DBGP("ipg_find_phyaddr\n");

	for (i = 0; i < 32; i++) {
		u32 status;

		/* Search for the correct PHY address among 32 possible. */
		phyaddr = (IPG_NIC_PHY_ADDRESS + i) % 32;

		/* 10/22/03 Grace change verify from GMII_PHY_STATUS to
		   GMII_PHY_ID1
		 */

		status = mdio_read(dev, phyaddr, MII_BMSR);

		if ((status != 0xFFFF) && (status != 0))
			return phyaddr;
	}

	return 0x1f;
}

/*
 * Configure IPG based on result of IEEE 802.3 PHY
 * auto-negotiation.
 */
static int ipg_config_autoneg(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	void *ioaddr = sp->ioaddr;
	unsigned int txflowcontrol;
	unsigned int rxflowcontrol;
	unsigned int fullduplex;
	u32 mac_ctrl_val;
	u8 phyctrl;

	DBGP("ipg_config_autoneg\n");

	phyctrl = ipg_r8(PHY_CTRL);
	mac_ctrl_val = ipg_r32(MAC_CTRL);

	/* Set flags for use in resolving auto-negotation, assuming
	 * non-1000Mbps, half duplex, no flow control.
	 */
	fullduplex = 0;
	txflowcontrol = 0;
	rxflowcontrol = 0;

	/* To accomodate a problem in 10Mbps operation,
	 * set a global flag if PHY running in 10Mbps mode.
	 */
	sp->tenmbpsmode = 0;

	DBG2("%s: Link speed = ", dev->name);

	/* Determine actual speed of operation. */
	switch (phyctrl & IPG_PC_LINK_SPEED) {
	case IPG_PC_LINK_SPEED_10MBPS:
		DBG2("10Mbps.\n");
		DBG2("%s: 10Mbps operational mode enabled.\n", dev->name);
		sp->tenmbpsmode = 1;
		break;
	case IPG_PC_LINK_SPEED_100MBPS:
		DBG2("100Mbps.\n");
		break;
	case IPG_PC_LINK_SPEED_1000MBPS:
		DBG2("1000Mbps.\n");
		break;
	default:
		DBG2("undefined!\n");
		return 0;
	}

	if (phyctrl & IPG_PC_DUPLEX_STATUS) {
		fullduplex = 1;
		txflowcontrol = 1;
		rxflowcontrol = 1;
	}

	/* Configure full duplex, and flow control. */
	if (fullduplex == 1) {
		/* Configure IPG for full duplex operation. */
		DBG2("%s: setting full duplex, ", dev->name);

		mac_ctrl_val |= IPG_MC_DUPLEX_SELECT_FD;

		if (txflowcontrol == 1) {
			DBG2("TX flow control");
			mac_ctrl_val |= IPG_MC_TX_FLOW_CONTROL_ENABLE;
		} else {
			DBG2("no TX flow control");
			mac_ctrl_val &= ~IPG_MC_TX_FLOW_CONTROL_ENABLE;
		}

		if (rxflowcontrol == 1) {
			DBG2(", RX flow control.");
			mac_ctrl_val |= IPG_MC_RX_FLOW_CONTROL_ENABLE;
		} else {
			DBG2(", no RX flow control.");
			mac_ctrl_val &= ~IPG_MC_RX_FLOW_CONTROL_ENABLE;
		}

		DBG2("\n");
	} else {
		/* Configure IPG for half duplex operation. */
		DBG2("%s: setting half duplex, "
		    "no TX flow control, no RX flow control.\n",
		    dev->name);

		mac_ctrl_val &= ~IPG_MC_DUPLEX_SELECT_FD &
		    ~IPG_MC_TX_FLOW_CONTROL_ENABLE &
		    ~IPG_MC_RX_FLOW_CONTROL_ENABLE;
	}
	ipg_w32(mac_ctrl_val, MAC_CTRL);
	return 0;
}

/* Determine and configure multicast operation and set
 * receive mode for IPG.
 */
static void ipg_nic_set_multicast_list(struct net_device *dev)
{
	void *ioaddr = ipg_ioaddr(dev);
	u8 receivemode;

	DBGP("ipg_nic_set_multicast_list\n");

	receivemode = IPG_RM_RECEIVEUNICAST | IPG_RM_RECEIVEBROADCAST |
		      IPG_RM_RECEIVEMULTICAST;

	/* Write empty hashtable, to the 4, 16 bit
	 * HASHTABLE IPG registers.
	 */
	ipg_w32(0x00, HASHTABLE_0);
	ipg_w32(0x00, HASHTABLE_1);

	ipg_w8(IPG_RM_RSVD_MASK & receivemode, RECEIVE_MODE);

	DBG2("ReceiveMode = %x\n", ipg_r8(RECEIVE_MODE));
}

static int ipg_io_config(struct net_device *dev)
{
	void *ioaddr = ipg_ioaddr(dev);
	u32 origmacctrl;
	u32 restoremacctrl;

	DBGP("ipg_io_config\n");

	origmacctrl = ipg_r32(MAC_CTRL);

	restoremacctrl = origmacctrl | IPG_MC_STATISTICS_ENABLE;

	/* Determine if transmitter and/or receiver are
	 * enabled so we may restore MACCTRL correctly.
	 */
	if (origmacctrl & IPG_MC_TX_ENABLED)
		restoremacctrl |= IPG_MC_TX_ENABLE;

	if (origmacctrl & IPG_MC_RX_ENABLED)
		restoremacctrl |= IPG_MC_RX_ENABLE;

	/* Transmitter and receiver must be disabled before setting
	 * IFSSelect.
	 */
	ipg_w32((origmacctrl & (IPG_MC_RX_DISABLE | IPG_MC_TX_DISABLE)) &
		IPG_MC_RSVD_MASK, MAC_CTRL);

	/* Now that transmitter and receiver are disabled, write
	 * to IFSSelect.
	 */
	ipg_w32((origmacctrl & IPG_MC_IFS_96BIT) & IPG_MC_RSVD_MASK,
		MAC_CTRL);

	/* Set RECEIVEMODE register. */
	ipg_nic_set_multicast_list(dev);

	ipg_w16(IPG_MAX_RXFRAME_SIZE, MAX_FRAME_SIZE);

	ipg_w8(IPG_RXDMAPOLLPERIOD_VALUE, RX_DMA_POLL_PERIOD);
	ipg_w8(IPG_RXDMAURGENTTHRESH_VALUE, RX_DMA_URGENT_THRESH);
	ipg_w8(IPG_RXDMABURSTTHRESH_VALUE, RX_DMA_BURST_THRESH);
	ipg_w8(IPG_TXDMAPOLLPERIOD_VALUE, TX_DMA_POLL_PERIOD);
	ipg_w8(IPG_TXDMAURGENTTHRESH_VALUE, TX_DMA_URGENT_THRESH);
	ipg_w8(IPG_TXDMABURSTTHRESH_VALUE, TX_DMA_BURST_THRESH);
	ipg_w16((IPG_IE_HOST_ERROR | IPG_IE_TX_DMA_COMPLETE |
		 IPG_IE_TX_COMPLETE | IPG_IE_INT_REQUESTED |
		 IPG_IE_UPDATE_STATS | IPG_IE_LINK_EVENT |
		 IPG_IE_RX_DMA_COMPLETE | IPG_IE_RX_DMA_PRIORITY),
		INT_ENABLE);
	ipg_w16(IPG_FLOWONTHRESH_VALUE, FLOW_ON_THRESH);
	ipg_w16(IPG_FLOWOFFTHRESH_VALUE, FLOW_OFF_THRESH);

	/* IPG multi-frag frame bug workaround.
	 * Per silicon revision B3 eratta.
	 */
	ipg_w16(ipg_r16(DEBUG_CTRL) | 0x0200, DEBUG_CTRL);

	/* IPG TX poll now bug workaround.
	 * Per silicon revision B3 eratta.
	 */
	ipg_w16(ipg_r16(DEBUG_CTRL) | 0x0010, DEBUG_CTRL);

	/* IPG RX poll now bug workaround.
	 * Per silicon revision B3 eratta.
	 */
	ipg_w16(ipg_r16(DEBUG_CTRL) | 0x0020, DEBUG_CTRL);

	/* Now restore MACCTRL to original setting. */
	ipg_w32(IPG_MC_RSVD_MASK & restoremacctrl, MAC_CTRL);

	/* Disable unused RMON statistics. */
	ipg_w32(IPG_RZ_ALL, RMON_STATISTICS_MASK);

	/* Disable unused MIB statistics. */
	ipg_w32(IPG_SM_MACCONTROLFRAMESXMTD | IPG_SM_MACCONTROLFRAMESRCVD |
		IPG_SM_BCSTOCTETXMTOK_BCSTFRAMESXMTDOK |
		IPG_SM_TXJUMBOFRAMES |
		IPG_SM_MCSTOCTETXMTOK_MCSTFRAMESXMTDOK |
		IPG_SM_RXJUMBOFRAMES |
		IPG_SM_BCSTOCTETRCVDOK_BCSTFRAMESRCVDOK |
		IPG_SM_UDPCHECKSUMERRORS | IPG_SM_TCPCHECKSUMERRORS |
		IPG_SM_IPCHECKSUMERRORS, STATISTICS_MASK);

	return 0;
}

/*
 * Create a receive buffer within system memory and update
 * NIC private structure appropriately.
 */
static int ipg_get_rxbuff(struct net_device *dev, int entry)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	struct ipg_rx *rxfd = sp->rxd + entry;
	struct io_buffer *iob;
	u64 rxfragsize;

	DBGP("ipg_get_rxbuff\n");

	iob = alloc_iob(IPG_RXSUPPORT_SIZE);

	/* Save the address of the io_buffer structure. */
	sp->rx_buff[entry] = iob;
	if (!iob) {
		DBG("ipg_get_rxbuff: out of memory\n");
		return -ENOMEM;
	}

	rxfd->frag_info = cpu_to_le64(virt_to_bus(iob->data));

	/* Set the RFD fragment length. */
	rxfragsize = IPG_RXFRAG_SIZE;
	rxfd->frag_info |=
	    cpu_to_le64((rxfragsize << 48) & IPG_RFI_FRAGLEN);

	return 0;
}

static int init_rfdlist(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	void *ioaddr = sp->ioaddr;
	unsigned int i;

	DBGP("ipg_init_rfdlist\n");

	for (i = 0; i < IPG_RFDLIST_LENGTH; i++) {
		struct ipg_rx *rxfd = sp->rxd + i;

		sp->rx_buff[i] = NULL;

		/* Clear out the RFS field. */
		rxfd->rfs = 0x0000000000000000;

		if (ipg_get_rxbuff(dev, i) < 0) {
			/*
			 * A receive buffer was not ready, break the
			 * RFD list here.
			 */
			DBG("Cannot allocate Rx buffer.\n");
			/* tell the NIC not to use this rxfd */
			rxfd->rfs = IPG_RFS_RFDDONE;

			/* Just in case we cannot allocate a single RFD.
			 * Should not occur.
			 */
			if (i == 0) {
				DBG("%s: No memory available"
				    " for RFD list.\n", dev->name);
				return -ENOMEM;
			}
		}

		rxfd->next_desc = cpu_to_le64(sp->rxd_map +
					      sizeof(struct ipg_rx) * (i + 1));
	}
	sp->rxd[i - 1].next_desc = cpu_to_le64(sp->rxd_map);

	sp->rx_current = 0;
	sp->rx_dirty = 0;

	/* Write the location of the RFDList to the IPG. */
	ipg_w32(sp->rxd_map, RFD_LIST_PTR_0);
	ipg_w32(0x00000000, RFD_LIST_PTR_1);

	return 0;
}

static void init_tfdlist(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	void *ioaddr = sp->ioaddr;
	unsigned int i;

	DBGP("ipg_init_tfdlist\n");

	for (i = 0; i < IPG_TFDLIST_LENGTH; i++) {
		struct ipg_tx *txfd = sp->txd + i;

		txfd->tfc = cpu_to_le64(IPG_TFC_TFDDONE);

		sp->tx_buff[i] = NULL;

		txfd->next_desc = cpu_to_le64(sp->txd_map +
					      sizeof(struct ipg_tx) * (i + 1));
	}
	sp->txd[i - 1].next_desc = cpu_to_le64(sp->txd_map);

	sp->tx_current = 0;
	sp->tx_dirty = 0;

	/* Write the location of the TFDList to the IPG. */
	DBG2("Starting TFDListPtr = %8x\n", sp->txd_map);
	ipg_w32(sp->txd_map, TFD_LIST_PTR_0);
	ipg_w32(0x00000000, TFD_LIST_PTR_1);
}

/*
 * Report all transmit buffers which have already been transfered
 * via DMA to the iPXE core with netdev_tx_complete.
 */
static void ipg_nic_txfree(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	unsigned int released, pending, dirty;

	DBGP("ipg_nic_txfree\n");

	pending = sp->tx_current - sp->tx_dirty;
	dirty = sp->tx_dirty % IPG_TFDLIST_LENGTH;

	for (released = 0; released < pending; released++) {
		struct io_buffer *iob = sp->tx_buff[dirty];
		struct ipg_tx *txfd = sp->txd + dirty;

		/* Look at each TFD's TFC field beginning
		 * at the last freed TFD up to the current TFD.
		 * If the TFDDone bit is set, report the associated
		 * buffer.
		 */
		if (!(txfd->tfc & cpu_to_le64(IPG_TFC_TFDDONE)))
			break;

		if (iob) {
			/* report sucessful transmit. */
			netdev_tx_complete(dev, iob);

			sp->tx_buff[dirty] = NULL;
		}
		dirty = (dirty + 1) % IPG_TFDLIST_LENGTH;
	}

	sp->tx_dirty += released;
}

/*
 * For TxComplete interrupts, free all transmit
 * buffers which have already been transfered via DMA
 * to the IPG.
 */
static void ipg_nic_txcleanup(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	void *ioaddr = sp->ioaddr;
	unsigned int i;

	DBGP("ipg_nic_txcleanup\n");

	for (i = 0; i < IPG_TFDLIST_LENGTH; i++) {
		/* Reading the TXSTATUS register clears the
		 * TX_COMPLETE interrupt.
		 */
		u32 txstatusdword = ipg_r32(TX_STATUS);

		/* Check for Transmit errors. Error bits only valid if
		 * TX_COMPLETE bit in the TXSTATUS register is a 1.
		 */
		if (!(txstatusdword & IPG_TS_TX_COMPLETE))
			break;

		/* Transmit error */
		if (txstatusdword & IPG_TS_TX_ERROR) {
			DBG("Transmit error.\n");
		}

		/* Late collision, re-enable transmitter. */
		if (txstatusdword & IPG_TS_LATE_COLLISION) {
			DBG("Late collision on transmit.\n");
			ipg_w32((ipg_r32(MAC_CTRL) | IPG_MC_TX_ENABLE) &
				IPG_MC_RSVD_MASK, MAC_CTRL);
		}

		/* Maximum collisions, re-enable transmitter. */
		if (txstatusdword & IPG_TS_TX_MAX_COLL) {
			DBG("Maximum collisions on transmit.\n");
			ipg_w32((ipg_r32(MAC_CTRL) | IPG_MC_TX_ENABLE) &
				IPG_MC_RSVD_MASK, MAC_CTRL);
		}

		/* Transmit underrun, reset and re-enable
		 * transmitter.
		 */
		if (txstatusdword & IPG_TS_TX_UNDERRUN) {
			DBG("Transmitter underrun.\n");
			ipg_reset(dev, IPG_AC_TX_RESET | IPG_AC_DMA |
				  IPG_AC_NETWORK | IPG_AC_FIFO);

			/* Re-configure after DMA reset. */
			if (ipg_io_config(dev) < 0) {
				DBG("%s: Error during re-configuration.\n",
				    dev->name);
			}
			init_tfdlist(dev);

			ipg_w32((ipg_r32(MAC_CTRL) | IPG_MC_TX_ENABLE) &
				IPG_MC_RSVD_MASK, MAC_CTRL);
		}
	}

	ipg_nic_txfree(dev);
}

/* Restore used receive buffers. */
static int ipg_nic_rxrestore(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	const unsigned int curr = sp->rx_current;
	unsigned int dirty = sp->rx_dirty;

	DBGP("ipg_nic_rxrestore\n");

	for (dirty = sp->rx_dirty; curr != dirty; dirty++) {
		unsigned int entry = dirty % IPG_RFDLIST_LENGTH;

		/* allocate new rx buffer */
		if (ipg_get_rxbuff(dev, entry) < 0) {
			DBG("Cannot allocate new Rx buffer.\n");

			break;
		}

		/* Reset the RFS field. */
		sp->rxd[entry].rfs = 0x0000000000000000;
	}
	sp->rx_dirty = dirty;

	return 0;
}

#define __RFS_MASK \
	cpu_to_le64(IPG_RFS_RFDDONE | IPG_RFS_FRAMESTART | IPG_RFS_FRAMEEND)

/* Transfer received Ethernet frames to higher network layers. */
static int ipg_nic_rx(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	unsigned int curr = sp->rx_current;
	struct ipg_rx *rxfd;
	uint64_t rfs;
	unsigned int i;

	DBGP("ipg_nic_rx\n");

	for (i = 0; i < IPG_RFDLIST_LENGTH; i++, curr++) {
		unsigned int entry = curr % IPG_RFDLIST_LENGTH;
		struct io_buffer *iob = sp->rx_buff[entry];
		unsigned int framelen;

		rxfd = sp->rxd + entry;
		rfs = rxfd->rfs;

		if (((rfs & __RFS_MASK) != __RFS_MASK) || !iob)
			break;

		/* Get received frame length. */
		framelen = le64_to_cpu(rfs) & IPG_RFS_RXFRAMELEN;

		/* Check for jumbo frame arrival with too small
		 * RXFRAG_SIZE.
		 */
		if (framelen > IPG_RXFRAG_SIZE) {
			DBG("RFS FrameLen > allocated fragment size.\n");

			framelen = IPG_RXFRAG_SIZE;
		}

		if ((le64_to_cpu(rfs & (IPG_RFS_RXFIFOOVERRUN |
					IPG_RFS_RXRUNTFRAME |
					IPG_RFS_RXALIGNMENTERROR |
					IPG_RFS_RXFCSERROR |
					IPG_RFS_RXOVERSIZEDFRAME |
					IPG_RFS_RXLENGTHERROR))))
		{
			/* rx error occured, dropping packet */

			DBG("Rx error, RFS = %16lx\n",
				      (unsigned long int)rfs);

			if (le64_to_cpu(rfs) & IPG_RFS_RXFIFOOVERRUN)
				DBG("RX FIFO overrun occured.\n");

			if (le64_to_cpu(rfs) & IPG_RFS_RXRUNTFRAME)
				DBG("RX runt occured.\n");

			/* report rx error */
			DBG("RX error\n");
			netdev_rx_err(dev, iob, -EINVAL);
		} else {
			/* we received a good packet */

			/* Adjust the new buffer length to accomodate the size
			 * of the received frame.
			 */
			iob_put(iob, framelen);

			/* Hand off frame for higher layer processing.
			 * The function netdev_rx() releases the io_buffer
			 * when processing completes.
			 */
			netdev_rx(dev, iob);
		}
		/* Assure RX buffer is not reused by IPG. */
		sp->rx_buff[entry] = NULL;
	}

	while ((le64_to_cpu(rfs) & IPG_RFS_RFDDONE) &&
	       !((le64_to_cpu(rfs) & IPG_RFS_FRAMESTART) &&
		 (le64_to_cpu(rfs) & IPG_RFS_FRAMEEND))) {

		unsigned int entry = curr++ % IPG_RFDLIST_LENGTH;

		rxfd = sp->rxd + entry;
		rfs = rxfd->rfs;

		DBG("Frame requires multiple RFDs.\n");

		/* An unexpected event, additional code needed to handle
		 * properly. So for the time being, just disregard the
		 * frame.
		 */

		/* Report error to higher level */
		if (sp->rx_buff[entry])
			netdev_rx_err(dev, sp->rx_buff[entry], -EINVAL);

		/* Assure RX buffer is not reused */
		sp->rx_buff[entry] = NULL;
	}

	sp->rx_current = curr;

	ipg_nic_rxrestore(dev);

	return 0;
}

static void ipg_rx_clear(struct ipg_nic_private *sp)
{
	unsigned int i;

	DBGP("ipg_rx_clear\n");

	for (i = 0; i < IPG_RFDLIST_LENGTH; i++) {
		free_iob(sp->rx_buff[i]);
		sp->rx_buff[i] = NULL;
	}
}

static void ipg_tx_clear(struct ipg_nic_private *sp)
{
	unsigned int i;

	DBGP("ipg_tx_clear\n");

	for (i = 0; i < IPG_TFDLIST_LENGTH; i++)
		sp->tx_buff[i] = NULL;
}

static int ipg_nic_open(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	void *ioaddr = sp->ioaddr;
	int rc, i;
	unsigned short *mac_word;

	DBGP("ipg_nic_open\n");

	mac_word = (unsigned short *)dev->ll_addr;
	/* send MAC address to NIC */
	for (i = 0; i < 3; i++)
		ipg_w16(mac_word[i], STATION_ADDRESS_0 + i * 2);

	/* disable interrupts */
	ipg_w16(0x0000, INT_ENABLE);

	rc = -ENOMEM;

	sp->rxd = malloc_dma(IPG_RX_RING_BYTES, IPG_RX_ALIGN);
	sp->rxd_map = virt_to_bus(sp->rxd);
	if (!sp->rxd)
		goto out;

	sp->txd = malloc_dma(IPG_TX_RING_BYTES, IPG_TX_ALIGN);
	sp->txd_map = virt_to_bus(sp->txd);
	if (!sp->txd)
		goto error;

	rc = init_rfdlist(dev);
	if (rc < 0) {
		DBG("%s: Error during configuration.\n", dev->name);
		goto error;
	}

	init_tfdlist(dev);

	rc = ipg_io_config(dev);
	if (rc < 0) {
		DBG("%s: Error during configuration.\n", dev->name);
		goto error;
	}

	/* Resolve autonegotiation. */
	if (ipg_config_autoneg(dev) < 0)
		DBG("%s: Auto-negotiation error.\n", dev->name);

	/* Enable transmit and receive operation of the IPG. */
	ipg_w32((ipg_r32(MAC_CTRL) | IPG_MC_RX_ENABLE | IPG_MC_TX_ENABLE) &
		IPG_MC_RSVD_MASK, MAC_CTRL);

	/* enable interrupt notifications */
	sp->interrupts = 1;
	ipg_w16(IPG_IE_TX_DMA_COMPLETE | IPG_IE_RX_DMA_COMPLETE |
		IPG_IE_HOST_ERROR | IPG_IE_INT_REQUESTED |
		IPG_IE_TX_COMPLETE | IPG_IE_LINK_EVENT |
		IPG_IE_UPDATE_STATS, INT_ENABLE);

      out:
	return rc;

      error:
	ipg_tx_clear(sp);
	ipg_rx_clear(sp);
	free_dma(sp->txd, IPG_TX_RING_BYTES);
	free_dma(sp->rxd, IPG_RX_RING_BYTES);
	goto out;
}

static void ipg_nic_stop(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	void *ioaddr = sp->ioaddr;

	DBGP("ipg_nic_stop\n");

	do {
		(void) ipg_r16(INT_STATUS_ACK);

		ipg_reset(dev,
			  IPG_AC_GLOBAL_RESET | IPG_AC_HOST | IPG_AC_DMA);
	} while (ipg_r16(INT_ENABLE) & IPG_IE_RSVD_MASK);

	ipg_rx_clear(sp);

	ipg_tx_clear(sp);

	/* free TX and RX ring */
	free_dma(sp->txd, IPG_TX_RING_BYTES);
	free_dma(sp->rxd, IPG_RX_RING_BYTES);
}

static void ipg_nic_poll(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	void *ioaddr = sp->ioaddr;
	unsigned short status;

	DBGP("ipg_nic_poll\n");

	/* Get interrupt source information, and acknowledge
	 * some (i.e. TxDMAComplete, RxDMAComplete, RxEarly,
	 * IntRequested, MacControlFrame, LinkEvent) interrupts
	 * if issued. Also, all IPG interrupts are disabled by
	 * reading IntStatusAck.
	 */
	status = ipg_r16(INT_STATUS_ACK);

	/* If LinkEvent interrupt, resolve autonegotiation. */
	if (status & IPG_IS_LINK_EVENT) {
		if (ipg_config_autoneg(dev) < 0)
			DBG("%s: Auto-negotiation error.\n", dev->name);
	}

	if (!((status & IPG_IS_RX_DMA_PRIORITY) ||
		    (status & IPG_IS_RFD_LIST_END) ||
		    (status & IPG_IS_RX_DMA_COMPLETE) ||
		    (status & IPG_IS_INT_REQUESTED)) ||
		    (status & IPG_IS_TX_COMPLETE)) {
		/* nothing to do here... */
		goto out;
	}

	ipg_nic_rx(dev);
	ipg_nic_txcleanup(dev);

    out:
	/* only re-enable interrupts, if not disabled by ipg_nic_irq */
	if(sp->interrupts) {
		ipg_w16(IPG_INTERRUPTS, INT_ENABLE);
	}
}

static void ipg_nic_irq(struct net_device *dev, int enabled)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	void *ioaddr = sp->ioaddr;

	DBGP("ipg_nic_irq\n");

	if(enabled) {
		sp->interrupts = 1;
		ipg_w16(IPG_INTERRUPTS, INT_ENABLE);
	} else {
		/* disable interrupts */
		sp->interrupts = 0;
		ipg_w16(0x0000, INT_ENABLE);
	}

	return;
}

static int ipg_nic_hard_start_xmit(struct net_device *dev,
				   struct io_buffer *iob)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	void *ioaddr = sp->ioaddr;
	unsigned int entry = sp->tx_current % IPG_TFDLIST_LENGTH;
	struct ipg_tx *txfd;

	DBGP("ipg_nic_hard_start_xmit\n");

	/* get status, acknowledge interrupts */
	(void) ipg_r16(INT_STATUS_ACK);

	txfd = sp->txd + entry;

	sp->tx_buff[entry] = iob;

	/* Clear all TFC fields, except TFDDONE. */
	txfd->tfc = cpu_to_le64(IPG_TFC_TFDDONE);

	/* Specify the TFC field within the TFD. */
	txfd->tfc |= cpu_to_le64(IPG_TFC_WORDALIGNDISABLED |
				 (IPG_TFC_FRAMEID & sp->tx_current) |
				 (IPG_TFC_FRAGCOUNT & (1 << 24)));
	/*
	 * 16--17 (WordAlign) <- 3 (disable),
	 * 0--15 (FrameId) <- sp->tx_current,
	 * 24--27 (FragCount) <- 1
	 */

	/* The fragment start location within system memory is defined
	 * by the io_buffer structure's data field. The physical address
	 * of this location within the system's virtual memory space
	 * is determined using the virt_to_bus function.
	 */
	txfd->frag_info = cpu_to_le64(virt_to_bus(iob->data));

	/* The length of the fragment within system memory is defined by
	 * the iob_len().
	 */
	txfd->frag_info |= cpu_to_le64(IPG_TFI_FRAGLEN &
				       ((u64) (iob_len(iob) & 0xffff) << 48));

	/* Clear the TFDDone bit last to indicate the TFD is ready
	 * for transfer to the IPG.
	 */
	txfd->tfc &= cpu_to_le64(~IPG_TFC_TFDDONE);

	sp->tx_current++;

	ipg_w32(IPG_DC_TX_DMA_POLL_NOW, DMA_CTRL);

	return 0;
}

static void ipg_set_phy_default_param(unsigned char rev,
				      struct net_device *dev,
				      int phy_address)
{
	unsigned short length;
	unsigned char revision;
	unsigned short *phy_param;
	unsigned short address, value;

	DBGP("ipg_set_phy_default_param\n");

	phy_param = &DefaultPhyParam[0];
	length = *phy_param & 0x00FF;
	revision = (unsigned char) ((*phy_param) >> 8);
	phy_param++;
	while (length != 0) {
		if (rev == revision) {
			while (length > 1) {
				address = *phy_param;
				value = *(phy_param + 1);
				phy_param += 2;
				mdio_write(dev, phy_address, address,
					   value);
				length -= 4;
			}
			break;
		} else {
			phy_param += length / 2;
			length = *phy_param & 0x00FF;
			revision = (unsigned char) ((*phy_param) >> 8);
			phy_param++;
		}
	}
}

static int ipg_read_eeprom_word(struct ipg_nic_private *sp, int eep_addr)
{
	void *ioaddr = sp->ioaddr;
	unsigned int i;
	int ret = 0;
	u16 value;

	DBGP("ipg_read_eeprom_word\n");

	value = IPG_EC_EEPROM_READOPCODE | (eep_addr & 0xff);
	ipg_w16(value, EEPROM_CTRL);

	for (i = 0; i < 1000; i++) {
		u16 data;

		mdelay(10);
		data = ipg_r16(EEPROM_CTRL);
		if (!(data & IPG_EC_EEPROM_BUSY)) {
			ret = ipg_r16(EEPROM_DATA);
			break;
		}
	}
	return ret;
}

static int ipg_read_eeprom(struct nvs_device *nvs,
                           unsigned int address,
                           void *data, size_t len)
{
	struct ipg_nic_private *sp;
	uint16_t *dest;

	DBGP("ipg_read_eeprom\n");

	sp = container_of(nvs, struct ipg_nic_private, nvs);
	dest = (uint16_t *) data;

	/* len can only be != 2 if there's something
	 * wrong with the nvs setup
	 */
	assert(len == 2);

	*dest = ipg_read_eeprom_word(sp, address);

	return 0;
}

static int ipg_write_eeprom(struct nvs_device *nvs __unused,
                            unsigned int address __unused,
                            const void *data __unused, size_t len __unused)
{
	DBGP("ipg_write_eeprom\n");
	return -ENOTSUP;
}

static void ipg_init_mii(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	int phyaddr;

	DBGP("ipg_init_mii\n");

	phyaddr = ipg_find_phyaddr(dev);

	if (phyaddr != 0x1f) {
		u16 mii_phyctrl, mii_1000cr;
		u8 revisionid = 0;

		mii_1000cr = mdio_read(dev, phyaddr, MII_CTRL1000);
		mii_1000cr |= ADVERTISE_1000FULL | ADVERTISE_1000HALF |
		    GMII_PHY_1000BASETCONTROL_PreferMaster;
		mdio_write(dev, phyaddr, MII_CTRL1000, mii_1000cr);

		mii_phyctrl = mdio_read(dev, phyaddr, MII_BMCR);

		/* Set default phyparam */
		pci_read_config_byte(sp->pdev, PCI_REVISION_ID,
				     &revisionid);
		ipg_set_phy_default_param(revisionid, dev, phyaddr);

		/* Reset PHY */
		mii_phyctrl |= BMCR_RESET | BMCR_ANRESTART;
		mdio_write(dev, phyaddr, MII_BMCR, mii_phyctrl);

	}
}

static int ipg_hw_init(struct net_device *dev)
{
	struct ipg_nic_private *sp = netdev_priv(dev);
	int rc;

	DBGP("ipg_hw_init\n");

	/* Read/Write and Reset EEPROM Value */
	/* Read LED Mode Configuration from EEPROM */
	nvs_read(&sp->nvs, IPG_EEPROM_LED_MODE, &sp->led_mode, 2);

	/* Reset all functions within the IPG. Do not assert
	 * RST_OUT as not compatible with some PHYs.
	 */
	rc = ipg_reset(dev, IPG_RESET_MASK);
	if (rc < 0)
		goto out;

	ipg_init_mii(dev);

	/* Read MAC Address from EEPROM */
	nvs_read(&sp->nvs, IPG_EEPROM_STATION_ADDR0, dev->ll_addr, ETH_ALEN);

      out:
	return rc;
}

static void ipg_remove(struct pci_device *pdev)
{
	struct net_device *dev = pci_get_drvdata(pdev);

	DBGP("ipg_remove\n");

	ipg_reset(dev, 0);

	/* Un-register Ethernet device. */
	unregister_netdev(dev);
	netdev_nullify(dev);
	netdev_put(dev);
}

static struct net_device_operations ipg_operations = {
	.open = ipg_nic_open,
	.transmit = ipg_nic_hard_start_xmit,
	.poll = ipg_nic_poll,
	.close = ipg_nic_stop,
	.irq = ipg_nic_irq,
};

static int ipg_probe(struct pci_device *pdev)
{
	struct ipg_nic_private *sp;
	struct net_device *dev;
	void *ioaddr;
	int rc;

	DBGP("ipg_probe\n");

	adjust_pci_device(pdev);

	/*
	 * Initialize net device.
	 */
	dev = alloc_etherdev(sizeof(struct ipg_nic_private));
	if (!dev) {
		rc = -ENOMEM;
		goto out;
	}

	sp = netdev_priv(dev);

	/* passing the driver functions to iPXE */
	netdev_init(dev, &ipg_operations);

	ioaddr = (void *)pdev->ioaddr;
	if (!ioaddr) {
		rc = -EIO;
		goto error;
	}

	/* Save the pointer to the PCI device information. */
	sp->ioaddr = ioaddr;
	sp->pdev = pdev;

	pci_set_drvdata(pdev, dev);
	dev->dev = &pdev->dev;

	/* setup nvs_device */
	sp->nvs.word_len_log2 = 1;
	sp->nvs.block_size = 1;
	sp->nvs.size = IPG_EEPROM_SIZE;
	sp->nvs.read = ipg_read_eeprom;
	sp->nvs.write = ipg_write_eeprom;

	rc = ipg_hw_init(dev);
	if (rc < 0)
		goto error;

	netdev_link_up(dev);

	rc = register_netdev(dev);
	if (rc < 0)
		goto error;

	DBG2("Ethernet device registered as: %s\n", dev->name);
      out:
	return rc;

      error:
	netdev_put(dev);
	goto out;
}

static struct pci_device_id ipg_pci_tbl[] = {
	PCI_ROM(0x13f0, 0x1023, "ip1000", "ip1000", 0),
	PCI_ROM(0x13f0, 0x2021, "st2021", "st2021", 0),
	PCI_ROM(0x13f0, 0x1021, "tc9020/9021", "tc9020/tx9021", 0),
	PCI_ROM(0x1186, 0x9021, "tc9020/9021", "tc9020/tx9021", 0),
	PCI_ROM(0x1186, 0x4000, "dlink", "dlink", 0),
	PCI_ROM(0x1186, 0x4020, "dlink_ip1000a", "dlink_ip1000a", 0),
};

struct pci_driver ipg_pci_driver __pci_driver = {
	.ids = ipg_pci_tbl,
	.id_count = (sizeof(ipg_pci_tbl) / sizeof(ipg_pci_tbl[0])),
	.probe = ipg_probe,
	.remove = ipg_remove,
};

/*
 * Local variables:
 *  c-basic-offset: 8
 *  c-indent-level: 8
 *  tab-width: 8
 * End:
 */
