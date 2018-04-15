/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2007-2010 coresystems GmbH
 * Copyright (C) 2011 The ChromiumOS Authors.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
 
#include <cbfs.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <console/console.h>
#include <cpu/intel/haswell/haswell.h>
#include <northbridge/intel/haswell/haswell.h>
#include <northbridge/intel/haswell/raminit.h>
#include <southbridge/intel/lynxpoint/pch.h>
#include <southbridge/intel/common/gpio.h>
#include <superio/nuvoton/common/nuvoton.h>
#include <superio/nuvoton/nct6791d/nct6791d.h>

const struct rcba_config_instruction rcba_config[] = {
	/* Device interrupt route registers */
	RCBA_SET_REG_32(D31IR, DIR_ROUTE(PIRQG, PIRQC, PIRQB, PIRQA)),/* LPC */
	RCBA_SET_REG_32(D29IR, DIR_ROUTE(PIRQD, PIRQD, PIRQD, PIRQD)),/* EHCI */
	RCBA_SET_REG_32(D28IR, DIR_ROUTE(PIRQA, PIRQB, PIRQC, PIRQD)),/* PCIE */
	RCBA_SET_REG_32(D27IR, DIR_ROUTE(PIRQG, PIRQG, PIRQG, PIRQG)),/* HDA */
	RCBA_SET_REG_32(D22IR, DIR_ROUTE(PIRQA, PIRQA, PIRQA, PIRQA)),/* ME */
	RCBA_SET_REG_32(D21IR, DIR_ROUTE(PIRQE, PIRQF, PIRQF, PIRQF)),/* SIO */
	RCBA_SET_REG_32(D20IR, DIR_ROUTE(PIRQC, PIRQC, PIRQC, PIRQC)),/* XHCI */
	RCBA_SET_REG_32(D23IR, DIR_ROUTE(PIRQH, PIRQH, PIRQH, PIRQH)),/* SDIO */
	RCBA_END_CONFIG,
};

void mainboard_romstage_entry(unsigned long bist)
{
	struct pei_data pei_data = {
		.pei_version = PEI_VERSION,
		.mchbar = (uintptr_t)DEFAULT_MCHBAR,
		.dmibar = (uintptr_t)DEFAULT_DMIBAR,
		.epbar = DEFAULT_EPBAR,
		.pciexbar = DEFAULT_PCIEXBAR,
		.smbusbar = SMBUS_IO_BASE,
		.wdbbar = 0x4000000,
		.wdbsize = 0x1000,
		.hpet_address = HPET_ADDR,
		.rcba = (uintptr_t)DEFAULT_RCBA,
		.pmbase = DEFAULT_PMBASE,
		.gpiobase = DEFAULT_GPIOBASE,
		.temp_mmio_base = 0xfed08000,
		.system_type = 1, // 0 Mobile, 1 Desktop/Server
		.tseg_size = CONFIG_SMM_TSEG_SIZE,
		.spd_addresses = { 0x50, 0x51, 0x52, 0x53 },
		.ec_present = 0,
		// 0 = leave channel enabled
		// 1 = disable dimm 0 on channel
		// 2 = disable dimm 1 on channel
		// 3 = disable dimm 0+1 on channel
		.dimm_channel0_disabled = 0,
		.dimm_channel1_disabled = 0,
		.max_ddr3_freq = 1600,
		.usb2_ports = {
			/* Length, Enable, OCn#, Location */
			{ 0x0040, 1, 0, /* P0: Back USB3 port  (OC0) */
			  USB_PORT_BACK_PANEL },
			{ 0x0040, 1, 0, /* P1: Back USB3 port  (OC0) */
			  USB_PORT_BACK_PANEL },
			{ 0x0040, 1, 1, /* P2: Flex Port on bottom (OC1) */
			  USB_PORT_FLEX },
			{ 0x0040, 1, USB_OC_PIN_SKIP, /* P3: Dock connector */
			  USB_PORT_DOCK },
			{ 0x0040, 1, USB_OC_PIN_SKIP, /* P4: Mini PCIE  */
			  USB_PORT_MINI_PCIE },
			{ 0x0040, 1, 1, /* P5: USB eSATA header (OC1) */
			  USB_PORT_FLEX },
			{ 0x0040, 1, 3, /* P6: Front Header J8H2 (OC3) */
			  USB_PORT_FRONT_PANEL },
			{ 0x0040, 1, 3, /* P7: Front Header J8H2 (OC3) */
			  USB_PORT_FRONT_PANEL },
			{ 0x0040, 1, 4, /* P8: USB/LAN Jack (OC4) */
			  USB_PORT_FRONT_PANEL },
			{ 0x0040, 1, 4, /* P9: USB/LAN Jack (OC4) */
			  USB_PORT_FRONT_PANEL },
			{ 0x0040, 1, 5, /* P10: Front Header J7H3 (OC5) */
			  USB_PORT_FRONT_PANEL },
			{ 0x0040, 1, 5, /* P11: Front Header J7H3 (OC5) */
			  USB_PORT_FRONT_PANEL },
			{ 0x0040, 1, 6, /* P12: USB/DP Jack (OC6) */
			  USB_PORT_FRONT_PANEL },
			{ 0x0040, 1, 6, /* P13: USB/DP Jack (OC6) */
			  USB_PORT_FRONT_PANEL },
		},
		.usb3_ports = {
			/* Enable, OCn# */
			{ 1, 0 }, /* P1; */
			{ 1, 0 }, /* P2; */
			{ 1, 0 }, /* P3; */
			{ 1, 0 }, /* P4; */
			{ 1, 0 }, /* P6; */
			{ 1, 0 }, /* P6; */
		},
	};

	struct romstage_params romstage_params = {
		.pei_data = &pei_data,
		.gpio_map = &mainboard_gpio_map,
		.rcba_config = &rcba_config[0],
		.bist = bist,
	};
	
	/* Setup SuperIO */
  static const pnp_devfn_t GLOBAL_PSEUDO_DEV = PNP_DEV(0x2e, 0);
  
  nuvoton_pnp_enter_conf_state(GLOBAL_PSEUDO_DEV);
  nuvoton_enable_serial(GLOBAL_PSEUDO_DEV, CONFIG_TTYS0_BASE);
  
  nuvoton_pnp_exit_conf_state(GLOBAL_PSEUDO_DEV);


	/* Call into the real romstage main with this board's attributes. */
	romstage_common(&romstage_params);
}
