/*
 * QEMU G233 SPI device (Learning QEMU 2025)
 *
 * Copyright (c) 2025 Chaoqun Zheng
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_G233_SPI_H
#define HW_G233_SPI_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qemu/typedefs.h"
#include "qemu/fifo8.h"

#define TYPE_G233_SPI "g233.spi"
#define G233_SPI(obj) OBJECT_CHECK(G233SPIState, (obj), TYPE_G233_SPI)

typedef struct G233SPIState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;

    uint32_t cr1;
    uint32_t cr2;
    uint32_t sr;
    uint32_t dr;
    uint32_t csctrl;

    SSIBus *spi;

    qemu_irq irq;
    qemu_irq *cs_lines;

    Fifo8 tx_fifo;
    Fifo8 rx_fifo;
} G233SPIState;

#endif /* HW_MISC_G233_SPI_H */