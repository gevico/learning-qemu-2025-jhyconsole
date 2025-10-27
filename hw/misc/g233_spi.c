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

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/fifo8.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/misc/g233_spi.h"
#include "hw/ssi/ssi.h"

#define G233_SPI_SIZE  0x1000
#define NUM_CS  4

/* register offsets */
#define G233_SPI_CR1      0x00
#define G233_SPI_CR2      0x04
#define G233_SPI_SR       0x08
#define G233_SPI_DR       0x0c
#define G233_SPI_CSCTRL   0x10

/* SPI Control Register 1 (CR1) bits */
#define SPI_CR1_SPE     (1 << 6)   /* SPI Enable */
#define SPI_CR1_MSTR    (1 << 2)   /* Master mode */

/* SPI Control Register 2 (CR2) bits */
#define SPI_CR2_TXEIE     (1 << 7)
#define SPI_CR2_RXNEIE    (1 << 6)
#define SPI_CR2_ERRIE     (1 << 5)
#define SPI_CR2_SSOE      (1 << 4)

/* SPI Status Register (SR) bits */
#define SPI_SR_TXE      (1 << 1)   /* Transmit buffer empty */
#define SPI_SR_RXNE     (1 << 0)   /* Receive buffer not empty */
#define SPI_SR_BSY      (1 << 7)   /* Busy flag */
#define SPI_SR_OVERRUN  (1 << 3)
#define SPI_SR_UNDERRUN (1 << 2)

#define RXDATA_EMPTY    (1 << 31)

#define FIFO_CAPACITY   1

#define CSi_EN(sr, i) \
    ( ( (sr) >> (i) ) & 1 )

#define CSi_ACT(sr, i) \
    ( ( (sr) >> ( (i) + 4 ) ) & 1)

static void g233_spi_txfifo_reset(G233SPIState *s)
{
    fifo8_reset(&s->tx_fifo);
}

static void g233_spi_rxfifo_reset(G233SPIState *s)
{
    fifo8_reset(&s->rx_fifo);
}

static void g233_spi_update_cs(G233SPIState *s)
{
    int i;

    for (i = 0; i < NUM_CS; i++) {
        if (!CSi_EN(s->csctrl, i)) {
            qemu_set_irq(s->cs_lines[i], 1);
            continue;
        }

        if (CSi_ACT(s->csctrl, i)) {
            qemu_set_irq(s->cs_lines[i], 0);
        } else {
            qemu_set_irq(s->cs_lines[i], 1);
        }
    }
}

static void g233_spi_update_irq(G233SPIState *s)
{
    int level = 0;

    /* Is trigger TX int ? */
    if ((s->cr2 & SPI_CR2_TXEIE) && (s->sr & SPI_SR_TXE))
        level = 1;

    /* Is trigger RX int ? */
    if ((s->cr2 & SPI_CR2_RXNEIE) && (s->sr & SPI_SR_RXNE))
        level = 1;

    /* Is trigger ERR int ? */
    if ((s->cr2 & SPI_CR2_ERRIE) && (s->sr & (SPI_SR_OVERRUN | SPI_SR_UNDERRUN)))
        level = 1;

    qemu_set_irq(s->irq, level);
}

static void g233_spi_update_state(G233SPIState *s)
{
    if (fifo8_is_empty(&s->tx_fifo)) {
        s->sr |= SPI_SR_TXE;
    } else {
        s->sr &= ~SPI_SR_TXE;
    }

    if (!fifo8_is_empty(&s->rx_fifo)) {
        s->sr |= SPI_SR_RXNE;
    } else {
        s->sr &= ~SPI_SR_RXNE;
    }

    g233_spi_update_irq(s);
}

static void g233_spi_reset(DeviceState *d)
{
    G233SPIState *s = G233_SPI(d);

    /* reset registers */
    s->cr1 = 0x0;
    s->cr2 = 0x0;
    s->sr = 0x2;
    s->dr = 0xc;
    s->csctrl = 0x0;

    g233_spi_txfifo_reset(s);
    g233_spi_rxfifo_reset(s);

    g233_spi_update_state(s);
    g233_spi_update_cs(s);
}

static void g233_spi_flush_txfifo(G233SPIState *s)
{
    uint8_t tx;
    uint8_t rx;

    s->sr |= SPI_SR_BSY;

    while (!fifo8_is_empty(&s->tx_fifo)) {
        tx = fifo8_pop(&s->tx_fifo);
        rx = ssi_transfer(s->spi, tx);

        if (!fifo8_is_full(&s->rx_fifo)) {
            fifo8_push(&s->rx_fifo, rx);
        } else {
            s->sr |= SPI_SR_OVERRUN;
        }
    }

    s->sr &= ~SPI_SR_BSY;
}

static uint64_t g233_spi_read(void *opaque, hwaddr offset,
                              unsigned int size)
{
    G233SPIState *s = opaque;
    uint32_t val = 0;

    /* Update state before reading. */
    g233_spi_update_state(s);

    switch (offset) {
    case G233_SPI_CR1:
        val = s->cr1;
        break;
    case G233_SPI_CR2:
        val = s->cr2;
        break;
    case G233_SPI_SR:
        val = s->sr;
        break;
    case G233_SPI_DR:
        if (fifo8_is_empty(&s->rx_fifo)) {
            return RXDATA_EMPTY;
        }
        val = fifo8_pop(&s->rx_fifo);
        break;
    case G233_SPI_CSCTRL:
        val = s->csctrl;
        break;
    }

    return val;
}

static void g233_spi_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned int size)
{
    G233SPIState *s = opaque;

    switch (offset) {
    case G233_SPI_CR1:
        s->cr1 = (uint32_t)value;
        break;
    case G233_SPI_CR2:
        s->cr2 = (uint32_t)value;
        break;
    case G233_SPI_SR:
        if (value & SPI_SR_OVERRUN)
            s->sr &= ~SPI_SR_OVERRUN;
        if (value & SPI_SR_UNDERRUN)
            s->sr &= ~SPI_SR_UNDERRUN;
        break;
    case G233_SPI_DR:
        if (!fifo8_is_full(&s->tx_fifo)) {
            fifo8_push(&s->tx_fifo, (uint8_t)value);
            g233_spi_flush_txfifo(s);
        } else {
            s->sr |= SPI_SR_OVERRUN;
        }
        break;
    case G233_SPI_CSCTRL:
        s->csctrl = (uint32_t)value;
        g233_spi_update_cs(s);
        break;
    }

    /* Update state after writing. */
    g233_spi_update_state(s);
}

static const MemoryRegionOps g233_spi_ops = {
    .read = g233_spi_read,
    .write = g233_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void g233_spi_realize(DeviceState *dev, Error **errp)
{
    int i;
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    G233SPIState *s = G233_SPI(dev);

    s->spi = ssi_create_bus(dev, "spi");

    sysbus_init_irq(sbd, &s->irq);
    s->cs_lines = g_new0(qemu_irq, NUM_CS);
    for (i = 0; i < NUM_CS; ++i) {
        sysbus_init_irq(sbd, &s->cs_lines[i]);
    }

    memory_region_init_io(&s->iomem, OBJECT(dev), &g233_spi_ops, s,
                          "g233-spi-mmio", G233_SPI_SIZE);

    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    fifo8_create(&s->tx_fifo, FIFO_CAPACITY);
    fifo8_create(&s->rx_fifo, FIFO_CAPACITY);
}

static void g233_spi_instance_init(Object *obj)
{
    /* nothing special */
}

static void g233_spi_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, g233_spi_reset);
    dc->realize = g233_spi_realize;
}

static const TypeInfo g233_spi_typeinfo = {
    .name = TYPE_G233_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(G233SPIState),
    .instance_init = g233_spi_instance_init,
    .class_init = g233_spi_class_init,
};

static void g233_spi_register_types(void)
{
    type_register_static(&g233_spi_typeinfo);
}

type_init(g233_spi_register_types)