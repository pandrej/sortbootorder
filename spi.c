/*
 * This file is part of the coreboot project.
 *
 * Copyright (C) 2012 Advanced Micro Devices, Inc.
 * Copyright (c) 2014 Sage Electronic Engineering, Inc.
 * Copyright (c) 2015-2016 PC Engines GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <arch/io.h>
#include "spi.h"
#include <pci.h>

#define APA_TRACE(msg) printf("[APA]"); printf(msg);

#define FCH_YANGTZEE

#if defined (CONFIG_SB800_IMC_FWM)
#include "SBPLATFORM.h"
#include <vendorcode/amd/cimx/sb800/ECfan.h>

static int bus_claimed = 0;
#endif

static u32 spibar;

#ifndef FCH_YANGTZEE
static void reset_internal_fifo_pointer(void)
{
    //
    // APA
    // SPIx00: b20      FifoPtrClr
    // SPIxC0: b8:10    FifoPtr
    //
    // Clear interna fifo ptr
	do {
		writeb(spibar + 2, readb(spibar + 2) | 0x10);
	} while (readb(spibar + 0xD) & 0x7);
}
#endif

#ifdef FCH_YANGTZEE

static void execute_command(void)
{
    APA_TRACE("execute_command\n");
    //
    // FLASHROM --- DOES NOT WAIT ON BUSY BIT
    //
    writeb(readb(spibar + 2) | 1, spibar + 2);
    while (readb(spibar + 2) & 1);
}
#else

static void execute_command(void)
{
    u8 val = 0;
    u8 busy = 0;
    APA_TRACE("execute_command\n");

    val = readb(spibar + 2) & 1;

    busy = readb(spibar + 3) & 0x80;

    printf("before execution  ExecuteOpCode 0x%02x busy 0x%02x\n", val, busy);

    //
    // APA
    // SPIx00: b16 ExecuteOpCode
    //         Write 1 to execute the transaction in
    //         the alternate program registers. This bit returns to 0 when the transaction is complete.
    //
    // SPIx00: b31 SpiBusy
    //
	writeb(spibar + 2, readb(spibar + 2) | 1);

	while ((readb(spibar + 2) & 1) && (readb(spibar+3) & 0x80));

	//
	// FLASHROM --- DOES NOT WAIT ON BUSY BIT
	//mmio_writeb(mmio_readb(sb600_spibar + 2) | 1, sb600_spibar + 2);
	//while (mmio_readb(sb600_spibar + 2) & 1)

	val = readb(spibar + 2) & 1;
	busy = readb(spibar + 3) & 0x80;

	printf("before execution  ExecuteOpCode 0x%02x busy 0x%02x\n", val, busy);
}
#endif

void spi_init(void)
{
	pcidev_t dev  = PCI_DEV(0,0x14,3);
    spibar = pci_read_config32(dev, 0xA0) & ~0x1F;

	APA_TRACE("spibar:");
	printf("0x%02x\n", spibar);

    APA_TRACE("spibar + 1:");
    printf("0x%02x\n", spibar+1);

    APA_TRACE("spibar + 2:");
    printf("0x%02x\n", spibar+2);
}



#define FIFO_SIZE_YANGTZE 71

#ifdef FCH_YANGTZEE

/* Check the number of bytes to be transmitted */
static int check_readwritecnt(unsigned int writecnt, unsigned int readcnt)
{
    unsigned int maxwritecnt = FIFO_SIZE_YANGTZE;
    unsigned int maxreadcnt = FIFO_SIZE_YANGTZE - 3;

    if (writecnt > maxwritecnt) {
        printf("%s: SPI controller can not send %d bytes, it is limited to %d bytes\n",
              __func__, writecnt, maxwritecnt);
        return 1;
    }

    if (readcnt > maxreadcnt) {
        printf("%s: SPI controller can not receive %d bytes, it is limited to %d bytes\n",
              __func__, readcnt, maxreadcnt);
        return 1;
    }
    return 0;
}

int spi_xfer(struct spi_slave *slave,
        const void *dout,
        unsigned int bitsout,
        void *din,
        unsigned int bitsin)
{
    /* First byte is cmd which can not being sent through FIFO. */
    unsigned int writeCnt = bitsout/8;
    unsigned int readCnt = bitsin/8;
    unsigned char* writeBuff = (unsigned char*)dout;
    unsigned char* readBuff = (unsigned char*)din;

    //
    // First byte is cmd
    // which can not be sent through the buffer.
    //
    unsigned char cmd = *writeBuff++;

    writeCnt--;

    APA_TRACE("spi_xfer\n");

    printf("%s, cmd=0x%02x, writecnt=%d, readcnt=%d\n", __func__, cmd, writeCnt, readCnt);
    writeb(cmd, spibar + 0);

    int ret = check_readwritecnt(writeCnt, readCnt);
    if (ret != 0)
        return ret;

    /* Use the extended TxByteCount and RxByteCount registers. */
    writeb(writeCnt, spibar + 0x48);
    writeb(readCnt, spibar + 0x4b);

    printf("Filling buffer: ");
    int count;
    for (count = 0; count < writeCnt; count++) {
        printf("[%02x]", writeBuff[count]);
        writeb(writeBuff[count], spibar + 0x80 + count);
    }
    printf("\n");

    execute_command();

    printf("Reading buffer: ");
    for (count = 0; count < readCnt; count++) {
        readBuff[count] = readb(spibar + 0x80 + (writeCnt + count) % FIFO_SIZE_YANGTZE);
        printf("[%02x]", readBuff[count]);
    }
    printf("\n");

    return 0;
}
#else
int spi_xfer(struct spi_slave *slave, const void *dout,
		unsigned int bitsout, void *din, unsigned int bitsin)
{
	/* First byte is cmd which can not being sent through FIFO. */
    u8 testCmd = *(u8*) dout;
    u8* dptr = (u8*) dout;
    unsigned int bOutCnt = bitsout/8;

    u32 testDoutVal = *(u32*)dout;

	u32 cmd = *(u32 *)dout++;
	u8 readoffby1;
	u32 readwrite;
	u8 bytesout, bytesin;
	u8 count;


	APA_TRACE("spi_xfer\n");
	printf("testCmd 0x%02x\n", testCmd);
	printf("testDoutVal 0x%02x  sizeof(testDoutVal) %lu \n", testDoutVal, sizeof(testDoutVal));

    printf("dout[0] 0x%02x,  dout[1] 0x%02x, dout[2] 0x%02x\n", dptr[0],  dptr[1], dptr[2]);
    printf("bOutCnt %d \n", bOutCnt);

    printf("cmd 0x%02x\n, bitsout %d \n", cmd, bitsout);
    APA_TRACE("spibar\n:");
    printf("0x%02x\n", spibar);

	bitsout -= 8;
	bytesout = bitsout / 8;
	bytesin  = bitsin / 8;

	printf("bytesout %d \n", bytesout);

	readoffby1 = bytesout ? 0 : 1;

	readwrite = (bytesin + readoffby1) << 4 | bytesout;

	printf("readoffby1 %d \n", readoffby1);
	printf("readwrite %d \n", readwrite);

    writeb(spibar + 1, readwrite);
	writeb(spibar + 0, cmd);


	APA_TRACE("spi_xfer reset_internal_fifo_pointer\n");
	reset_internal_fifo_pointer();

	//SPIx0C SPI_Cntrl1
	//  b7:0 SpiParameters TX/RX FIFO port which can take up to 8 bytes
	//

	printf("APA spi_xfer writing bytes to TX/RX FIFO : bytesout = %d\n", bytesout);
	for (count = 0; count < bytesout; count++, dout++) {
		writeb(spibar + 0x0C, *(u32 *)dout);
	}

	APA_TRACE("spi_xfer reset_internal_fifo_pointer\n");
	reset_internal_fifo_pointer();

	APA_TRACE("spi_xfer execute command\n");
	execute_command();

	APA_TRACE("spi_xfer reset_internal_fifo_pointer\n");
	reset_internal_fifo_pointer();
	/* Skip the bytes we sent. */
	printf("APA spi_xfer skipping bytesout = %d bytes before reading result from TX/RX FIFO\n", bytesout);
	for (count = 0; count < bytesout; count++) {
		cmd = readb(spibar + 0x0C);
	}

	//reset_internal_fifo_pointer();
	printf("APA spi_xfer reading bytes bytesin %d bytes \n", bytesin);

	for (count = 0; count < bytesin; count++, din++) {
		*(u8 *)din = readb(spibar + 0x0C);
	}
	return 0;
}
#endif

#if defined (CONFIG_SB800_IMC_FWM)

static void ImcSleep(void)/* Check the number of bytes to be transmitted and extract opcode. */
static int check_readwritecnt(struct flashctx *flash, unsigned int writecnt, unsigned int readcnt)
{
	unsigned int maxwritecnt = flash->mst->spi.max_data_write + 3;
	if (writecnt > maxwritecnt) {
		msg_pinfo("%s: SPI controller can not send %d bytes, it is limited to %d bytes\n",
			  __func__, writecnt, maxwritecnt);
		return SPI_INVALID_LENGTH;
	}

	unsigned int maxreadcnt = flash->mst->spi.max_data_read;
	if (readcnt > maxreadcnt) {
		msg_pinfo("%s: SPI controller can not receive %d bytes, it is limited to %d bytes\n",
			  __func__, readcnt, maxreadcnt);
		return SPI_INVALID_LENGTH;
	}
	return 0;
}
{
	u8	cmd_val = 0x96;		/* Kick off IMC Mailbox command 96 */
	u8	reg0_val = 0;		/* clear response register */
	u8	reg1_val = 0xB4;	/* request ownership flag */

	WriteECmsg (MSG_REG0, AccWidthUint8, &reg0_val);
	WriteECmsg (MSG_REG1, AccWidthUint8, &reg1_val);
	WriteECmsg (MSG_SYS_TO_IMC, AccWidthUint8, &cmd_val);

	WaitForEcLDN9MailboxCmdAck();
}


static void ImcWakeup(void)
{
	u8	cmd_val = 0x96;		/* Kick off IMC Mailbox command 96 */
	u8	reg0_val = 0;;		/* clear response register */
	u8	reg1_val = 0xB5;	/* release ownership flag */

	WriteECmsg (MSG_REG0, AccWidthUint8, &reg0_val);
	WriteECmsg (MSG_REG1, AccWidthUint8, &reg1_val);
	WriteECmsg (MSG_SYS_TO_IMC, AccWidthUint8, &cmd_val);

	WaitForEcLDN9MailboxCmdAck();
}
#endif

int spi_claim_bus(struct spi_slave *slave)
{
    APA_TRACE("spi_claim_bus");
#if defined (CONFIG_SB800_IMC_FWM)

	if (slave->rw == SPI_WRITE_FLAG) {
		bus_claimed++;
		if (bus_claimed == 1)
			ImcSleep();
	}
#endif

	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
    APA_TRACE("spi_release_bus");
#if defined (CONFIG_SB800_IMC_FWM)

	if (slave->rw == SPI_WRITE_FLAG)  {
		bus_claimed--;
		if (bus_claimed <= 0) {
			bus_claimed = 0;
			ImcWakeup();
		}
	}
#endif
}

void spi_cs_activate(struct spi_slave *slave)
{
}

void spi_cs_deactivate(struct spi_slave *slave)
{
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	struct spi_slave *slave = malloc(sizeof(*slave));

	if (!slave) {
		return NULL;
	}

	memset(slave, 0, sizeof(*slave));

	return slave;
}
