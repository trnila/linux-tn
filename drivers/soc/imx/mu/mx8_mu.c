/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mx8_mu.h>

#define TIMEOUT_US 1000000

static int version;

/*!
 * This function sets the Flag n of the MU.
 */
int32_t MU_SetFn(void __iomem *base, uint32_t Fn)
{
	uint32_t reg, offset;

	reg = Fn & (~MU_CR_Fn_MASK1);
	if (reg > 0)
		return -EINVAL;

	offset = unlikely(version == MU_VER_ID_V10)
			  ? MU_V10_ACR_OFFSET1 : MU_ACR_OFFSET1;

	reg = readl_relaxed(base + offset);
	/*  Clear ABFn. */
	reg &= ~MU_CR_Fn_MASK1;
	reg |= Fn;
	writel_relaxed(reg, base + offset);

	return 0;
}

/*!
 * This function reads the status from status register.
 */
uint32_t MU_ReadStatus(void __iomem *base)
{
	uint32_t reg, offset;

	offset = unlikely(version == MU_VER_ID_V10)
			  ? MU_V10_ASR_OFFSET1 : MU_ASR_OFFSET1;

	reg = readl_relaxed(base + offset);

	return reg;
}

/*!
 * This function enables specific RX full interrupt.
 */
void MU_EnableRxFullInt(void __iomem *base, uint32_t index)
{
	uint32_t reg, offset;

	offset = unlikely(version == MU_VER_ID_V10)
			  ? MU_V10_ACR_OFFSET1 : MU_ACR_OFFSET1;

	reg = readl_relaxed(base + offset);
	reg &= ~(MU_CR_GIRn_MASK1 | MU_CR_NMI_MASK1);
	reg |= MU_CR_RIE0_MASK1 >> index;
	writel_relaxed(reg, base + offset);
}

/*!
 * This function enables specific general purpose interrupt.
 */
void MU_EnableGeneralInt(void __iomem *base, uint32_t index)
{
	uint32_t reg, offset;

	offset = unlikely(version == MU_VER_ID_V10)
			  ? MU_V10_ACR_OFFSET1 : MU_ACR_OFFSET1;

	reg = readl_relaxed(base + offset);
	reg &= ~(MU_CR_GIRn_MASK1 | MU_CR_NMI_MASK1);
	reg |= MU_CR_GIE0_MASK1 >> index;
	writel_relaxed(reg, base + offset);
}

/*
 * Wait and send message to the other core.
 */
void MU_SendMessage(void __iomem *base, uint32_t regIndex, uint32_t msg)
{
	void __iomem *ASR, *ATR0;
	uint32_t mask = MU_SR_TE0_MASK1 >> regIndex;
	int err;
	uint32_t reg;

	ASR = base + MU_ASR_OFFSET1;
	ATR0 = base + MU_ATR0_OFFSET1;

	if (unlikely(version == MU_VER_ID_V10)) {
		ASR = base + MU_V10_ASR_OFFSET1;
		ATR0 = base + MU_V10_ATR0_OFFSET1;
	}

	/* Wait TX register to be empty. */
	err = readl_poll_timeout(ASR, reg, reg & mask, 0, TIMEOUT_US);
	if(err) {
		pr_err("MU_SendMessage timed out\n");
	}

	writel_relaxed(msg, ATR0 + regIndex * 4);
}


/*
 * Wait to receive message from the other core.
 */
void MU_ReceiveMsg(void __iomem *base, uint32_t regIndex, uint32_t *msg)
{
	void __iomem *ASR, *ARR0;
	uint32_t mask = MU_SR_RF0_MASK1 >> regIndex;
	int err;
	uint32_t reg;

	ASR = base + MU_ASR_OFFSET1;
	ARR0 = base + MU_ARR0_OFFSET1;

	if (unlikely(version == MU_VER_ID_V10)) {
		ASR = base + MU_V10_ASR_OFFSET1;
		ARR0 = base + MU_V10_ARR0_OFFSET1;
	}

	/* Wait RX register to be full. */
	err = readl_poll_timeout(ASR, reg, reg & mask, 0, TIMEOUT_US);
	if(err) {
		pr_err("MU_ReceiveMsg timed out\n");
	}
	*msg = readl_relaxed(ARR0 + (regIndex * 4));
}



void MU_Init(void __iomem *base)
{
	uint32_t reg, offset, err;

	version = readl_relaxed(base) >> 16;

	offset = unlikely(version == MU_VER_ID_V10)
			  ? MU_V10_ACR_OFFSET1 : MU_ACR_OFFSET1;

	reg = readl_relaxed(base + offset);
	/* Clear GIEn, RIEn, TIEn, GIRn and ABFn. */
	reg &= ~(MU_CR_GIEn_MASK1 | MU_CR_RIEn_MASK1 | MU_CR_TIEn_MASK1
		 | MU_CR_GIRn_MASK1 | MU_CR_NMI_MASK1 | MU_CR_Fn_MASK1);
	/* clear MUR to reset MU on both sides */
	reg |= MU_CR_MUR_MASK1;
	writel_relaxed(reg, base + offset);

	/* wait for reset */
	offset = unlikely(version == MU_VER_ID_V10)
			  ? MU_V10_ASR_OFFSET1 : MU_ASR_OFFSET1;
	err = readl_poll_timeout(base + offset, reg, !(reg & MU_SR_BRS_MASK1), 0, TIMEOUT_US);
	if(err) {
		printk("Failed to reset MU!\n");
	}
}

EXPORT_SYMBOL(MU_ReceiveMsg);
EXPORT_SYMBOL(MU_ReadStatus);
EXPORT_SYMBOL(MU_SetFn);
EXPORT_SYMBOL(MU_EnableRxFullInt);
EXPORT_SYMBOL(MU_Init);
EXPORT_SYMBOL(MU_SendMessage);

/**@}*/

