/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#define IRQN 28

#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rng.h>

static uint64_t last_invoke = 0;

void rng_isr(void) {
	uint64_t cur = sys_clock_cycle_get_64();
	uint32_t fifo_data[4];

	if (LL_RNG_GetFfFullIrq(RNG)) {
		/* FIFO is full - flush it */
		if (!LL_RNG_IsActiveFlag_FIFO_FULL(RNG)) {
			printk("FF_FULL_IRQ but no FIFO_FULL?!\n");
		}

		int cnt = 0;
		while (LL_RNG_IsActiveFlag_VAL_READY(RNG)) {
			fifo_data[cnt++] = LL_RNG_GetRndVal(RNG);
		}

		if (cnt != 4) {
			printk("Only %d in FIFO?!\n", cnt);
		}

		WRITE_REG(RNG->IRQ_SR, RNG_IRQ_SR_FF_FULL_IRQ);

		const uint64_t delta = cur - last_invoke;
		printk("Delta=%lld µs - ", k_cyc_to_us_floor64(delta));
		last_invoke = cur;

		for (int i = 0; i < cnt; i++) {
			printk("%08X ", fifo_data[i]);
		}
		printk("\n");

	}

	if (LL_RNG_GetErrorIrq(RNG)) {
		printk("ERROR!\n");
		irq_disable(IRQN);
	}
}

int main(void)
{
	IRQ_CONNECT(IRQN, 0, rng_isr, NULL, 0);
	irq_enable(IRQN);

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_RNG);

	last_invoke = sys_clock_cycle_get_64();

	LL_RNG_Enable(RNG);
	LL_RNG_EnableEnErrorIrq(RNG);
	LL_RNG_EnableEnFfFullIrq(RNG);

	printk("Hello World! %s\n", CONFIG_BOARD_TARGET);
	printk("rng started, now we wait...\n");

	return 0;
}
