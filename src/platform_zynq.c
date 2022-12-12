/*
 * Copyright (C) 2010 - 2021 Xilinx, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */
/*
* platform_zynq.c
*
* Zynq platform specific functions.
*
* 02/29/2012: UART initialization is removed. Timer initializations are
* removed. All unnecessary include files and hash defines are removed.
* 03/01/2013: Timer initialization is added back. Support for SI #692601 is
* added in the timer callback. The SI #692601 refers to the following issue.
*
* The EmacPs has a HW bug on the Rx path for heavy Rx traffic.
* Under heavy Rx traffic because of the HW bug there are times when the Rx path
* becomes unresponsive. The workaround for it is to check for the Rx path for
* traffic (by reading the stats registers regularly). If the stats register
* does not increment for sometime (proving no Rx traffic), the function resets
* the Rx data path.
*
* </pre>
 */

#include "xparameters.h"
#include "xparameters_ps.h"	/* defines XPAR values */
#include "xil_cache.h"
#include "platform.h"
#include "xscugic.h"
#include "xil_printf.h"
#include "netif/xadapter.h"
#include "xscutimer.h"
#include "xaxidma.h"
#include "xgpio.h"
#include "xtime_l.h"
#include <string.h>


#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define TIMER_DEVICE_ID		XPAR_SCUTIMER_DEVICE_ID
#define INTC_BASE_ADDR		XPAR_SCUGIC_0_CPU_BASEADDR
#define INTC_DIST_BASE_ADDR	XPAR_SCUGIC_0_DIST_BASEADDR
#define TIMER_IRPT_INTR		XPAR_SCUTIMER_INTR
#define DMA_DEV_ID			XPAR_AXIDMA_0_DEVICE_ID
#define RX_INTR_ID			XPAR_FABRIC_AXIDMA_0_S2MM_INTROUT_VEC_ID
#define TX_INTR_ID			XPAR_FABRIC_AXIDMA_0_MM2S_INTROUT_VEC_ID

#define GPIO_AD_SEL_ID 	  	XPAR_AXI_GPIO_AD_SEL_DEVICE_ID
#define GPIO_D_OUT_ID     	XPAR_AXI_GPIO_D_OUT_DEVICE_ID
#define GPIO_D_TRIG_ID    	XPAR_AXI_GPIO_D_TRIG_DEVICE_ID
#define GPIO_EOC_ID       	XPAR_AXI_GPIO_EOC_DEVICE_ID
#define GPIO_EOS_ID       	XPAR_AXI_GPIO_EOS_DEVICE_ID
#define GPIO_START_SIG_ID 	XPAR_AXI_GPIO_START_SIG_DEVICE_ID
#define GPIO_EOC_INTR_ID  	XPAR_FABRIC_AXI_GPIO_EOC_IP2INTC_IRPT_INTR
#define GPIO_D_TRIG_INTR_ID XPAR_FABRIC_AXI_GPIO_D_TRIG_IP2INTC_IRPT_INTR
#define GPIO_EOS_INTR_ID 	XPAR_FABRIC_AXI_GPIO_EOS_IP2INTC_IRPT_INTR

#define GPIO_CHANNEL 1


#define RESET_RX_CNTR_LIMIT	400
#define RESET_TIMEOUT_COUNTER 10000

static XScuTimer timer_instance;
static XAxiDma dma_instance;
static XGpio gpio_trig, gpio_eoc, gpio_eos, gpio_d_out, gpio_start;

static int reset_rx_cntr = 0;
extern struct netif server_netif;

volatile int tx_done = 0;
volatile int rx_done = 0;
volatile int error = 0;
int send_udp = 0;

u8 tx_buffer[BUFFER_SIZE] = {0};
u8 rx_buffer[BUFFER_SIZE] = {0};

u16 counter_bits = 0;
u16 data_read = 0;
int counter_pixels = 0;


void timer_callback(XScuTimer * timer_inst)
{

	/* For providing an SW alternative for the SI #692601. Under heavy
	 * Rx traffic if at some point the Rx path becomes unresponsive, the
	 * following API call will ensures a SW reset of the Rx path. The
	 * API xemacpsif_resetrx_on_no_rxdata is called every 100 milliseconds.
	 * This ensures that if the above HW bug is hit, in the worst case,
	 * the Rx path cannot become unresponsive for more than 100
	 * milliseconds.
	 */
	if (reset_rx_cntr >= RESET_RX_CNTR_LIMIT) {
		xemacpsif_resetrx_on_no_rxdata(&server_netif);
		reset_rx_cntr = 0;
	}

	XScuTimer_ClearInterruptStatus(timer_inst);
}

static void rx_dma_callback(void *callback)
{
	u32 irq_status;
	int time_out;
	XAxiDma *axi_dma_inst = (XAxiDma *)callback;

	irq_status = XAxiDma_IntrGetIrq(axi_dma_inst, XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrAckIrq(axi_dma_inst, irq_status, XAXIDMA_DEVICE_TO_DMA);

	if (!(irq_status & XAXIDMA_IRQ_ALL_MASK)) {
		return;
	}

	if ((irq_status & XAXIDMA_IRQ_ERROR_MASK)) {

		error = 1;
		XAxiDma_Reset(axi_dma_inst);
		time_out = RESET_TIMEOUT_COUNTER;

		while (time_out)
		{
			if(XAxiDma_ResetIsDone(axi_dma_inst))
			{
				break;
			}
			time_out -= 1;
		}

		return;
	}

	if ((irq_status & XAXIDMA_IRQ_IOC_MASK))
	{
		rx_done = 1;
	}
}


static void tx_dma_callback(void *callback)
{

	u32 irq_status;
	int time_out;
	XAxiDma *axi_dma_inst = (XAxiDma *)callback;

	irq_status = XAxiDma_IntrGetIrq(axi_dma_inst, XAXIDMA_DMA_TO_DEVICE);
	XAxiDma_IntrAckIrq(axi_dma_inst, irq_status, XAXIDMA_DMA_TO_DEVICE);

	if (!(irq_status & XAXIDMA_IRQ_ALL_MASK)) {

		return;
	}

	if ((irq_status & XAXIDMA_IRQ_ERROR_MASK)) {

		error = 1;
		XAxiDma_Reset(axi_dma_inst);

		time_out = RESET_TIMEOUT_COUNTER;

		while (time_out)
		{
			if (XAxiDma_ResetIsDone(axi_dma_inst))
			{
				break;
			}
			time_out -= 1;
		}
		return;
	}

	if ((irq_status & XAXIDMA_IRQ_IOC_MASK))
	{
		tx_done = 1;
		send_udp = 1;
	}
}

static void gpio_eos_intr_callback(void *callback)
{
	XGpio *gpio_inst = (XGpio *)callback;
	u32 irq_status = XGpio_InterruptGetStatus(gpio_inst);

	XGpio_InterruptClear(gpio_inst, GPIO_CHANNEL);

	if(irq_status & XGPIO_IR_CH1_MASK)
	{
		xil_printf("Interrupt for GPIO EOS\r\n");
		counter_pixels = 0;
		dma_transfer();
		send_udp = 1;
	}
	else
	{
		xil_printf("Unknown interrupt for GPIO EOS\r\n");
	}


}

static void gpio_eoc_intr_callback(void *callback)
{
	XGpio *gpio_inst = (XGpio *)callback;
	u32 irq_status = XGpio_InterruptGetStatus(gpio_inst);
	if(irq_status & XGPIO_IR_CH1_MASK)
	{
		xil_printf("Interrupt for GPIO EOC\r\n");

		rx_buffer[counter_pixels] = data_read;
		counter_bits = 0;
		data_read = 0;
		counter_pixels++;

	}
	else
	{
		xil_printf("Unknown interrupt for GPIO EOS\r\n");
	}

	XGpio_InterruptClear(gpio_inst, GPIO_CHANNEL);

}

static void gpio_d_trig_intr_callback(void *callback)
{
	XGpio *gpio_inst = (XGpio *)callback;
	u32 irq_status = XGpio_InterruptGetStatus(gpio_inst);
	if(irq_status & XGPIO_IR_CH1_MASK)
	{
		xil_printf("Interrupt for GPIO D TRIG\r\n");
		read_data_from_d_out();

	}
	else
	{
		xil_printf("Unknown interrupt for GPIO EOS\r\n");
	}

	XGpio_InterruptClear(gpio_inst, GPIO_CHANNEL);

}

void read_data_from_d_out(void)
{
	u8 bit_out = XGpio_DiscreteRead(&gpio_d_out, GPIO_CHANNEL);
	data_read |= bit_out << counter_bits;
	counter_bits++;
}

void start_stop_measurements(int start_stop)
{
	if(start_stop)
	{
		XGpio_DiscreteWrite(&gpio_start, GPIO_CHANNEL, 1);
	}
	else
	{
		XGpio_DiscreteWrite(&gpio_start, GPIO_CHANNEL, 0);
	}
}

void init_buff(void)
{
	u8 *tx_buffer_ptr = tx_buffer;
	for (int i = 0; i < BUFFER_SIZE; i++)
		tx_buffer_ptr[i] = (i % 10) + '0';
}

int dma_transfer(void)
{
	u8 *tx_buffer_ptr = tx_buffer;
	u8 *rx_buffer_ptr = rx_buffer;

	tx_done = 0;
	rx_done = 0;
	error = 0;

	init_buff();
	Xil_DCacheFlushRange((UINTPTR)tx_buffer_ptr, BUFFER_SIZE);
	Xil_DCacheFlushRange((UINTPTR)rx_buffer_ptr, BUFFER_SIZE);

	int status = XAxiDma_SimpleTransfer(&dma_instance,(UINTPTR) rx_buffer_ptr,
			BUFFER_SIZE, XAXIDMA_DEVICE_TO_DMA);

	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	status = XAxiDma_SimpleTransfer(&dma_instance,(UINTPTR) tx_buffer_ptr,
			BUFFER_SIZE, XAXIDMA_DMA_TO_DEVICE);
	Xil_DCacheFlushRange((UINTPTR)rx_buffer_ptr, BUFFER_SIZE);

	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	while (!tx_done && !rx_done && !error) {
			/* NOP */
	}

	return XST_SUCCESS;
}

void platform_setup_gpio(void)
{
	XGpio_Config *cfg_ptr;
	XGpio gpio_ad_sel;

	cfg_ptr = XGpio_LookupConfig(GPIO_AD_SEL_ID);
	XGpio_CfgInitialize(&gpio_ad_sel, cfg_ptr, cfg_ptr->BaseAddress);
	XGpio_SetDataDirection(&gpio_ad_sel, GPIO_CHANNEL, 0);
	XGpio_DiscreteWrite(&gpio_ad_sel, GPIO_CHANNEL, 0);

	cfg_ptr = XGpio_LookupConfig(GPIO_D_OUT_ID);
	XGpio_CfgInitialize(&gpio_d_out, cfg_ptr, cfg_ptr->BaseAddress);
	XGpio_SetDataDirection(&gpio_d_out, GPIO_CHANNEL, 1);

	cfg_ptr = XGpio_LookupConfig(GPIO_START_SIG_ID);
	XGpio_CfgInitialize(&gpio_start, cfg_ptr, cfg_ptr->BaseAddress);
	XGpio_SetDataDirection(&gpio_start, GPIO_CHANNEL, 0);

	cfg_ptr = XGpio_LookupConfig(GPIO_EOS_ID);
	XGpio_CfgInitialize(&gpio_eos, cfg_ptr, cfg_ptr->BaseAddress);
	XGpio_SetDataDirection(&gpio_eos, GPIO_CHANNEL, 1);

	cfg_ptr = XGpio_LookupConfig(GPIO_EOC_ID);
	XGpio_CfgInitialize(&gpio_eoc, cfg_ptr, cfg_ptr->BaseAddress);
	XGpio_SetDataDirection(&gpio_eoc, GPIO_CHANNEL, 1);

	cfg_ptr = XGpio_LookupConfig(GPIO_D_TRIG_ID);
	XGpio_CfgInitialize(&gpio_trig, cfg_ptr, cfg_ptr->BaseAddress);
	XGpio_SetDataDirection(&gpio_trig, GPIO_CHANNEL, 1);

}

void platform_setup_dma(void)
{
	int status = XST_SUCCESS;
	XAxiDma_Config *config_ptr;

	config_ptr = XAxiDma_LookupConfig(DMA_DEV_ID);
	status = XAxiDma_CfgInitialize(&dma_instance, config_ptr);

	if (status != XST_SUCCESS) {
		xil_printf("DMA Cfg initialization failed...\r\n");
		return;
	}

	if(XAxiDma_HasSg(&dma_instance)){
		xil_printf("Device configured as SG mode \r\n");
		return;
	}

	XAxiDma_IntrDisable(&dma_instance, XAXIDMA_IRQ_ALL_MASK,
						XAXIDMA_DMA_TO_DEVICE);

	XAxiDma_IntrDisable(&dma_instance, XAXIDMA_IRQ_ALL_MASK,
				XAXIDMA_DEVICE_TO_DMA);

	return;
}

void platform_setup_timer(void)
{
	int status = XST_SUCCESS;
	XScuTimer_Config *config_ptr;
	int timer_load_value = 0;

	config_ptr = XScuTimer_LookupConfig(TIMER_DEVICE_ID);
	status = XScuTimer_CfgInitialize(&timer_instance, config_ptr,
			config_ptr->BaseAddr);
	if (status != XST_SUCCESS) {

		xil_printf("Scutimer Cfg initialization failed\r\n");
		return;
	}

	status = XScuTimer_SelfTest(&timer_instance);
	if (status != XST_SUCCESS) {
		xil_printf("Scutimer Self test failed\r\n");
		return;

	}

	XScuTimer_EnableAutoReload(&timer_instance);
	/*
	 * Set for 250 milli seconds timeout.
	 */
	timer_load_value = XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ / 8;

	XScuTimer_LoadTimer(&timer_instance, timer_load_value);
	return;
}

void platform_setup_interrupts(void)
{
	Xil_ExceptionInit();

	XScuGic_DeviceInitialize(INTC_DEVICE_ID);

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
			(Xil_ExceptionHandler)XScuGic_DeviceInterruptHandler,
			(void *)INTC_DEVICE_ID);

	XScuGic_RegisterHandler(INTC_BASE_ADDR, TIMER_IRPT_INTR,
					(Xil_ExceptionHandler)timer_callback,
					(void *)&timer_instance);
	XScuGic_RegisterHandler(INTC_BASE_ADDR, RX_INTR_ID,
					(Xil_ExceptionHandler)rx_dma_callback,
					(void *)&dma_instance);
	XScuGic_RegisterHandler(INTC_BASE_ADDR, TX_INTR_ID,
					(Xil_ExceptionHandler)tx_dma_callback,
					(void *)&dma_instance);
	XScuGic_RegisterHandler(INTC_BASE_ADDR, GPIO_EOC_INTR_ID,
					(Xil_ExceptionHandler)gpio_eoc_intr_callback,
					(void *)&gpio_eoc);
	XScuGic_RegisterHandler(INTC_BASE_ADDR, GPIO_EOS_INTR_ID,
					(Xil_ExceptionHandler)gpio_eos_intr_callback,
					(void *)&gpio_eos);
	XScuGic_RegisterHandler(INTC_BASE_ADDR, GPIO_D_TRIG_INTR_ID,
					(Xil_ExceptionHandler)gpio_d_trig_intr_callback,
					(void *)&gpio_trig);


	XScuGic_EnableIntr(INTC_DIST_BASE_ADDR, TIMER_IRPT_INTR);
	XScuGic_EnableIntr(INTC_DIST_BASE_ADDR, RX_INTR_ID);
	XScuGic_EnableIntr(INTC_DIST_BASE_ADDR, TX_INTR_ID);
	XScuGic_EnableIntr(INTC_DIST_BASE_ADDR, GPIO_EOC_INTR_ID);
	XScuGic_EnableIntr(INTC_DIST_BASE_ADDR, GPIO_EOS_INTR_ID);
	XScuGic_EnableIntr(INTC_DIST_BASE_ADDR, GPIO_D_TRIG_INTR_ID);


	return;
}

void platform_enable_interrupts()
{
	Xil_ExceptionEnable();

	XScuTimer_EnableInterrupt(&timer_instance);
	XScuTimer_Start(&timer_instance);

	XAxiDma_IntrEnable(&dma_instance, XAXIDMA_IRQ_ALL_MASK,
							XAXIDMA_DMA_TO_DEVICE);
	XAxiDma_IntrEnable(&dma_instance, XAXIDMA_IRQ_ALL_MASK,
							XAXIDMA_DEVICE_TO_DMA);


	XGpio_InterruptGlobalEnable(&gpio_trig);
	XGpio_InterruptGlobalEnable(&gpio_eoc);
	XGpio_InterruptGlobalEnable(&gpio_eos);
	XGpio_InterruptEnable(&gpio_trig, XGPIO_IR_CH1_MASK);
	XGpio_InterruptEnable(&gpio_eoc, XGPIO_IR_CH1_MASK);
	XGpio_InterruptEnable(&gpio_eos, XGPIO_IR_CH1_MASK);

	return;
}

void init_platform()
{
	platform_setup_timer();
	platform_setup_dma();
	platform_setup_gpio();
	platform_setup_interrupts();

	return;
}

void cleanup_platform()
{
	Xil_ICacheDisable();
	Xil_DCacheDisable();
	return;
}

u64 get_time_ms()
{
#define COUNTS_PER_MILLI_SECOND (COUNTS_PER_SECOND/1000)
	XTime t_cur = 0;
	XTime_GetTime(&t_cur);
	return (t_cur/COUNTS_PER_MILLI_SECOND);
}

