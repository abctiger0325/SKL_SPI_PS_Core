/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include <stdlib.h>
#include "platform.h"
#include "xil_printf.h"
#include "sleep.h"
#include "string.h"
#include "xuartps.h"
#include "xil_exception.h"
#include "xplatform_info.h"

#ifdef XPAR_INTC_0_DEVICE_ID
#include "xintc.h"
#else
#include "xscugic.h"
#endif

//#define GPIOAddress  0x41200000
#define mem0Address  0x43C00000
#define mem1Address  0x43C10000
#define regOffset0  0
#define regOffset1  4
#define regOffset2  8
#define regOffset3	12
#define regOffset4	16
#define regOffset5	20

#define Reset_bit   0x01
#define DataIn_bit  0x02
#define RdyRece_bit 0x04
#define RdySend_bit 0x08
#define Done_bit    0x80

#define TEST_BUFFER_SIZE 5

#ifdef XPAR_INTC_0_DEVICE_ID
#define XUartPsHandler_IT		XIntc
#define UART_DEVICE_ID		XPAR_XUARTPS_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_INTC_0_DEVICE_ID
#define UART_INT_IRQ_ID		XPAR_INTC_0_UARTPS_0_VEC_ID
#else
#define XUartPsHandler_IT		XScuGic
//#define UART_DEVICE_ID		XPAR_XUARTPS_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define UART_INT_IRQ_ID		XPAR_XUARTPS_1_INTR
#endif


char printBuffer[64] = {0};
uint8_t regRead = 0;
uint32_t regVerify = 0;
// uint8_t LEDPattern = 0x04;
// uint8_t LEDMask = 0x5F;
uint8_t receBuff[12] = {0};
uint8_t cmdBuff[64] = {0};
uint8_t f_write = 0;
uint8_t f_read = 0;
uint8_t f_header = 0;
uint8_t headerBuff[3] = {0};
uint8_t r_StatusReg = 0;
uint8_t writeCnt = 0;
uint8_t receCounter = 0;


int PS_Uart_Init(XUartPsHandler_IT* xuart_it,XUartPs* xuart, uint16_t DeviceID, uint16_t UartIntID);
int PS_Uart_Receive_IT(XUartPsHandler_IT* xuart_it, XUartPs * xuart, uint16_t UartIntID);
void X_Uart_Interrupt_Handler(void *CallBackRef,uint32_t Event,unsigned int Data);
void X_Uart_ReceCplt_Handler(XUartPs* xuart);

XUartPs xuart1;
XUartPsHandler_IT xuart1_IT;

int receCnt, tranCnt, errCnt;

int PS_Uart_Init(XUartPsHandler_IT* xuart_it,XUartPs* xuart, uint16_t DeviceID, uint16_t UartIntID){
	uint8_t status;
	uint32_t IntrMask;
	XUartPs_Config *Config;

	#ifndef TESTAPP_GEN
		if (XGetPlatform_Info() == XPLAT_ZYNQ_ULTRA_MP) {
	#ifdef XPAR_XUARTPS_1_DEVICE_ID
			DeviceID = XPAR_XUARTPS_1_DEVICE_ID;
	#endif
		}
	#endif

	Config = XUartPs_LookupConfig(DeviceID);
	if (Config == NULL) {
		return XST_FAILURE;
	}

	status = XUartPs_CfgInitialize(xuart, Config, Config->BaseAddress);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Check hardware build */
	status = XUartPs_SelfTest(xuart);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the UART to the interrupt subsystem such that interrupts
	 * can occur. This function is application specific.
	 */
	status = PS_Uart_Receive_IT(xuart_it, xuart, UartIntID);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Setup the handlers for the UART that will be called from the
	 * interrupt context when data has been sent and received, specify
	 * a pointer to the UART driver instance as the callback reference
	 * so the handlers are able to access the instance data
	 */
	XUartPs_SetHandler(xuart, (XUartPs_Handler)X_Uart_Interrupt_Handler, xuart);

	/*
	 * Enable the interrupt of the UART so interrupts will occur, setup
	 * a local loopback so data that is sent will be received.
	 */
	IntrMask =
		XUARTPS_IXR_TOUT | XUARTPS_IXR_PARITY | XUARTPS_IXR_FRAMING |
		XUARTPS_IXR_OVER | XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_RXFULL |
		XUARTPS_IXR_RXOVR;

	if (xuart->Platform == XPLAT_ZYNQ_ULTRA_MP) {
		IntrMask |= XUARTPS_IXR_RBRK;
	}

	XUartPs_SetInterruptMask(xuart, IntrMask);

	XUartPs_SetOperMode(xuart, XUARTPS_OPER_MODE_NORMAL);

	XUartPs_SetRecvTimeout(xuart, 10);

	return XST_SUCCESS;
}

int PS_Uart_Receive_IT(XUartPsHandler_IT* xuart_it, XUartPs * xuart, uint16_t UartIntID){
		int Status;

	#ifdef XPAR_INTC_0_DEVICE_ID
	#ifndef TESTAPP_GEN
		/*
		* Initialize the interrupt controller driver so that it's ready to
		* use.
		*/
		Status = XIntc_Initialize(xuart_it, INTC_DEVICE_ID);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	#endif
		/*
		* Connect the handler that will be called when an interrupt
		* for the device occurs, the handler defined above performs the
		* specific interrupt processing for the device.
		*/
		Status = XIntc_Connect(xuart_it, UartIntID,
			(XInterruptHandler) XUartPs_InterruptHandler, xuart);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

	#ifndef TESTAPP_GEN
		/*
		* Start the interrupt controller so interrupts are enabled for all
		* devices that cause interrupts.
		*/
		Status = XIntc_Start(xuart_it, XIN_REAL_MODE);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	#endif
		/*
		* Enable the interrupt for uart
		*/
		XIntc_Enable(xuart_it, UartIntID);

		#ifndef TESTAPP_GEN
		/*
		* Initialize the exception table.
		*/
		Xil_ExceptionInit();

		/*
		* Register the interrupt controller handler with the exception table.
		*/
		Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
					(Xil_ExceptionHandler) XIntc_InterruptHandler,
					xuart_it);
		#endif
	#else
	#ifndef TESTAPP_GEN
		XScuGic_Config *IntcConfig; /* Config for interrupt controller */

		/* Initialize the interrupt controller driver */
		IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
		if (NULL == IntcConfig) {
			return XST_FAILURE;
		}

		Status = XScuGic_CfgInitialize(xuart_it, IntcConfig,
						IntcConfig->CpuBaseAddress);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		/*
		* Connect the interrupt controller interrupt handler to the
		* hardware interrupt handling logic in the processor.
		*/
		Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
					(Xil_ExceptionHandler) XScuGic_InterruptHandler,
					xuart_it);
	#endif

		/*
		* Connect a device driver handler that will be called when an
		* interrupt for the device occurs, the device driver handler
		* performs the specific interrupt processing for the device
		*/
		Status = XScuGic_Connect(xuart_it, UartIntID,
					(Xil_ExceptionHandler) XUartPs_InterruptHandler,
					(void *) xuart);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		/* Enable the interrupt for the device */
		XScuGic_Enable(xuart_it, UartIntID);

	#endif
	#ifndef TESTAPP_GEN
		/* Enable interrupts */
		Xil_ExceptionEnable();
	#endif

		return XST_SUCCESS;
}

void Cmd_Handler(uint8_t* config,uint8_t* list){
//	XUartPs_Send(&xuart1,config,3);
	if (config[1] == 0x00 || config[1] == 0x01){
		uint64_t memAddr;
        switch(config[0]){
			case 1:
				memAddr = mem0Address;
				break;

			case 'D':
			case 'd':
				memAddr = mem1Address;
				break;

			default:
				break;
		}
		for (int i = 0;i<headerBuff[2];i++){
			Xil_Out32(memAddr+regOffset1,list[i]);
			r_StatusReg |= DataIn_bit;
			Xil_Out32(memAddr+regOffset0,r_StatusReg);
			regVerify = Xil_In32(memAddr+regOffset0);
			while (regVerify & DataIn_bit){
				regVerify = Xil_In32(memAddr+regOffset0);
			}
	        r_StatusReg = regVerify;
		}

        r_StatusReg |= RdySend_bit;
        if (config[1] == 0x01)r_StatusReg |= RdyRece_bit;

        Xil_Out32(memAddr+regOffset0,r_StatusReg);
        regVerify = Xil_In32(memAddr+regOffset0);
        while (!(regVerify & Done_bit)){
            regVerify = Xil_In32(memAddr+regOffset0);
        }

        if (config[1] == 0x01){
        	regRead = Xil_In32(memAddr+regOffset2);
        	uint8_t r_Return[2];
        	r_Return[0] = 0x02;
        	r_Return[1] = regRead;
        	XUartPs_Send(&xuart1,r_Return,2);
        }

        regVerify &= 0x7B;
        r_StatusReg = regVerify;
        Xil_Out32(memAddr+regOffset0,r_StatusReg);
	} else if (config[1] == 0x03){
		Xil_Out32(mem1Address+regOffset4,list[0]);
	} else if (config[1] == 0x04){
		int r_Cnt = 0;
		r_Cnt = (uint16_t)(list[0] << 8) | list[1];
		Xil_Out32(mem1Address+regOffset5,r_Cnt);
	}
}

void X_Uart_ReceCplt_Handler(XUartPs * xuart){
	if (!f_header){
//		XUartPs_Send(&xuart1,(uint8_t*)"Ping",4);
		f_header = 1;
		receCounter = headerBuff[2];
		switch(headerBuff[0]){
			case 1:
				Xil_Out32(mem0Address+regOffset3,receCounter);
				break;

			case 'D':
				Xil_Out32(mem1Address+regOffset3,receCounter);
				break;

			case 'G':
			default:
				break;
		}
		XUartPs_Recv(&xuart1, receBuff, receCounter);
	} else {
//		XUartPs_Send(&xuart1,(uint8_t*)"Bong",4);
		f_header = 0;
		Cmd_Handler(headerBuff,receBuff);
		XUartPs_Recv(&xuart1, headerBuff, 3);
		receCounter = 3;
	}
	// LEDPattern = 0xFF;

}


void X_Uart_Interrupt_Handler(void *CallBackRef,uint32_t Event,unsigned int Data){
	switch(Event){
		case XUARTPS_EVENT_RECV_DATA:
//			XUartPs_Send(&xuart1,(uint8_t*)"Ding",4);
			if (Data == receCounter){
				X_Uart_ReceCplt_Handler(&xuart1);
			}
			break;

		case XUARTPS_EVENT_RECV_TOUT:
		case XUARTPS_EVENT_RECV_ERROR:
//			XUartPs_Send(&xuart1,(uint8_t*)"Dong",4);
			break;

		default:

			break;

	}
}

int main()
{
    init_platform();
    PS_Uart_Init(&xuart1_IT,&xuart1,UART_DEVICE_ID,UART_INT_IRQ_ID);
	XUartPs_Recv(&xuart1, headerBuff, 3);
	receCounter = 3;
    Xil_Out32(mem0Address+regOffset3,3);

    // regVerify = Xil_In32(mem0Address+regOffset2);
    // if (regVerify == 0x2345)
    // 	sprintf(printBuffer,"Verify Done Value: %04x\n\r",(unsigned int)regVerify);
    // else
    // 	sprintf(printBuffer,"Verify Fail Value: %04x\n\r",(unsigned int)regVerify);
    // XUartPs_Send(&xuart1,(uint8_t*)printBuffer,sizeof(printBuffer));

    while(1){
    	// regValue = LEDPattern;
    	// regValue |= LEDMask << 8;

    	// Xil_Out32(mem0Address+regOffset0,regValue);
    	// regVerify = Xil_In32(mem0Address+regOffset1);

    	// LEDPattern = LEDPattern << 1;
    	// if (LEDPattern == 0) LEDPattern = 0x4;
    	sleep(1);
    }
    cleanup_platform();
    return 0;
}
