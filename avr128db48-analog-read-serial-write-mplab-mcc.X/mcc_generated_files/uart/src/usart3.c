/**
  @Company
    Microchip Technology Inc.

  @Description
    This Source file provides APIs.
    Generation Information :
    Driver Version    :   1.0.0
*/
/*
Copyright (c) [2012-2020] Microchip Technology Inc.  

    All rights reserved.

    You are permitted to use the accompanying software and its derivatives 
    with Microchip products. See the Microchip license agreement accompanying 
    this software, if any, for additional info regarding your rights and 
    obligations.
    
    MICROCHIP SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT 
    WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT 
    LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT 
    AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP OR ITS
    LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT 
    LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE 
    THEORY FOR ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT 
    LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES, 
    OR OTHER SIMILAR COSTS. 
    
    To the fullest extend allowed by law, Microchip and its licensors 
    liability will not exceed the amount of fees, if any, that you paid 
    directly to Microchip to use this software. 
    
    THIRD PARTY SOFTWARE:  Notwithstanding anything to the contrary, any 
    third party software accompanying this software is subject to the terms 
    and conditions of the third party's license agreement.  To the extent 
    required by third party licenses covering such third party software, 
    the terms of such license will apply in lieu of the terms provided in 
    this notice or applicable license.  To the extent the terms of such 
    third party licenses prohibit any of the restrictions described here, 
    such restrictions will not apply to such third party software.
*/


#include "../usart3.h"
#define RECEIVE_ERROR_MASK 0x46


static void DefaultFramingErrorCallback(void);
static void DefaultOverrunErrorCallback(void);
static void DefaultParityErrorCallback(void);
void (*USART3_framing_err_cb)(void) = &DefaultFramingErrorCallback;
void (*USART3_overrun_err_cb)(void) = &DefaultOverrunErrorCallback;
void (*USART3_parity_err_cb)(void) = &DefaultParityErrorCallback;

const struct UART_INTERFACE USART3_Interface = {
  .Initialize = USART3_Initialize,
  .Write = USART3_Write,
  .Read = USART3_Read,
  .RxCompleteCallbackRegister = NULL,
  .TxCompleteCallbackRegister = NULL,
  .ErrorCallbackRegister = NULL,
  .FramingErrorCallbackRegister = USART3_FramingErrorCallbackRegister,
  .OverrunErrorCallbackRegister = USART3_OverrunErrorCallbackRegister,
  .ParityErrorCallbackRegister = USART3_ParityErrorCallbackRegister,
  .ChecksumErrorCallbackRegister = NULL,
  .IsRxReady = USART3_IsRxReady,
  .IsTxReady = USART3_IsTxReady,
  .IsTxDone = USART3_IsTxDone
};

void USART3_Initialize(void)
{
    //set baud rate register
    USART3.BAUD = (uint16_t)USART3_BAUD_RATE(9600);
	
    // ABEIE disabled; DREIE disabled; LBME disabled; RS485 DISABLE; RXCIE disabled; RXSIE disabled; TXCIE disabled; 
    USART3.CTRLA = 0x0;
	
    // MPCM disabled; ODME disabled; RXEN enabled; RXMODE NORMAL; SFDEN disabled; TXEN enabled; 
    USART3.CTRLB = 0xC0;
	
    // CMODE Asynchronous Mode; UCPHA enabled; UDORD disabled; CHSIZE Character size: 8 bit; PMODE No Parity; SBMODE 1 stop bit; 
    USART3.CTRLC = 0x3;
	
    //DBGCTRL_DBGRUN
    USART3.DBGCTRL = 0x0;
	
    //EVCTRL_IREI
    USART3.EVCTRL = 0x0;
	
    //RXPLCTRL_RXPL
    USART3.RXPLCTRL = 0x0;
	
    //TXPLCTRL_TXPL
    USART3.TXPLCTRL = 0x0;
	

}

void USART3_FramingErrorCallbackRegister(void* callbackHandler)
{
    USART3_framing_err_cb=callbackHandler;
}

void USART3_OverrunErrorCallbackRegister(void* callbackHandler)
{
    USART3_overrun_err_cb=callbackHandler;
}

void USART3_ParityErrorCallbackRegister(void* callbackHandler)
{
    USART3_parity_err_cb=callbackHandler;
}

static void DefaultFramingErrorCallback(void)
{
    /* Add your interrupt code here or use USART3.FramingCallbackRegister function to use Custom ISR */
}

static void DefaultOverrunErrorCallback(void)
{
   /* Add your interrupt code here or use USART3.OverrunCallbackRegister function to use Custom ISR */
}

static void DefaultParityErrorCallback(void)
{
    /* Add your interrupt code here or use USART3.ParityCallbackRegister function to use Custom ISR */
}

void USART3_Enable(void)
{
    USART3.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
}

void USART3_EnableRx(void)
{
    USART3.CTRLB |= USART_RXEN_bm;
}

void USART3_EnableTx(void)
{
    USART3.CTRLB |= USART_TXEN_bm;
}

void USART3_Disable(void)
{
    USART3.CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);
}

uint8_t USART3_GetData(void)
{
    return USART3.RXDATAL;
}

bool USART3_IsTxReady(void)
{
    return (USART3.STATUS & USART_DREIF_bm);
}

bool USART3_IsRxReady(void)
{
    return (USART3.STATUS & USART_RXCIF_bm);
}

bool USART3_IsTxBusy(void)
{
    return (!(USART3.STATUS & USART_TXCIF_bm));
}

bool USART3_IsTxDone(void)
{
    return (USART3.STATUS & USART_TXCIF_bm);
}

void USART3_ErrorCheck(void)
{
    uint8_t errorMask = USART3.RXDATAH;
    if(errorMask & RECEIVE_ERROR_MASK)
    {
        if(errorMask & USART_PERR_bm)
        {
            USART3_parity_err_cb();
        } 
        if(errorMask & USART_FERR_bm)
        {
            USART3_framing_err_cb();
        }
        if(errorMask & USART_BUFOVF_bm)
        {
            USART3_overrun_err_cb();
        }
    }
    
}

uint8_t USART3_Read(void)
{       
    return USART3.RXDATAL;
}

void USART3_Write(const uint8_t data)
{
    USART3.TXDATAL = data;
}