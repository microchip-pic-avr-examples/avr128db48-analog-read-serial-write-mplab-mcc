/**
 * USART3 Generated Driver API Header File
 * 
 * @file usart3.c
 * 
 * @ingroup usart3
 * 
 * @brief This is the generated driver implementation file for the USART3 driver using 
 *
 * @version USART3 Driver Version 2.0.3
*/

/*
© [2022] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

/**
  Section: Included Files
*/

#include "../usart3.h"

/**
  Section: Macro Declarations
*/



/**
  Section: Driver Interface
 */

const uart_drv_interface_t UART3 = {
    .Initialize = &USART3_Initialize,
    .Deinitialize = &USART3_Deinitialize,
    .Read = &USART3_Read,
    .Write = &USART3_Write,
    .IsRxReady = &USART3_IsRxReady,
    .IsTxReady = &USART3_IsTxReady,
    .IsTxDone = &USART3_IsTxDone,
    .TransmitEnable = &USART3_TransmitEnable,
    .TransmitDisable = &USART3_TransmitDisable,
    .AutoBaudSet = &USART3_AutoBaudSet,
    .AutoBaudQuery = &USART3_AutoBaudQuery,
    .BRGCountSet = NULL,
    .BRGCountGet = NULL,
    .BaudRateSet = NULL,
    .BaudRateGet = NULL,
    .AutoBaudEventEnableGet = NULL,
    .ErrorGet = &USART3_ErrorGet,
    .TxCompleteCallbackRegister = NULL,
    .RxCompleteCallbackRegister = NULL,
    .TxCollisionCallbackRegister = NULL,
    .FramingErrorCallbackRegister = &USART3_FramingErrorCallbackRegister,
    .OverrunErrorCallbackRegister = &USART3_OverrunErrorCallbackRegister,
    .ParityErrorCallbackRegister = &USART3_ParityErrorCallbackRegister,
    .EventCallbackRegister = NULL,
};

/**
  Section: USART3 variables
*/
static volatile usart3_status_t usart3RxLastError;

/**
  Section: USART3 APIs
*/
void (*USART3_FramingErrorHandler)(void);
void (*USART3_OverrunErrorHandler)(void);
void (*USART3_ParityErrorHandler)(void);

static void USART3_DefaultFramingErrorCallback(void);
static void USART3_DefaultOverrunErrorCallback(void);
static void USART3_DefaultParityErrorCallback(void);



/**
  Section: USART3  APIs
*/

#if defined(__GNUC__)

int USART3_printCHAR(char character, FILE *stream)
{
    while(!(USART3_IsTxReady()));
    USART3_Write(character);
    return 0;
}

FILE USART3_stream = FDEV_SETUP_STREAM(USART3_printCHAR, NULL, _FDEV_SETUP_WRITE);

#elif defined(__ICCAVR__)

int putchar (int outChar)
{
    while(!(USART3_IsTxReady()));
    USART3_Write(outChar);
    return outChar;
}
#endif

void USART3_Initialize(void)
{
    // Set the USART3 module to the options selected in the user interface.

    //BAUD 1666; 
    USART3.BAUD = (uint16_t)USART3_BAUD_RATE(9600);
	
    // ABEIE disabled; DREIE disabled; LBME disabled; RS485 DISABLE; RXCIE disabled; RXSIE disabled; TXCIE disabled; 
    USART3.CTRLA = 0x0;
	
    // MPCM disabled; ODME disabled; RXEN enabled; RXMODE NORMAL; SFDEN disabled; TXEN enabled; 
    USART3.CTRLB = 0xC0;
	
    // CMODE Asynchronous Mode; UCPHA enabled; UDORD disabled; CHSIZE Character size: 8 bit; PMODE No Parity; SBMODE 1 stop bit; 
    USART3.CTRLC = 0x3;
	
    //DBGRUN disabled; 
    USART3.DBGCTRL = 0x0;
	
    //IREI disabled; 
    USART3.EVCTRL = 0x0;
	
    //RXPL 0x0; 
    USART3.RXPLCTRL = 0x0;
	
    //TXPL 0x0; 
    USART3.TXPLCTRL = 0x0;
	
    USART3_FramingErrorCallbackRegister(USART3_DefaultFramingErrorCallback);
    USART3_OverrunErrorCallbackRegister(USART3_DefaultOverrunErrorCallback);
    USART3_ParityErrorCallbackRegister(USART3_DefaultParityErrorCallback);
    usart3RxLastError.status = 0;  
#if defined(__GNUC__)
    stdout = &USART3_stream;
#endif
}

void USART3_Deinitialize(void)
{
    USART3.BAUD = 0x00;	
    USART3.CTRLA = 0x00;	
    USART3.CTRLB = 0x00;	
    USART3.CTRLC = 0x00;	
    USART3.DBGCTRL = 0x00;	
    USART3.EVCTRL = 0x00;	
    USART3.RXPLCTRL = 0x00;	
    USART3.TXPLCTRL = 0x00;	
}

void USART3_Enable(void)
{
    USART3.CTRLB |= USART_RXEN_bm | USART_TXEN_bm; 
}

void USART3_Disable(void)
{
    USART3.CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm); 
}

void USART3_TransmitEnable(void)
{
    USART3.CTRLB |= USART_TXEN_bm; 
}

void USART3_TransmitDisable(void)
{
    USART3.CTRLB &= ~(USART_TXEN_bm); 
}

void USART3_ReceiveEnable(void)
{
    USART3.CTRLB |= USART_RXEN_bm ; 
}

void USART3_ReceiveDisable(void)
{
    USART3.CTRLB &= ~(USART_RXEN_bm); 
}

void USART3_AutoBaudSet(bool enable)
{
    if(enable)
    {
        USART3.CTRLB |= USART_RXMODE_gm & (0x02 << USART_RXMODE_gp); 
        USART3.STATUS |= USART_WFB_bm ; 
    }
    else
    {
       USART3.CTRLB &= ~(USART_RXMODE_gm); 
       USART3.STATUS &= ~(USART_BDF_bm);  
    }
}

bool USART3_AutoBaudQuery(void)
{
     return (bool)(USART3.STATUS & USART_BDF_bm) ; 
}

bool USART3_IsAutoBaudDetectError(void)
{
     return (bool)(USART3.STATUS & USART_ISFIF_bm) ; 
}

void USART3_AutoBaudDetectErrorReset(void)
{
    USART3.STATUS |= USART_ISFIF_bm ;
	USART3_AutoBaudSet(false);
    USART3_ReceiveDisable();
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    USART3_ReceiveEnable();
    USART3_AutoBaudSet(true);
}

bool USART3_IsRxReady(void)
{
    return (bool)(USART3.STATUS & USART_RXCIF_bm);
}

bool USART3_IsTxReady(void)
{
    return (bool)(USART3.STATUS & USART_DREIF_bm);
}

bool USART3_IsTxDone(void)
{
    return (bool)(USART3.STATUS & USART_TXCIF_bm);
}

size_t USART3_ErrorGet(void)
{
    usart3RxLastError.status = 0;
    
    if(USART3.RXDATAH & USART_FERR_bm)
    {
        usart3RxLastError.ferr = 1;
        if(NULL != USART3_FramingErrorHandler)
        {
            USART3_FramingErrorHandler();
        }  
    }
    if(USART3.RXDATAH & USART_PERR_bm)
    {
        usart3RxLastError.perr = 1;
        if(NULL != USART3_ParityErrorHandler)
        {
            USART3_ParityErrorHandler();
        }  
    }
    if(USART3.RXDATAH & USART_BUFOVF_bm)
    {
        usart3RxLastError.oerr = 1;
        if(NULL != USART3_OverrunErrorHandler)
        {
            USART3_OverrunErrorHandler();
        }   
    }
    return usart3RxLastError.status;
}

uint8_t USART3_Read(void)
{
    return USART3.RXDATAL;
}


void USART3_Write(uint8_t txData)
{
    USART3.TXDATAL = txData;    // Write the data byte to the USART.
}
static void USART3_DefaultFramingErrorCallback(void)
{
    
}

static void USART3_DefaultOverrunErrorCallback(void)
{
    
}

static void USART3_DefaultParityErrorCallback(void)
{
    
}

void USART3_FramingErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        USART3_FramingErrorHandler = callbackHandler;
    }
}

void USART3_OverrunErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        USART3_OverrunErrorHandler = callbackHandler;
    }    
}

void USART3_ParityErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        USART3_ParityErrorHandler = callbackHandler;
    } 
}




