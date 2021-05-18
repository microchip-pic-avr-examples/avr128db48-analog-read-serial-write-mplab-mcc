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
#include "mcc_generated_files/system/system.h"

#define START_TOKEN 0x03			/* Data stream Start of Frame Token */
#define END_TOKEN (~START_TOKEN)	/* Data Stream End of Frame Token */
#define ADC_ACCUMULATION 4			/* Number of accumulations chosen for ADC */

void USART3_SendByte(const uint8_t data)
{
    while(!(USART3_IsTxReady()));	/* Wait until USART3 Data Register Empty */
    USART3_Write(data);				/* Send byte */
}

/* This function sends data stream <START_TOKEN + measurement data (16 bits) + END TOKEN> over USART 3 */
void USART3_send16bitDataStream( const uint16_t data)
{
	USART3_SendByte(START_TOKEN);		/* Send data stream start of frame token */
	USART3_SendByte(data & 0xFF);		/* Send first 8 bits of measurement (low byte) */
	USART3_SendByte(data >> 8);			/* Send last 8 bits of measurement (high byte) */
	USART3_SendByte(END_TOKEN);			/* Send data stream stop of frame token */
}

int main(void)
{
    SYSTEM_Initialize();
        
    while(1)
    {
		/* Do analog measurement and divide by number of accumulations */
		uint16_t measurement = ADC0_GetConversion(ADC_MUXPOS_AIN4_gc);	
		measurement /= ADC_ACCUMULATION;
		
		USART3_send16bitDataStream(measurement);	/* Send data stream out on USART3 */
    }
}