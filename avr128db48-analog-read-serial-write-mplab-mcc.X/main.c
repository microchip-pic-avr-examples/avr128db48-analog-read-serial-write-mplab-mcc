 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.0
*/

/*
� [2022] Microchip Technology Inc. and its subsidiaries.

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
#include "mcc_generated_files/system/system.h"

#define START_TOKEN 0x03    /* Start Frame Token */
#define END_TOKEN 0xFC      /* End Frame Token */
#define ADC_ACCUMULATION 4 /* Number of accumulations chosen for ADC */


void USART3_SendByte(const uint8_t data)
{
    while(!(USART3_IsTxReady()));		/* Wait until USART3 Data Register Empty */
    USART3_Write(data);						/* Send byte */
}

void USART3_send16bitDataStream( const uint16_t data)
{
	USART3_SendByte(START_TOKEN);			/* Send start token */
	USART3_SendByte(data & 0x00FF);			/* Send first 8 bits of measurement (low byte) */
	USART3_SendByte(data >> 8);					/* Send last 8 bits of measurement (high byte) */
	USART3_SendByte(END_TOKEN);				/* Send stop token */
}


/*
    Main application
*/

int main(void)
{
    SYSTEM_Initialize();
        
    while(1)
    {
		/* When variable is used, it will update value with latest ADC0 result */
		uint16_t measurement = ADC0_GetConversion(ADC_MUXPOS_AIN4_gc);	

		measurement /= ADC_ACCUMULATION;			/* Divide readout by number of accumulated result chosen */
		USART3_send16bitDataStream(measurement);	/* Send data stream out on USART3 */
    }
}
