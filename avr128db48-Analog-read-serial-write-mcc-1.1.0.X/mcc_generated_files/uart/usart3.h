/**
 * USART3 Generated Driver API Header File
 * 
 * @file usart3.h
 * 
 * @defgroup usart3 USART3
 * 
 * @brief This file contains API prototypes and other datatypes for USART3 module.
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

#ifndef USART3_H
#define USART3_H

/**
  Section: Included Files
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "../system/system.h"
#include "uart_drv_interface.h"

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

/* Normal Mode, Baud register value */
#define USART3_BAUD_RATE(BAUD_RATE) (((float)4000000 * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#define UART3_interface UART3


#define UART3_Initialize     USART3_Initialize
#define UART3_Deinitialize   USART3_Deinitialize
#define UART3_Write          USART3_Write
#define UART3_Read           USART3_Read
#define UART3__IsRxReady     USART3_IsRxReady
#define UART3_IsTxReady      USART3_IsTxReady
#define UART3_IsTxDone       USART3_IsTxDone

#define UART3_TransmitEnable       USART3_TransmitEnable
#define UART3_TransmitDisable      USART3_TransmitDisable
#define UART3_AutoBaudSet          USART3_AutoBaudSet
#define UART3_AutoBaudQuery        USART3_AutoBaudQuery
#define UART3_BRGCountSet               (NULL)
#define UART3_BRGCountGet               (NULL)
#define UART3_BaudRateSet               (NULL)
#define UART3_BaudRateGet               (NULL)
#define UART3__AutoBaudEventEnableGet   (NULL)
#define UART3_ErrorGet             USART3_ErrorGet

#define UART3_TxCompleteCallbackRegister     (NULL)
#define UART3_RxCompleteCallbackRegister      (NULL)
#define UART3_TxCollisionCallbackRegister  (NULL)
#define UART3_FramingErrorCallbackRegister USART3_FramingErrorCallbackRegister
#define UART3_OverrunErrorCallbackRegister USART3_OverrunErrorCallbackRegister
#define UART3_ParityErrorCallbackRegister  USART3_ParityErrorCallbackRegister
#define UART3_EventCallbackRegister        (NULL)


/**
 @ingroup usart3
 @struct usart3_status_t
 @breif This is an instance of USART3_STATUS for USART3 module
 */
typedef union {
    struct {
        uint8_t perr : 1;     /**<This is a bit field for Parity Error status*/
        uint8_t ferr : 1;     /**<This is a bit field for Framing Error status*/
        uint8_t oerr : 1;     /**<This is a bit field for Overfrun Error status*/
        uint8_t reserved : 5; /**<Reserved*/
    };
    size_t status;            /**<Group byte for status errors*/
}usart3_status_t;



/**
 Section: Data Type Definitions
 */

/**
 * @ingroup usart3
 * @brief External object for usart3_interface.
 */
extern const uart_drv_interface_t UART3;

/**
 * @ingroup usart3
 * @brief This API initializes the USART3 driver.
 *        This routine initializes the USART3 module.
 *        This routine must be called before any other USART3 routine is called.
 *        This routine should only be called once during system initialization.
 * @param None.
 * @return None.
 */
void USART3_Initialize(void);

/**
 * @ingroup usart3
 * @brief This API Deinitializes the USART3 driver.
 *        This routine disables the USART3 module.
 * @param None.
 * @return None.
 */
void USART3_Deinitialize(void);

/**
 * @ingroup usart3
 * @brief This API enables the USART3 module.     
 * @param None.
 * @return None.
 */
void USART3_Enable(void);

/**
 * @ingroup usart3
 * @brief This API disables the USART3 module.
 * @param None.
 * @return None.
 */
void USART3_Disable(void);

/**
 * @ingroup usart3
 * @brief This API enables the USART3 transmitter.
 *        USART3 should also be enable to send bytes over TX pin.
 * @param None.
 * @return None.
 */
void USART3_TransmitEnable(void);

/**
 * @ingroup usart3
 * @brief This API disables the USART3 transmitter.
 * @param None.
 * @return None.
 */
void USART3_TransmitDisable(void);

/**
 * @ingroup usart3
 * @brief This API enables the USART3 Receiver.
 *        USART3 should also be enable to receive bytes over RX pin.
 * @param None.
 * @return None.
 */
void USART3_ReceiveEnable(void);

/**
 * @ingroup usart3
 * @brief This API disables the USART3 Receiver.
 * @param None.
 * @return None.
 */
void USART3_ReceiveDisable(void);



/**
 * @ingroup usart3
 * @brief This API enables the USART3 AutoBaud Detection.
 * @param bool enable.
 * @return None.
 */
void USART3_AutoBaudSet(bool enable);

/**
 * @ingroup usart3
 * @brief This API reads the USART3 AutoBaud Detection Complete bit.
 * @param None.
 * @return None.
 */
bool USART3_AutoBaudQuery(void);

/**
 * @ingroup usart3
 * @brief This API reads the USART3 AutoBaud Detection error bit.
 * @param None.
 * @return None.
 */
bool USART3_IsAutoBaudDetectError(void);

/**
 * @ingroup usart3
 * @brief This API Reset the USART3 AutoBaud Detection error bit.
 * @param None.
 * @return None.
 */
void USART3_AutoBaudDetectErrorReset(void);

/**
 * @ingroup usart3
 * @brief This API checks if USART3 receiver has received data and ready to be read.
 * @param None.
 * @retval true if USART3 receiver FIFO has a data
 * @retval false USART3 receiver FIFO is empty
 */
bool USART3_IsRxReady(void);

/**
 * @ingroup usart3
 * @brief This function checks if USART3 transmitter is ready to accept a data byte.
 * @param None.
 * @retval true if USART3 transmitter FIFO has atleast 1 byte space
 * @retval false if USART3 transmitter FIFO is full
 */
bool USART3_IsTxReady(void);

/**
 * @ingroup usart3
 * @brief This function return the status of transmit shift register (TSR).
 * @param None.
 * @retval true if Data completely shifted out from the TSR
 * @retval false if Data is present in Transmit FIFO and/or in TSR
 */
bool USART3_IsTxDone(void);

/**
 * @ingroup usart3
 * @brief This function gets the error status of the last read byte.
 *        This function should be called before USART3_Read().
 * @param None.
 * @return Status of the last read byte. See usart3_status_t struct for more details.
 */
size_t USART3_ErrorGet(void);

/**
 * @ingroup usart3
 * @brief This function reads the 8 bits from receiver FIFO register.
 * @pre The transfer status should be checked to see if the receiver is not empty
 *      before calling this function. USART3_IsRxReady() should be checked in if () before calling this API.
 * @param None.
 * @return 8-bit data from RX FIFO register.
 */
uint8_t USART3_Read(void);

/**
 * @ingroup usart3
 * @brief This function writes a byte of data to the transmitter FIFO register.
 * @pre The transfer status should be checked to see if the transmitter is ready to accept a byte
 *      before calling this function. USART3_IsTxReady() should be checked in if() before calling this API.
 * @param txData  - Data byte to write to the TX FIFO.
 * @return None.
 */
void USART3_Write(uint8_t txData);

/**
 * @ingroup usart3
 * @brief This API registers the function to be called upon USART3 framing error.
 * @param callbackHandler - a function pointer which will be called upon framing error condition.
 * @return None.
 */
void USART3_FramingErrorCallbackRegister(void (* callbackHandler)(void));

/**
 * @ingroup usart3
 * @brief This API registers the function to be called upon USART3 overrun error.
 * @param callbackHandler - a function pointer which will be called upon overrun error condition.
 * @return None.
 */
void USART3_OverrunErrorCallbackRegister(void (* callbackHandler)(void));

/**
 * @ingroup usart3
 * @brief This API registers the function to be called upon USART3 Parity error.
 * @param callbackHandler - a function pointer which will be called upon Parity error condition.
 * @return None.
 */
void USART3_ParityErrorCallbackRegister(void (* callbackHandler)(void));


#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif  // USART3_H
