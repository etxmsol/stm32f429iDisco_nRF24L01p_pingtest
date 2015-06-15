#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f429i_discovery.h"
#include <stdlib.h>


#define SPIx                             SPI4
#define SPIx_CLK_ENABLE()                __SPI4_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __GPIOE_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __GPIOE_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __GPIOE_CLK_ENABLE()
#define GPIO_CS_CE_CLK_ENABLE()			 __GPIOA_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __SPI4_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __SPI4_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_2
#define SPIx_SCK_GPIO_PORT               GPIOE
#define SPIx_SCK_AF                      GPIO_AF5_SPI4
#define SPIx_MISO_PIN                    GPIO_PIN_5
#define SPIx_MISO_GPIO_PORT              GPIOE
#define SPIx_MISO_AF                     GPIO_AF5_SPI4
#define SPIx_MOSI_PIN                    GPIO_PIN_6
#define SPIx_MOSI_GPIO_PORT              GPIOE
#define SPIx_MOSI_AF                     GPIO_AF5_SPI4

/* Definition of CSN and CE Pins */
#define GPIO_CS_CE                  	 GPIOA
#define GPIO_Pin_CE              		 GPIO_PIN_3
#define GPIO_Pin_CS              		 GPIO_PIN_7

/* Definition for SPIx's NVIC */
#define SPIx_IRQn                        SPI4_IRQn
#define SPIx_IRQHandler                  SPI4_IRQHandler

/* Size of buffer */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

