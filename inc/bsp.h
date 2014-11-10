
/**
  ******************************************************************************
  * @file    bsp.c
  * @author  Xue Liu
  * @version V0.1
  * @date    10-15-2014
  * @brief   borad support package
  ******************************************************************************
  */
#ifndef SYS_INIT_H_
#define SYS_INIT_H_
  
/**
******************************************************************************
* Type definations
******************************************************************************
*/

#define LEDn 2

#define LED_RED_PIN GPIO_Pin_4
#define LED_RED_GPIO_PORT GPIOB
#define LED_RED_GPIO_CLK RCC_APB2Periph_GPIOB

#define LED_GREEN_PIN GPIO_Pin_5
#define LED_GREEN_GPIO_PORT GPIOB
#define LED_GREEN_GPIO_CLK RCC_APB2Periph_GPIOB

typedef enum { LED_RED = 0, LED_GREEN = 1 } Led_TypeDef;

/**
  ******************************************************************************
  * Public Declarations
  ******************************************************************************
  */

/*
 * MySystemInit is used to set default system initializations and configure system * clock to 72MHz.
*/
void init_system_clk(void);

/*
 * Initializes pin PB5 which is connected to the green LED. 
 * Port mode is push-pull output.
 */
void init_blink(void);

/*
 * Initializes pins connected to motors and configures respective timers 
 * to output PWM signals.
 */
void init_motors(void);


/**
  ******************************************************************************
  * End of the file
  ******************************************************************************
  */
#endif
