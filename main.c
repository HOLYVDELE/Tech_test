/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PSoC 6 MCU: Hello World Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2019-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyhal_gpio.h"
#include "stdlib.h"
#include "string.h"





/*******************************************************************************
* Macros
*******************************************************************************/

/* PWM Frequency = 2Hz */
#define PWM_FREQUENCY (2u)
/* PWM Duty-cycle = 50% */
#define PWM_DUTY_CYCLE (50.0f)
#define GPIO_INTERRUPT_PRIORITY (7u)



/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
void btn_itrp_init(void);
void led_blink(void);
void led_off(void);
void led_on(void);
void pwm_init(void);


/*******************************************************************************
* Global Variables
*******************************************************************************/
bool timer_interrupt_flag = false;
bool led_blink_active_flag = true;
/* Variable for storing character read from terminal */
char uart_read_value_buff[64];
volatile bool isButt = false;
/* PWM object used for blinking the LED */
cyhal_pwm_t pwm_led_control;
size_t read_len = 0;

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It sets up a timer to trigger a 
* periodic interrupt. The main while loop checks for the status of a flag set 
* by the interrupt and toggles an LED at 1Hz to create an LED blinky. The 
* while loop also checks whether the 'Enter' key was pressed and 
* stops/restarts LED blinking.
*
* Parameters:
*  none
*
* Return:
*  int
*
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                 CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

       /* Initialize the user button */
         result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                         CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    //


    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("HELLO WORD\n");
    printf("frequensy is 0.5hz\r\n");
    /* Initialize timer to toggle the LED */
    pwm_init();
    btn_itrp_init();




    for (;;)
    {
    	cyhal_system_delay_ms(100);
    	read_len = cyhal_uart_readable(&cy_retarget_io_uart_obj);

       if ( read_len == 0)
       {
    	   continue;
       }
       memset(uart_read_value_buff, 0, 64);


        if (cyhal_uart_read(&cy_retarget_io_uart_obj, uart_read_value_buff, &read_len)
             == CY_RSLT_SUCCESS)
        {

        	char* token = strtok(uart_read_value_buff,"=");

            if (strcmp("blink",uart_read_value_buff) == 0 )
            {

            	led_blink();

            }
            else if(strcmp("on",uart_read_value_buff) == 0 )
            {

            	led_on();
            }
            else if(strcmp("off",uart_read_value_buff) == 0 )
            {

                led_off();
            }

            else if (strcmp("hz",token) == 0 )
            {

            	if(token != NULL)
            	{
            			token = strtok(NULL,"=");
            			cyhal_pwm_set_duty_cycle(&pwm_led_control, 50.0f,(uint32_t)atoi(token));
            	}
            }


        }

    }
}


//Turn led ON
void led_on(void)
{
	printf("LED turned on\r\n");
	cyhal_pwm_set_duty_cycle(&pwm_led_control, 0.0f, 1.0f);
	cyhal_pwm_start(&pwm_led_control);
	led_blink_active_flag = true;
}
//Turn led OFF//
void led_off(void)
{
	printf("LED turned off\r\n");
	cyhal_pwm_stop(&pwm_led_control);
	led_blink_active_flag = true;
}
//Turn led in blinking mode//
void led_blink(void)
{
	 //ON Blink mode
	                if (led_blink_active_flag)
	                {

	                cyhal_pwm_start(&pwm_led_control);

	                 led_blink_active_flag = false;



	                }
	   //OFF Blink mode
	                else if(!led_blink_active_flag)
	                {
	                	cyhal_pwm_stop(&pwm_led_control);
	                	led_blink_active_flag = true;

	                }


	                /* Move cursor to previous line */
	                printf("\x1b[1F");

}



void btn_itrp_init (void)
{

	  /* Initialize the user button */
	   cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
	                    CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);

	    /* Configure GPIO interrupt */
	   cyhal_gpio_callback_data_t intr_callback = {gpio_interrupt_handler, NULL, NULL, CYBSP_USER_BTN };
	   cyhal_gpio_register_callback(CYBSP_USER_BTN, &intr_callback);

	    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
	                                 GPIO_INTERRUPT_PRIORITY, true);
}
void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{

    if(!isButt)
          {

    		cyhal_pwm_set_duty_cycle(&pwm_led_control, 80.0f, 2);
    		cyhal_pwm_start(&pwm_led_control);
    		char msg_wr[] = "\n\rThat device set to default value\n\r";
    		size_t size_wr = sizeof(msg_wr);
    		cyhal_uart_write(&cy_retarget_io_uart_obj, msg_wr, &size_wr);
    		isButt = true;

           }
          else if(isButt)
            {
        	 cyhal_pwm_stop(&pwm_led_control);

             	 isButt = true;
             }
}
void pwm_init(void)
{
	cy_rslt_t result;

	 result = cyhal_pwm_init(&pwm_led_control, CYBSP_USER_LED, NULL);
	    if(CY_RSLT_SUCCESS != result)
	    {
	        CY_ASSERT(false);
	    }
	    cyhal_pwm_stop(&pwm_led_control);



	    result = cyhal_pwm_set_duty_cycle(&pwm_led_control, PWM_DUTY_CYCLE, PWM_FREQUENCY);
	        if(CY_RSLT_SUCCESS != result)
	        {
	            CY_ASSERT(false);
	        }

}



/* [] END OF FILE */
