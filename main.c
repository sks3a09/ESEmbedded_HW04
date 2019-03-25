#include <stdint.h>
#include "blink.h"
#include "reg.h"

int main(void)
{
	/**
     * 
     * BUTTON init
     * 
     */
	SET_BIT(RCC_BASE + RCC_AHB1ENR_OFFSET, GPIO_EN_BIT(GPIO_PORTA));

	//MODER led pin = 00 => input mode
	CLEAR_BIT(GPIO_BASE(GPIO_PORTA) + GPIOx_MODER_OFFSET, MODERy_1_BIT(BUTTON));
	CLEAR_BIT(GPIO_BASE(GPIO_PORTA) + GPIOx_MODER_OFFSET, MODERy_0_BIT(BUTTON));

	//OT led pin = x 

	//OSPEEDR led pin = xx 
	
	//PUPDR led pin = 00 => No pull-up, pull-down
	CLEAR_BIT(GPIO_BASE(GPIO_PORTD) + GPIOx_PUPDR_OFFSET, PUPDRy_1_BIT(BUTTON));
	CLEAR_BIT(GPIO_BASE(GPIO_PORTD) + GPIOx_PUPDR_OFFSET, PUPDRy_0_BIT(BUTTON));

	while(1){
		if(READ_BIT(GPIO_BASE(GPIO_PORTA) + GPIOx_IDR_OFFSET,IDRy_BIT(BUTTON))){
			blink(LED_BLUE);
		}
		else{
			
		}
	}
	
}
