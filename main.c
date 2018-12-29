#include <stdio.h>
#include <string.h>
// #include <stm32f10x_usart.h>

#include "main.h"

#define DIM 4

int mapping[DIM][DIM] = {
	{0x1b,0x1a,0x11,0x10},
	{0x07,0x06,0x05,0x04},
	{0x03,0x02,0x01,0x00},
	{0x13,0x0f,0x0c,0x0b}
};

int mapping_z[DIM] = {0x14,0x15,0x16,0x17};

void delay(int millis) {
    while (millis-- > 0) {
        volatile int x = 5971;
        while (x-- > 0) {
            __asm("nop");
        }
    }
}

uint16_t toPin(int number) {
	return 1<<(number %16);
}

GPIO_TypeDef *toPort(int number) {
	switch(number >> 4) {
	case 0x00:
		return GPIOA;
	case 0x01:
		return GPIOB;
	case 0x02:
		return GPIOC;
	case 0x03:
		return GPIOD;
	}
	return 0;
}

/**********************************************************
 * USART1 interrupt request handler: on reception of a 
 * character 't', toggle LED and transmit a character 'T'
 *********************************************************/
void USART1_IRQHandler(void)
{
    /* RXNE handler */
    /*if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        // If received 't', toggle LED and transmit 'T'
        if((char)USART_ReceiveData(USART1) == 't')
        {
            //led_toggle();
            //USART_SendData(USART1, 'T');
            /* Wait until Tx data register is empty, not really 
             * required for this example but put in here anyway.
             */
            /*
            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
            {
            }
        }
    }
     */
    /* ------------------------------------------------------------ */
    /* Other USART1 interrupts handler can go here ...             */
} 

/*****************************************************
 * Initialize USART1: enable interrupt on reception
 * of a character
 *****************************************************/
/*void USART1_Init(void)
{
    // USART configuration structure for USART1
    USART_InitTypeDef usart1_init_struct;
    // Bit configuration structure for GPIOA PIN9 and PIN10
    GPIO_InitTypeDef gpioa_init_struct;
     
    // Enalbe clock for USART1, AFIO and GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_GPIOA, ENABLE);
                            
    // GPIOA PIN9 alternative function Tx
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_9;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpioa_init_struct);
    // GPIOA PIN9 alternative function Rx 
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_10;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpioa_init_struct);
 
    // Enable USART1
    USART_Cmd(USART1, ENABLE);  
    /* Baud rate 9600, 8-bit data, One stop bit
     * No parity, Do both Rx and Tx, No HW flow control
     */
    /*usart1_init_struct.USART_BaudRate = 115200;   
    usart1_init_struct.USART_WordLength = USART_WordLength_8b;  
    usart1_init_struct.USART_StopBits = USART_StopBits_1;   
    usart1_init_struct.USART_Parity = USART_Parity_No ;
    usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // Configure USART1
    USART_Init(USART1, &usart1_init_struct);
    // Enable RXNE interrupt
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    // Enable USART1 global interrupt
    NVIC_EnableIRQ(USART1_IRQn);
}*/

// GPIO structure for port initialization
GPIO_InitTypeDef GPIO_InitStructure;
void setFloating(int num) {
	// configure port A1 for driving an LED
	GPIO_InitStructure.GPIO_Pin = toPin(num);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // output push-pull mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // highest speed
	GPIO_Init(toPort(num), &GPIO_InitStructure) ;             // initialize port

}
void setOutput(int num) {
	// configure port A1 for driving an LED
	GPIO_InitStructure.GPIO_Pin = toPin(num);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // output push-pull mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // highest speed
	GPIO_Init(toPort(num), &GPIO_InitStructure) ;             // initialize port

}



int main(void) {
//    USART1_Init();
//    printf("start\n");


    // enable clock on APB2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,  ENABLE);

    for(int i=0; i<DIM;i++) {
	    for(int j=0; j<DIM; j++) {
		setFloating(mapping[j][i]);
	    }
    }

    // main loop
    int x=0,y=0,z=0;
    while(1) {
	if(++x >= DIM) {
			x=0;
			if(++y >= DIM) {
				y=0;
        			GPIO_SetBits(toPort(mapping_z[z]), toPin(mapping_z[z]));    // turn the LED off
				setFloating(mapping_z[z]);
				if(++z >= DIM) {
					z=0;
				}
				setOutput(mapping_z[z]);
        			GPIO_ResetBits(toPort(mapping_z[z]), toPin(mapping_z[z]));    // turn the LED off
			}
		}

	setOutput(mapping[x][y]);
        GPIO_SetBits(toPort(mapping[x][y]), toPin(mapping[x][y]));    // turn the LED on
        //GPIO_SetBits(GPIOC, GPIO_Pin_13);    // turn the LED on
        delay(DELAY);

        GPIO_ResetBits(toPort(mapping[x][y]), toPin(mapping[x][y]));    // turn the LED off
	setFloating(mapping[x][y]);
        //GPIO_ResetBits(GPIOC, GPIO_Pin_13);  // turn the LED off
        //delay(DELAY);
    }
}
