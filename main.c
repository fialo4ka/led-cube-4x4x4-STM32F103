#include <stdio.h>
#include <string.h>
// #include <stm32f10x_usart.h>

#include "main.h"

#define DIM 4

int mapping[DIM][DIM] = {
	{0x1b, 0x1a, 0x11, 0x10},
	{0x07, 0x06, 0x05, 0x04},
	{0x03, 0x02, 0x01, 0x00},
//	{0x0b, 0x0c, 0x0f, 0x13}
	{0x15, 0x16, 0x17, 0x18}
};

int mapping_z[DIM] = {0x1d,0x1e,0x1f,0x08};

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

void turnLEDon(int a){
		setOutput(a);
		GPIO_SetBits(toPort(a), toPin(a));    // turn the LED on
}
void turnLEDoff(int a){
		GPIO_ResetBits(toPort(a), toPin(a));    // turn the LED off
		setFloating(a);
}
void turnLEDonRow(int a){
		setOutput(a);
		GPIO_ResetBits(toPort(a), toPin(a));    // turn the LED on
}
void turnLEDoffRow(int a){
		GPIO_SetBits(toPort(a), toPin(a));    // turn the LED off
		setFloating(a);
}

void setLayer(){
	for (int z=0; z<4;z++){
		turnLEDonRow( mapping_z[z]);
		for (int y=0; y<4;y++){
			for (int x=0; x<4;x++){
				turnLEDon( mapping[x][y]);
			}
		}
		delay(DELAY);
		for (int y=0; y<4;y++){
			for (int x=0; x<4;x++){
				turnLEDoff(mapping[x][y]);
			}
		}
		turnLEDoffRow(mapping_z[z]);
	}

}
void setColor(){
	for (int z=0; z<4;z++){
		turnLEDonRow( mapping_z[z]);
		
		for (int y=z; y<4;y++){
			for (int x=z; x<4;x++){
				turnLEDon( mapping[x][y]);
			}
		}

		delay(DELAY*10);
		for (int y=z; y<4;y++){
			for (int x=z; x<4;x++){
				turnLEDoff(mapping[x][y]);
			}
		}
		turnLEDoffRow(mapping_z[z]);
	}

}
void setLed(){

	for (int z=0; z<4;z++){
		turnLEDonRow(mapping_z[z]);
		for (int y=0; y<4;y++){
			for (int x=0; x<4;x++){
				turnLEDon( mapping[x][y]);
			        delay(DELAY);
	       			turnLEDoff(mapping[x][y]);
			}
		}	
		turnLEDoffRow(mapping_z[z]);
	}
}
 
int main(void) {
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
    while(1) {
    	setLed();
	setLayer();setLayer();setLayer();
	delay(DELAY);
	setColor();setColor();setColor();setColor();
	delay(DELAY);
    }
    
}
