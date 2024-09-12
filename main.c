#include "stm32f030x6.h"
#include "STM32F030-CMSIS-USART-lib.c"
//STM32F030K6T6
//    USART1_Tx = PA2 (pin 8)
int main( void )
{
    //LED PB0
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 

    GPIOB->MODER |= ( 0b01 << GPIO_MODER_MODER0_Pos );

    USART_init( USART1, 112500 );

    USART_putc('H');
    USART_puts("ello World!\n");

    while( 1 )
    {
        GPIOB->ODR ^= GPIO_ODR_0;
        USART_puts("Test!\n");
        for( uint32_t x=0; x<308e3; x++) ;
    }
    return 0;
}