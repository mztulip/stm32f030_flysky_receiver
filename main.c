#include <stdbool.h>
#include "stm32f030x6.h"
#include "STM32F030-CMSIS-USART-lib.c"

void init_spi_gpio(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // SCK PA5
    GPIOA->MODER |= GPIO_MODER_MODER5_1;  // alternate function
    // GPIOB->OTYPER default push-pull
    // GPIOB->AFR[] default AF 0: SPI1_SCK,
    // MOSI PA7
    GPIOA->MODER |= GPIO_MODER_MODER7_1;  // alternate function
    // MISO PA6
    GPIOA->MODER |= GPIO_MODER_MODER6_1;  // alternate function

    //SCS PA4 General purpose output mode
    GPIOA->MODER |= ( 0b01 << GPIO_MODER_MODER0_Pos );
    GPIOA->ODR |= GPIO_ODR_4; //Logic 1, deactivate SCS
}

void init_spi( void )
{
    init_spi_gpio();

    SPI1->CR1 |= SPI_CR1_BR_1;            // spi_sck = SystemCoreClock / 8 = 6 MHz
    SPI1->CR1 |= SPI_CR1_SSI;             // software CS
    SPI1->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2| SPI_CR2_DS_3; //16bit

    SPI1->CR1 |= SPI_CR1_SSM;
    SPI1->CR1 |= SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE;

}

void select_a7105(bool state)
{
    if(state)
    {
        GPIOA->ODR &= ~GPIO_ODR_4;
    }
    else 
    {
        GPIOA->ODR |= GPIO_ODR_4;
    }
}

uint8_t A7105_read_reg(uint8_t address)
{ 
	uint8_t result;

    select_a7105(true);
    //bit 7 command bit 0-register 1-strobe
    //bit6 R/W bit 1-read 0 -write
    uint16_t data = address|=0x40;
    SPI1->DR = data;
    while(!(SPI1->SR & SPI_SR_TXE));      // make sure TX buffer is empty
    while(SPI1->SR & SPI_SR_BSY);         // make sure SPI isn't busy
    select_a7105(false);
    result = SPI1->DR;
	return(result); 
} 

//STM32F030K6T6
//    USART1_Tx = PA2 (pin 8)
int main( void )
{
    //LED PB0
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 

    GPIOB->MODER |= ( 0b01 << GPIO_MODER_MODER0_Pos );

    USART_init( USART1, 112500 );
    USART_puts("Hello World!\n");

    const uint8_t RScale = 0x31;

    while( 1 )
    {
        GPIOB->ODR ^= GPIO_ODR_0;
        USART_puts("Test!\n");
        for( uint32_t x=0; x<308e3; x++) ;
        uint8_t rscale_val = A7105_read_reg(RScale);
        
    }
    return 0;
}