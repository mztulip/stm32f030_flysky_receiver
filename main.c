#include <stdbool.h>
#include <stdio.h>
#include "stm32f030x6.h"
#include "usart.h"

void printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;
    
    for (i = size-1; i >= 0; i--) {
        for (j = 7; j >= 0; j--) {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
    puts("\r");
}

static inline void select_a7105(bool state)
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

static inline void SCK_low(void)
{
    GPIOA->ODR &= ~GPIO_ODR_5; 
}

static inline void SCK_high(void)
{
    GPIOA->ODR |= GPIO_ODR_5; 
}

static inline void SDIO_output(void)
{
    GPIOA->MODER |= GPIO_MODER_MODER7_0;
}

static inline void SDIO_input(void)
{
    GPIOA->MODER &= ~GPIO_MODER_MODER7_Msk;
}

static inline bool SDIO_get_state(void)
{
    // printf("0x%lx\n\r", GPIOA->IDR&GPIO_ODR_7);
    return ( GPIOA->IDR&GPIO_ODR_7) != 0;
}

static inline void SDIO_set_state(bool x)
{
    if(x)
    {
        GPIOA->ODR |= GPIO_ODR_7;
    }
    else 
    {
        GPIOA->ODR &= ~GPIO_ODR_7;
    }
}

void init_3wire_gpio(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // SCK PA5
    GPIOA->MODER |= GPIO_MODER_MODER5_0;
    // SDIO PA7
    // GPIOA->MODER |= GPIO_MODER_MODER7_0;
    // GIO1 PA6
    GPIOA->MODER |= GPIO_MODER_MODER6_0;
    //SCS PA4 General purpose output mode
    GPIOA->MODER |= ( 0b01 << GPIO_MODER_MODER4_Pos );

    select_a7105(false);//Logic 1, deactivate SCS
    SCK_low();
}

void A7105_write_reg(uint8_t address, uint8_t value)
{ 
    select_a7105(true);
    //bit 7 command bit 0-register 1-strobe
    //bit6 R/W bit 1-read 0 -write
    uint16_t addr = address;
    uint16_t data = addr << 8;
    select_a7105(false);
} 

uint8_t A7105_read_reg(uint8_t address)
{ 
	uint16_t result = 0;
    select_a7105(false);
    SCK_low();
    SDIO_output();
    SDIO_set_state(0);
    select_a7105(true);
    //bit 7 command bit 0-register 1-strobe
    //bit6 R/W bit 1-read 0 -write
    uint8_t addr = (address|=0x40);
    for(int i = 0x80 ; i != 0;i=i>>1)
    {
        bool bit = i&addr;
        SDIO_set_state(bit);
        SCK_high();
        SCK_low();
    }
    SDIO_input();
    for(int i = 0x80 ; i != 0;i=i>>1)
    {
        SCK_high();
        if(SDIO_get_state())
        {
            result |= i;
        }
        SCK_low();
    }

    select_a7105(false);
	return(result); 
}

void led_init(void)
{
    //LED PB0
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 
    GPIOB->MODER |= ( 0b01 << GPIO_MODER_MODER0_Pos );
}

void led_toogle(void)
{
    GPIOB->ODR ^= GPIO_ODR_0;
}

//STM32F030K6T6
//    USART1_Tx = PA2 (pin 8)
int main( void )
{
    led_init();
    USART_init( USART1, 112500 );
    printf("Hell World printf\n\r");
    init_3wire_gpio();
    printf("3wire initialised!\n\r");
    const uint8_t RScale_reg = 0x31;
    const uint8_t battery_detect_reg = 0x27;

    while( 1 )
    {
        led_toogle();
        for( uint32_t x=0; x<308e3; x++) ;

        uint8_t val;
        val = A7105_read_reg(battery_detect_reg);
        printf("Battery: %02x\n\r", val);
        // A7105_write_reg(RScale_reg, 1);
        val = A7105_read_reg(RScale_reg);
        printf("Rscale: %02x\n\r", val);
        
    }
    return 0;
}