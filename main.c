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
    select_a7105(false);
    SCK_low();
    SDIO_output();
    SDIO_set_state(0);
    select_a7105(true);
    //bit 7 command bit 0-register 1-strobe
    //bit6 R/W bit 1-read 0 -write
    uint16_t addr = address;
    uint16_t data = (addr << 8)| value;
    for(int i = 0x8000 ; i != 0;i=i>>1)
    {
        bool bit = i&data;
        SDIO_set_state(bit);
        SCK_high();
        SCK_low();
    }
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

const uint8_t Mode_reg = 0x0;
const uint8_t Mode_control_reg = 0x1;
const uint8_t ID_Data_reg = 0x06;
const uint8_t IDL_reg = 0x1F;
const uint8_t RScale_reg = 0x31;
const uint8_t battery_detect_reg = 0x27;


enum A7105_Command {
    A7105_STROBE_SLEEP     = 0x80,
    A7105_STROBE_IDLE      = 0x90,
    A7105_STROBE_STANDBY   = 0xA0,
    A7105_STROBE_PLL       = 0xB0,
    A7105_STROBE_RX        = 0xC0,
    A7105_STROBE_TX        = 0xD0,
    A7105_STROBE_RST_WRPTR = 0xE0,
    A7105_STROBE_RST_RDPTR = 0xF0,
};

bool A7105_presence_test(void)
{
    A7105_write_reg(ID_Data_reg, 0x55);
    uint8_t val = A7105_read_reg(ID_Data_reg);
    if(val == 0x55)
    {
        printf("A7105 present, Success!\n\r");
        return true;
    }
    printf("Error: A7105 not detected. Fail!\n\r");
    return false;
}

static inline void delay(void)
{
    for( uint32_t x=0; x<308e3; x++) ;
}

void A7105_strobe(uint8_t cmd)
{
     select_a7105(false);
    SCK_low();
    SDIO_output();
    SDIO_set_state(0);
    select_a7105(true);
    //bit 7 command bit 0-register 1-strobe
    //bit6 R/W bit 1-read 0 -write
    uint8_t data = 0x80| cmd;
    for(int i = 0x80 ; i != 0;i=i>>1)
    {
        bool bit = i&data;
        SDIO_set_state(bit);
        SCK_high();
        SCK_low();
    }
    select_a7105(false);
}

bool A7105_reset(void)
{
    A7105_write_reg(Mode_reg, 0);
    delay();
    A7105_strobe(A7105_STROBE_STANDBY);
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
//USART1_Tx = PA2 (pin 8)
int main( void )
{
    led_init();
    USART_init( USART1, 112500 );
    printf("Hell World printf\n\r");
    init_3wire_gpio();
    printf("3wire initialised!\n\r");

    A7105_presence_test();
    A7105_reset();
    //Crystal A7105= 16MHz=Fxtal
    // FMS=1(FIFO mode) AIF=1(Auto IF offset for RX)
    // ARSSI = 1( Enable RSSI measurement in RX mode)
    A7105_write_reg(1, 0x62);
    A7105_write_reg(2, 0); //Calibs disable
    A7105_write_reg(3, 0x25); //FIFO End pointer set
    A7105_write_reg(4, 0); //Fifo pointer margin
    A7105_write_reg(5, 0xff); //FIFO data 
    A7105_write_reg(6, 0xff); //ID Data
    A7105_write_reg(7, 0); //RC OSC  as default(internal usage)
    A7105_write_reg(8, 0); //RC OSC 2 as default (internal usage)
    A7105_write_reg(9, 0); //RC OSC 3 TWWS_E=0(disabled), RCOSC_E=0(internal rc disabled), BBCKS01=0(digit block clk=Fsys/8)
    A7105_write_reg(10, 0); //CKO pin control, SCKI=0(clock non inverted), CKOE=0(high z) 
    //CKOI=0(non inverted) CKOS[3:0]=0(DCK(Data clock) in TX and RCK in RX)
    A7105_write_reg(11, 0x01); //GIO1 control reg, GIO1OE=1(Enabled), GIO1S[3:0]=0 (Wait until RX or TX finished)
    A7105_write_reg(12, 0x0D);  //GIO2 control reg, GIO2OE=1(enabled pin), GIO2S[3:0]=0b0011 
    A7105_write_reg(13, 0x05);//Clock register, XS=1(Crystal), CGS=0 (disable internal 32MHz PLL)
    //CSC[1:0]=01(syclk divider Fsys=FMCLK/2=16MHz),GRC[3:0]=0b000(clock generation reference counter)
    A7105_write_reg(14, 0x00);//Data rate register, Data rate = Fsysclk/32/(SDR+1), SDR=0
    A7105_write_reg(15, 50); //PLL register 1, CHN[7:0]=50 (LO channel number)

    A7105_write_reg(0x10, 0x9e);//1001 1110, PLL register 2, BIP8=0, CHR[3:0]=0b1111 
    //RRC[1:0]=0b00 DBL=1(FXREF=2xFxtal=32MHz=FMclk)
    A7105_write_reg(0x11, 0x4b); //PLL register 3, BIP[7:0]=0x4b(LO base frequency integer part)
    A7105_write_reg(0x12, 0x00); //PLL register 4, BFP[15:8]=0
    A7105_write_reg(0x13, 0x02); //PLL register 5, BFP[7:0]=2(LO base frequency fractional part)
    A7105_write_reg(0x14, 0x16); //TX register 1, FDP[2:0]=0b110(frequency deviation power setting)
    //FS=0(gaussian filter disabled), TME=1(tx modulation enable), TXDI=0(tx data not inverted) TXSM[1:0]=0b00 (no average for no filter selection)
    A7105_write_reg(0x15, 0x2b); //TX register 2, FD[4:0]=0xb(frequency deviation), PDV[1:0]=01(should be 01)
    A7105_write_reg(0x16, 0x12); //Delay register 1, PDL[2:0]=2(Delay for TX setting from PLL to WPLL, 10,70us) 2 is recommended val
    //TDL[1:0]=10(reccommended val, 10,60us,delay from WPLL to TX for TX) DPR[2:0]=0(reccomended val, delay scale)
    A7105_write_reg(0x17, 0x4f); //Delay register 2, RS_DLY[2:0]=0b111(80us, RSSI measurement delay)
    //AGC_D[1:0]=0b01(20us, AGC delay settling) WSEL[2:0]=0b010(600us, recommended val, xtal settling delay) in my understading with this oscillator is ready in 900us from standby
    A7105_write_reg(0x18, 0x62); //0b0110 0010 RX register, ULS=0(Rx up side band)  BWS=1(500kHz bandwidth FIF) DMG=0(should be 0)
    //RXDI=0(RX data output not inverted) FC=0(frequency compensation disabled), RXSM[1:0]=0b11(internal usage should be 11)
    A7105_write_reg(0x19, 0x80);//RX gain register 1, LGC[2:0]=0(LNA gain 24dB), MGC[1:0]=0(mixer gain 24dB),
    //IGC=0(internal should be 0), MVGS=1(recommended val, manual VGA calibrate) VGA(variable gain amplifiers, placed after IF filter)
    A7105_write_reg(0x1a, 0xff);
    A7105_write_reg(0x1b, 0xff);
    A7105_write_reg(0x1c, 0x2a);
    A7105_write_reg(0x1d, 0x32);
    A7105_write_reg(0x1e, 0xc3);
    A7105_write_reg(0x1f, 0x1f);



    while( 1 )
    {
        led_toogle();
        delay();

        uint8_t val;
        val = A7105_read_reg(battery_detect_reg);
        printf("Battery: %02x\n\r", val);
        val = A7105_read_reg(RScale_reg);
        printf("Rscale: %02x\n\r", val);
        
    }
    return 0;
}