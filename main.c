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

static inline bool GIO1_get_state(void)
{
    return ( GPIOA->IDR&GPIO_ODR_6) != 0;
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
const uint8_t Calibration_control_reg = 0x02;
const uint8_t ID_Data_reg = 0x06;
const uint8_t GIO1_Pin_Control_reg1 = 0x0b;
const uint8_t PLL_reg1 = 0x0f;
const uint8_t IDL_reg = 0x1F;
const uint8_t VCO_current_calibration_reg = 0x24;
const uint8_t VCO_single_band_cal_reg1 = 0x25;
const uint8_t VCO_single_band_cal_reg2 = 0x26;
const uint8_t battery_detect_reg = 0x27;
const uint8_t RScale_reg = 0x31;

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

static inline void delay_fast(void)
{
    for( uint32_t x=0; x<100e3; x++) ;
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

void A7105_reset(void)
{
    A7105_write_reg(Mode_reg, 0);
}

void A7105_write_ID(uint32_t id)
{
    select_a7105(false);
    SCK_low();
    SDIO_output();
    SDIO_set_state(0);
    select_a7105(true);
    //bit 7 command bit 0-register 1-strobe
    //bit6 R/W bit 1-read 0 -write
    uint8_t addr = ID_Data_reg;
    for(int i = 0x80 ; i != 0;i=i>>1)
    {
        bool bit = i&addr;
        SDIO_set_state(bit);
        SCK_high();
        SCK_low();
    }

    for(uint32_t i = 0x80000000 ; i != 0;i=i>>1)
    {
        bool bit = i&id;
        SDIO_set_state(bit);
        SCK_high();
        SCK_low();
    }
    select_a7105(false);
}

uint32_t A7105_read_ID(void)
{
    uint32_t result = 0;
    select_a7105(false);
    SCK_low();
    SDIO_output();
    SDIO_set_state(0);
    select_a7105(true);
    //bit 7 command bit 0-register 1-strobe
    //bit6 R/W bit 1-read 0 -write
    uint8_t addr = (ID_Data_reg|0x40);
    for(int i = 0x80 ; i != 0;i=i>>1)
    {
        bool bit = i&addr;
        SDIO_set_state(bit);
        SCK_high();
        SCK_low();
    }
    SDIO_input();
    for(uint32_t i = 0x80000000 ; i != 0;i=i>>1)
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

void A7105_init(void)
{
    A7105_reset();
    A7105_presence_test();
    delay();

    A7105_write_reg(0, 0xff); //As in A7105 Reference code for FIFO mode
    //Crystal A7105= 16MHz=Fxtal
    // FMS=1(FIFO mode) AIF=0(Auto IF offset for RX)
    // ARSSI = 1( Enable RSSI measurement in RX mode)
    A7105_write_reg(1, 0x42); // mbed, multimodule have 0x42
    A7105_write_reg(2, 0); //Calibs disable
    A7105_write_reg(3, 0x14); //FIFO End pointer set, todo change, 20 bytes
    A7105_write_reg(4, 0); //Fifo pointer margin
    A7105_write_reg(5, 0xff); //FIFO data 
    A7105_write_reg(6, 0xff); //ID Data
    A7105_write_reg(7, 0); //RC OSC  as default(internal usage)
    A7105_write_reg(8, 0); //RC OSC 2 as default (internal usage)
    A7105_write_reg(9, 0); //RC OSC 3 TWWS_E=0(disabled), RCOSC_E=0(internal rc disabled), BBCKS01=0(digit block clk=Fsys/8)
    A7105_write_reg(10, 0); //CKO pin control, SCKI=0(clock non inverted), CKOE=0(high z) 
    //CKOI=0(non inverted) CKOS[3:0]=0(DCK(Data clock) in TX and RCK in RX)
    A7105_write_reg(11, 0x01); //GIO1 control reg, GIO1OE=1(Enabled), GIO1S[3:0]=0 (Wait until RX or TX finished)
    A7105_write_reg(12, 0x0D);  //GIO2 control reg, GIO2OE=1(enabled pin), GIO2S[3:0]=0b0011,there is no plan to use GIO2
    A7105_write_reg(13, 0x05);//Clock register, XS=1(Crystal), CGS=0 (disable internal 32MHz PLL)
    //CSC[1:0]=01(syclk divider Fsys=FMCLK/2=16MHz),GRC[3:0]=0b000(clock generation reference counter)
    A7105_write_reg(14, 0x00);//Data rate register, Data rate = Fsysclk/32/(SDR+1), SDR=0 Datarete=16MHz/32/1=0.5MHz
    A7105_write_reg(15, 50); //PLL register 1, CHN[7:0]=50 (LO channel number), this is overwritten later

    A7105_write_reg(0x10, 0x9e);//1001 1110, PLL register 2, BIP8=0, CHR[3:0]=0b1111(16) 
    //RRC[1:0]=0b00(FPLL=Fxref/1=32MHz) DBL=1(FXREF=2xFxtal=32MHz=FMclk)     Freq=2400+32*50(CHN)/(4*(16+1))=2400+1600/68=2400+23
    A7105_write_reg(0x11, 0x4b); //PLL register 3, BIP[7:0]=0x4b(LO base frequency integer part) 32MHz*75(0x4b)=2400MHz
    A7105_write_reg(0x12, 0x00); //PLL register 4, BFP[15:8]=0
    A7105_write_reg(0x13, 0x02); //PLL register 5, BFP[7:0]=2(LO base frequency fractional part)
    A7105_write_reg(0x14, 0x16); //TX register 1, FDP[2:0]=0b110(frequency deviation power setting)
    //FS=0(gaussian filter disabled), TME=1(tx modulation enable), TXDI=0(tx data not inverted) TXSM[1:0]=0b00 (no average for no filter selection)
    A7105_write_reg(0x15, 0x2b); //TX register 2, FD[4:0]=0xb(frequency deviation), PDV[1:0]=01(should be 01)
    A7105_write_reg(0x16, 0x12); //Delay register 1, PDL[2:0]=2(Delay for TX setting from PLL to WPLL, 10,70us) 2 is recommended val
    //TDL[1:0]=10(reccommended val, 10,60us,delay from WPLL to TX for TX) DPR[2:0]=0(reccomended val, delay scale)
    A7105_write_reg(0x17, 0x00); //Delay register 2, RS_DLY[2:0]=0b000(10us, RSSI measurement delay)
    //AGC_D[1:0]=0b00(10us,recommended val, AGC delay settling) WSEL[2:0]=0b010(200us xtal settling delay)
    // in my understading with this oscillator is ready in 300+200us=500us from standby
    A7105_write_reg(0x18, 0x62); //0b0110 0010 RX register, ULS=0(Rx up side band)  BWS=1(500kHz bandwidth FIF) DMG=0(should be 0)
    //RXDI=0(RX data output not inverted) FC=0(frequency compensation disabled), RXSM[1:0]=0b11(internal usage should be 11)
    A7105_write_reg(0x19, 0x80);//RX gain register 1, LGC[2:0]=0(LNA gain 24dB), MGC[1:0]=0(mixer gain 24dB),
    //IGC=0(internal should be 0), MVGS=1(recommended val, manual VGA calibrate) VGA(variable gain amplifiers, placed after IF filter)
    A7105_write_reg(0x1a, 0xff); //RX gain register 2 for internal use, should be 0x0a, multi have 0x80, mbed hax 0x80
    A7105_write_reg(0x1b, 0xff); //Rx gain register 3 for internal use, should be 0xb4, multi have 0x00, mbed, 0x00
    A7105_write_reg(0x1c, 0x2a); //Rx gain register 4, for internal use, should be 0x8e, multi,mbed, 0x0a
    A7105_write_reg(0x1d, 0x32); //RSSI threshold register, 
    A7105_write_reg(0x1e, 0xc3); //0b1100 0011 ADC Control register, CDM=1(RSSI continuous) RSS=1(RSSI od carrier detect, only one option)
    //XADS=0 (convert RSS signal, only one option), FSARS=0(ADC clk 4Mhz) ERSS=0(RSSI measuerement until RX mode) RSM[1:0]=11(RSSI margin 20, recommended val)
    A7105_write_reg(0x1f, 0x0f); //Code register 1, PML[1:0]=11(preamble 4bytes) IDL=1(preamble 4 bytes) CRCS=1(crc enabled)
    //FECS=0 (FEC disabled) WHTS=0 (data whitening disabled) MCS=0(do not know what is it)

    A7105_write_reg(0x20, 0x13); //Control register 2, PMD[1:0]=11(preamble pattern detection length 16bits)
    //ETH[1:0]=0(id code error tolerance 0bit) DCL[2:0]=1(demodulator dc estimation average mode, reccomended val)
    A7105_write_reg(0x21, 0xc3); //Code register 3, WS[6:0]=0xc3 Data whitening seed setting
    A7105_write_reg(0x22, 0x00); //IF calibration register 1, 0 auto calibration(recommended)
    A7105_write_reg(0x23, 0xff); //IF calibration register 2, readonly, IF filter calibration deviation
    A7105_write_reg(0x24, 0); //VCO current calibration, shall be 0, auto calibration
    A7105_write_reg(0x25, 0); //VCO single band calibration register, 0 means auto
    A7105_write_reg(0x26, 0x3b); //0b0011 1011 VCO single band calibration register 2, VTL[2:0]=011(recommended val, vco tuning voltage lower threshold 0.4V)
    //VTH[2:0]=111(recommended val, 1.3V vco tuning voltage upper threshold)
    A7105_write_reg(0x27, 0); //Battery detect register, BDF=0(detection when battery less than threshold), BDS=0(battery detect disabled)...
    //BVT[2:0]=0 RGV[1:0]=00(2.1V VVD_A VDD_D voltage setting in non sleep mode, recommended val) RGS=0(VDD_D 3/5*REG_I)
    A7105_write_reg(0x28, 0x17); //0b0001 0111 TX test register, TBG[2:0]=111(tx buffer setting), PAC[1:0]=10(recommended val, PA current setting)
    //TCS=0(only one value) Chapter 19, Values means 0.1dBm  
    A7105_write_reg(0x29, 0x47); //0b0100 0111 RX DEM test register 1, SLF[2:0]=111(recommended val, internal) MPL[1:0]=00(recommended val,internal)
    //DCM[1:0]=10(Average and hold mode, DC level is average value hold about 8 bit data rate later if preamble detected) DMT=0(shall be 0)
    A7105_write_reg(0x2a, 0x80); //RX DEM test register 2, (recommended val, demodulator fix dc mode value)
    A7105_write_reg(0x2b, 0x03); //Charge pump current register, CPC[1:0]=3(reccomended val, 2mA)
    A7105_write_reg(0x2c, 0x01); //Crystal test register, XC[1:0]=1(shall be 1), XCC=0(shall be 0, internal) DBD=0(shall be 0, internal)
    A7105_write_reg(0x2d, 0x45); //PLL test register, 0b0100 0101, NSDO=1(sholud be 1, internal) SDPW=0(should be 0, internal)
    //PRIC[1:0]=01(should be 01, internal), PRRC[1:0]=00(should be 0, internal) PMPE=1(should be 1, internal)
    A7105_write_reg(0x2e, 0x18); //VCO test register 1, 0b0001 1000 , internal should be as is
    A7105_write_reg(0x2f, 0); //VCO test register 2, rf analog pin configuration for testing, should be as is

    A7105_write_reg(0x30, 0x01); //IFAT register, internal should be as is
    A7105_write_reg(0x31, 0x0f); //RScale, internal, should be as is
    A7105_write_reg(0x32, 0xff); //Filter test register, internal, should be set as is

    A7105_strobe(A7105_STROBE_STANDBY);
}

void A7105_calibrate(void)
{
    //IF Filter Bank Calibration
    A7105_write_reg( Calibration_control_reg, 0x01 );
    while ( A7105_read_reg(Calibration_control_reg) ); //wait calibration end

    //VCO Current Calibration manual, I found that others do it in that way.
    A7105_write_reg(VCO_current_calibration_reg, 0x13);

    //VCO Bank Calibration, VDD_A=1.3V, upper threshold, lowe 0.4V
    A7105_write_reg(VCO_single_band_cal_reg2, 0x03B);


    
    //VCO Bank Calibrate channel 0x00
    A7105_write_reg( PLL_reg1, 0x00); //Set Channel 0 it means 2400MHz+0
    A7105_write_reg( Calibration_control_reg, 0x02); //VCO Calibration
    printf("VCO calibration started with channel 0x0\n\r");
    while ( A7105_read_reg(Calibration_control_reg) )//wait calibration end
    {
        delay();
        printf("Waiting for calibraiton end\n\r");
    } 
    uint8_t vco_calibration0=A7105_read_reg(VCO_single_band_cal_reg1);
    if(vco_calibration0&0x08)
    {//VCBS=1 means calibration failed
        printf("VCO calibration failed with ch0\n\r");
    }
    
    
    //VCO Bank Calibrate channel 0xA0
    A7105_write_reg( PLL_reg1, 0xA0); //Set Channel 0xA0
    A7105_write_reg( Calibration_control_reg, 0x02); //VCO Calibration
    printf("VCO calibration started with channel 0xA0\n\r");
    while ( A7105_read_reg(Calibration_control_reg) )//wait calibration end
    {
        delay();
        printf("Waiting for calibraiton end\n\r");
    } 
    vco_calibration0=A7105_read_reg(VCO_single_band_cal_reg1);
    if(vco_calibration0&0x08)
    {//VCBS=1 means calibration failed
        printf("VCO calibration failed with channel 0xA0\n\r");
    }

    //Switch to manual VCO bank calibration value MVBS=1, WHY?
    // A7105_write_reg(VCO_single_band_cal_reg1,0x08);
    printf("Calibrations finished\n\r");
}

void A7105_continuous_TX(void)
{
    //PA6-GIO1 as output
    GPIOA->MODER |= ( 0b01 << GPIO_MODER_MODER6_Pos );

    //Continuous TX test
    //0b1000 0000 = 0x80
    //FMS=0 direct mode FMT=0, WWSE=0, DCFC=0, AIF=0, ARSSI=0, DDPC=0(SDIO as modulation drive pin disabled)
    A7105_write_reg(Mode_control_reg, 0x00);
    A7105_write_reg( PLL_reg1, 0x00); //Set Channel 0 it means 2400MHz+0
    A7105_write_reg(GIO1_Pin_Control_reg1, (0x9<<2)| 0x01); //TXD direct mode
    A7105_strobe(A7105_STROBE_TX);
    uint8_t val;
    val = A7105_read_reg(Mode_reg);
    printf("Mode reg(0x00): %02x\n\r", val);
    printf("Continuous TX activated\n\r");

    //This hops with frequency beetwen 2.39985 and 2.40025
    while(1)
    {
        led_toogle();
        GPIOA->ODR ^= GPIO_ODR_6;
        delay();
    }
}

void A7105_continuous_TX_hops(void)
{
    //PA6-GIO1 as output
    GPIOA->MODER |= ( 0b01 << GPIO_MODER_MODER6_Pos );

    //Continuous TX test
    //0b1000 0000 = 0x80
    //FMS=0 direct mode FMT=0, WWSE=0, DCFC=0, AIF=0, ARSSI=0, DDPC=0(SDIO as modulation drive pin disabled)
    A7105_write_reg(Mode_control_reg, 0x00);
    A7105_write_reg( PLL_reg1, 0x00); //Set Channel 0 it means 2400MHz+0
    A7105_write_reg(GIO1_Pin_Control_reg1, (0x9<<2)| 0x01); //TXD direct mode
    A7105_strobe(A7105_STROBE_TX);
    uint8_t val;
    val = A7105_read_reg(Mode_reg);
    printf("Mode reg(0x00): %02x\n\r", val);
    printf("Continuous TX activated\n\r");


    //This hops with frequency beetwen f0+-250khz
    uint8_t channel = 0;
    while(1)
    {
        led_toogle();
        GPIOA->ODR ^= GPIO_ODR_6;
        delay_fast();
        GPIOA->ODR ^= GPIO_ODR_6;
        delay_fast();
        //Freq=2400+32*(CHN)/(4*(16+1))=2400+CHN*0.47MHz
        //Max 0xa0(160) Fchmax=2400*160*0,47=2.4752MHz
        A7105_write_reg( PLL_reg1, channel%0xa0);
        channel++;
    }
}


//STM32F030K6T6
//USART1_Tx = PA2 (pin 8)
int main( void )
{
    led_init();
    USART_init( USART1, 112500 );
    printf("Hello World printf\n\r");
    init_3wire_gpio();
    printf("3wire initialised!\n\r");
    delay();
    A7105_init();
    A7105_calibrate();

    A7105_write_ID(0x5475c52A);
    printf("Reading ID\n\r");
    uint32_t id_result = 0x12345678;
    id_result = A7105_read_ID();
    printf("\033[32mFrame id set to: %lx\033[0m\n\r", id_result);
    if(id_result != 0x5475c52A)
    {
        printf("\033[31mError: Frame id set failed\033[0m\n\r");
        //TODO reboot
    }

    // A7105_continuous_TX_hops();
    A7105_strobe(A7105_STROBE_STANDBY);
    A7105_strobe(A7105_STROBE_RST_RDPTR);
    //Binding packets are received on ch0
    A7105_write_reg( PLL_reg1, 0x00); //Set Channel 0 it means 2400MHz+0
    A7105_strobe(A7105_STROBE_RX);
    printf("Waiting for radio frame\n\r");

    while( 1 )
    {
        led_toogle();
        if(GIO1_get_state()) //Packet REceived
        {
            printf("RX\n\r");
            uint8_t val = A7105_read_reg(Mode_reg);
            printf("Mode reg(0x00): %02x\n\r", val);
            if((val&(1<<5)) == 0) //CRC ok, CRCF bit cleared
            {
                printf("CRC ok \n\r");
            }
        }
    }
    return 0;
}