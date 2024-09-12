//  ==========================================================================================
//  STM32F030-CMSIS-USART-lib.c
//  ------------------------------------------------------------------------------------------
//  Minimalist Serial UART library for the STM32F030
//  ------------------------------------------------------------------------------------------
//  https://github.com/EZdenki/STM32F030-CMSIS-USART-lib
//  Released under the MIT License
//  Copyright (c) 2023
//  Mike Shegedin, EZdenki.com
//    Version 1.3   11 Oct 2023   Had putc wait until character is actually sent before
//                                returning to the calling routine.
//    Version 1.2   28 Aug 2023   Ported USART_puti, USART_puth, USART_pollc from
//                                STM32F103 code.
//    Version 1.1   16 Aug 2023   Added baud rate as parameter to USART_init
//    Version 1.0   24 Jul 2023   Updated core files and comments
//  ------------------------------------------------------------------------------------------
//  Target Devices: STM32F030xx and a USB-Serial dongle
//
//  Hardware: STM32030xx, USB-Serial dongle
//  Software: PuTTY for Windows or Linux (or any other serial terminal program)
//
//  ------------------------------------------------------------------------------------------
//  Summary:
//    Library of most basic functions to support serial communication to and from the
//    STM32F030. Note that only USART1 is currently supported.
//
//    USART1_Tx = PA2 (pin 8), Alternate Function 1
//    USART1_Rx = PA3 (pin 9), Alternate Function 1
//
//    Baudrate Calculation:
//      Assuming the internal RC clock: f(CK) = 8 MHz
//        Mantissa = whole part of f(CK) / (16 * Baud)
//        Fraction = remainder of above * 16
//          f(CK)    Baud     Mantissa   Fraction
//          -----   -------   --------   --------
//          8 MHz     9,600      52          1
//                  115,200       4          5
//                  460,800       1          1
//                  500,000       1          0
//
//        If calculating in program:
//          uartDiv  = freqClk / baud
//          mantissa = uartDiv / 16
//          fraction = uartDiv % 16
//
//    Steps to Set Up UART on STM32F030xx -- done by USART_init():
//      1. Enable GPIO Port A via RCC->AHBENR
//      2. Set PA2 and PA3 as Alternate Functions via GPIOA->MODER
//      3. Set Alternate Function 1 for PA2 and PA3 via GPIOA->AFR[0]
//      4. Enable USART1 peripheral via RCC->APB2ENR
//      5. Set Baudrate via USART1->BRR
//      6. Enable (turn on) Tx, Rx, and USART via USART1->CR1
//  ==========================================================================================

#ifndef __STM32F030_CMSIS_USART_LIB_C
#define __STM32F030_CMSIS_USART_LIB_C

#include <stdlib.h>
#include "stm32f030x6.h"  // Primary CMSIS header file


USART_TypeDef *USART_USART; // Global USART_USART varible to point to desired USART port


//  void
//  USART_init( USART_TypeDef *thisUSART, uint32_t baudrate )
//  Associate USART routines to designated USART port. The STM32F030F4 only supports USART1
//  so thisUSART must be passed only USART1 in this port.
//  Also sets the baud rate. Tested working baud rates are from 300 to 460,800 while using
//  the PuTTY terminal program.
//  This if this routine is modified to work with other microcontrollers, then the rest of
//  the USART_ routines should work as-is.
void
USART_init( USART_TypeDef *thisUSART, uint32_t baudrate )
{
  uint32_t speedMant, speedFrac;
    
  // Calculate the mantissa (speedMant) and fraction (speedFrac) values for an 8 MHz CPU
  // Note that the 8E6 constants are cast as uint32_t, otherwise gcc will consider them
  // to be floating constants with much more overhead.
  speedMant  = (uint32_t)8E6 / baudrate / 16;
  speedFrac = ( (uint32_t)8E6 - baudrate * speedMant * 16 ) / baudrate;

  USART_USART = thisUSART;    // Set global USART_USART varible to point to the desired port

  if( 1 ) // For parts with more than one USART, change this line to:
          // if( thisUSART == USART1 ){
          //   ...
          // and add setup code for other USART ports as needed.
  {
    // Enable GPIO Port A
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    // Set PA2 and PA3 as Alternate Functions
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER2_Pos); // USART1_TX/PA2/AF1/Pin8
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER3_Pos); // USART1_RX/PA3/AF1/Pin9
  
    // Set PA2 and PA3 as Alternate Function 1
    GPIOA->AFR[0] |= (0b0001 << GPIO_AFRL_AFRL2_Pos);
    GPIOA->AFR[0] |= (0b0001 << GPIO_AFRL_AFRL3_Pos);
  
    // Enable USART1 peripheral
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  
    // Set Baudrate by loading the baudrate Mantissa and Fractional part as described above
    USART_USART->BRR = ( speedMant << USART_BRR_DIV_MANTISSA_Pos ) |
                       ( speedFrac << USART_BRR_DIV_FRACTION_Pos );
  
    // Enable (turn on) Tx, Rx, and USART
    USART_USART->CR1 = (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE) ;
  }
  // End per-port setup
}


// void
// USART_putc
// Output a single character to the USART Tx pin (PA2)
void
USART_putc( char c )
{
    // Wait until the transmit data register is empty
    while( !(USART_USART->ISR & USART_ISR_TXE ) ) ;
    
    // Put character into the data register
    USART_USART->TDR = c; 

    // Wait until character is actually sent
    while( !(USART_USART->ISR & USART_ISR_TC) ) ;
}


// void
// USART_puts
// Output a string to the serial port. String should be null terminated (standard C string).
void
USART_puts( char *s )
{
    while( *s )
        USART_putc( *s++ );
}


//  void
//  USART_puti( int data, uint8_t base )
//  Writes an integer value to the serial port. "base" determines the base of the output:
//  10 is decimal, 16 is hex, 2 is binary.
void
USART_puti( int data, uint8_t base )
{
  char myString[10];
  itoa( data, myString, base );
  USART_puts( myString );
}


// char
// USART_getc( void )
// Waits for a character on the serial port and returns the character
// when received.
char
USART_getc( void )
{
    while( !( USART_USART->ISR & USART_ISR_RXNE ) ) ;
    return USART_USART->RDR;
}


//  void
//  USART_puth( uint32_t number, uint8_t places )
//  Writes a hex value to the serial port. "places" determins the number of digits to
//  output. If the value would exceed this number of digits, then a period . will be
//  printed in place of each digit. Can print from 1 to 8 hex digits.
void
USART_puth( uint32_t number, uint8_t places )
{
  uint8_t  thisDigit;
  uint32_t oob;     // 1 if out of number has more hex digits than "places", 0 if okay

  oob =  number >> ( places *4 ) ;
  for( int32_t x = (places - 1) * 4; x >=0; x -= 4 )
  {
    thisDigit = (number >> x) & 0xF;    // Extract the hex value for this digit
    if( oob )
      USART_putc( '.' );
    else
    {
      if( thisDigit < 10 )
        thisDigit += 0x30;
      else
        thisDigit += 0x37;
    
      USART_putc( thisDigit );
    }
  }
}


//  char
//  USART_pollc()
//  Poll serial terminal for input. If there is no input, return 0, otherwise return ASCII
//  code for key pressed. Note that this routine will return codes for non-printable
//  characters.
char
USART_pollc()
{
  if( USART_USART->ISR & USART_ISR_RXNE )
    return USART_USART->RDR;
  else
    return 0;
}


//  uint32_t
//  USART_gets( char *inStr, uint32_t bufLen )
//  Gets a string input from the serial port. Characters will be added to the inpString buffer
//  until <Enter> is pressed. Once the string reaches the length of bufLen-1, only the
//  backspace or <Enter> keys will be recognized. The typed in string will be output to the
//  terminal. The inpString will be terminated with a null character (0x00) character. There
//  is no ending linefeed (\n) character attached to the string. Returns the length of the
//  string minus the ending null character.
uint32_t
USART_gets( char *inStr, uint32_t strLen )
{
  uint32_t strPos = 0;    // Track position in string
  uint8_t  oneChar;       // Hold currently entered character for processing
  
  oneChar = USART_getc();                 // Get the first character
  
  while( oneChar != 13 )                  // Loop until <Enter> key is pressed
  {
    // If got a printable character and not at end of the string...
    if( oneChar >= 0x20 && oneChar <= 0x7E && strPos < ( strLen - 1 ) )
    {
      inStr[ strPos++ ] = oneChar;        // Add character to input string
      USART_putc( oneChar );              // Display character on terminal
    }
    else                                  // Otherwise, got some kind of control character
    {
      if( oneChar == 127 && strPos > 0 )  // Process backspace key
      {
        USART_putc( oneChar );            // Update terminal. 0x7F displays as proper delete 
        strPos--;                         // Decrement bufCnt to effectively erase the
      }                                   // previous character from inpString.
    }
    oneChar = USART_getc();               // Get next character
  }
  inStr[ strPos ] = 0x00;                 // Terminate string with null character
  return strPos;                          // Return the length of the string not including
}                                         // the end-of-string character


#endif /* __STM32F030-CMSS_USART_LIB_C   */
