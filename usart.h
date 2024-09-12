#ifndef __STM32F030_CMSIS_USART_LIB_C
#define __STM32F030_CMSIS_USART_LIB_C

void USART_init( USART_TypeDef *thisUSART, uint32_t baudrate );
void USART_putc( char c );
void USART_puts( char *s );
void USART_puti( int data, uint8_t base );
char USART_getc( void );
void USART_puth( uint32_t number, uint8_t places );
char USART_pollc();
uint32_t USART_gets( char *inStr, uint32_t strLen );


#endif /* __STM32F030-CMSS_USART_LIB_C   */
