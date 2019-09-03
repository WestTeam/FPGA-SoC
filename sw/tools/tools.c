
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

 #include "tools.h"


volatile uint32_t* uart_jtag = (uint32_t*)(0x01000400);



volatile uint32_t* uart_rs232_0 = (volatile uint32_t*)(0x01000440);
volatile uint32_t* uart_rs232_1 = (volatile uint32_t*)(0x01000480);

volatile uint32_t* uart_rs232 = (volatile uint32_t*)(0x01000440);

inline uint32_t get_time()
{
	int val;
	asm volatile ("csrr %0,mtime":"=r"(val));
	return val;
}

void delay(int cycles)
{
	unsigned start=get_time();
	while(get_time() - start < cycles);
}



void jtaguart_putc(char c)
{

	while((uart_jtag[1]&0xffff0000) == 0){
	}//uart fifo full
	uart_jtag[0]=c;

}

void jtaguart_puts(char* s)
{
	while(*s){
		jtaguart_putc(*s++);
	}

}

char jtaguart_getchar()
{
    unsigned int reg = uart_jtag[0];
    
    if (((reg>>15)&0x1)==1)
        return (reg&0xff);
    else
        return '\0';

}


void uart_rs232_select(uint8_t id)
{
    uart_rs232 = uart_rs232_0;
    if (id == 1)
        uart_rs232 = uart_rs232_1;
}

#define UART_RS232_REG_DIVISOR 4

void uart_rs232_configure(uint16_t divisor)
{
    uart_rs232[UART_RS232_REG_DIVISOR] = (uint32_t)(divisor+1);

}

#define UART_RS232_REG_RX_DATA 0
#define UART_RS232_REG_TX_DATA 1
#define UART_RS232_REG_STATUS 2

#define UART_RS232_REG_STATUS_TRDY_OFFSET  (6)
#define UART_RS232_REG_STATUS_TRDY_MASK (1<<UART_RS232_REG_STATUS_TRDY_OFFSET)

void uart_rs232_tx(uint8_t data)
{
    uint32_t status;

    do {
        status = uart_rs232[UART_RS232_REG_STATUS];
   
        if (status & UART_RS232_REG_STATUS_TRDY_MASK)
        {
            uart_rs232[UART_RS232_REG_TX_DATA] = (uint32_t)(data);
            return;
        }
    } while (1);

}

#define ALTERA_AVALON_UART_CONTROL_RRDY_MSK           (0x80)
#define ALTERA_AVALON_UART_CONTROL_RRDY_OFST          (7)

uint8_t uart_rs232_rx(uint8_t *data, uint32_t timeout)
{
    uint32_t status;

    do {
        status = uart_rs232[UART_RS232_REG_STATUS];
   
        if (status & ALTERA_AVALON_UART_CONTROL_RRDY_MSK)
        {
            uint8_t dummy;
            if (data != NULL)
                *data = (uint8_t)uart_rs232[UART_RS232_REG_RX_DATA];
            else
                dummy = (uint8_t)uart_rs232[UART_RS232_REG_RX_DATA];
            return 0;
        }
    } while (timeout-- > 0);

    return 1;
}


void uart_rs232_tx_frame(uint8_t *data, uint32_t len)
{
    uint32_t i;

    for (i=0;i<len;i++)
    {
        uart_rs232_tx(data[i]);
    }

}

uint32_t uart_rs232_rx_frame(uint8_t *data, uint16_t buffer_size,uint32_t timeout_sof, uint32_t timeout_eof)
{
    uint16_t len = 0;
    uint8_t error;

    error = uart_rs232_rx(&data[len],timeout_sof);

    if (error)
        return 0;

    do {
        len++;
        error = uart_rs232_rx(&data[len],timeout_eof);
    } while (error == 0 && len < buffer_size);

    return len;
}

/*



int 
altera_avalon_uart_read(altera_avalon_uart_state* sp, char* ptr, int len,
  int flags)
{
  int block;
  unsigned int status;

  block = !(flags & O_NONBLOCK);

  do
  {
    status = IORD_ALTERA_AVALON_UART_STATUS(sp->base);

    // clear any error flags

    IOWR_ALTERA_AVALON_UART_STATUS(sp->base, 0);

    if (status & ALTERA_AVALON_UART_CONTROL_RRDY_MSK)
    {
      ptr[0] = IORD_ALTERA_AVALON_UART_RXDATA(sp->base);

      if (!(status & (ALTERA_AVALON_UART_STATUS_PE_MSK | 
      ALTERA_AVALON_UART_STATUS_FE_MSK)))
      {
        return 1;
      }
    }
  }
  while (block);

  ALT_ERRNO = EWOULDBLOCK;
 
  return 0;
}
*/



void uart_rs232_buffer_init(uart_tx_state* state, uint8_t* buffer, uint16_t buffer_size)
{
    state->buffer = buffer;
    state->buffer_size = buffer_size;
    state->tx_index = 0;
    state->tx_size = 0;
}

void uart_rs232_buffer_tx(uart_tx_state* state, uint16_t size)
{
    state->tx_size = size;
    if (size > state->buffer_size)
        state->tx_size = state->buffer_size;
    state->tx_index = 0;
}

void uart_rs232_buffer_tx_process(uart_tx_state* state)
{
    if (state->tx_size != 0)
    {
        uint32_t status = uart_rs232[UART_RS232_REG_STATUS];
        
        if (status & UART_RS232_REG_STATUS_TRDY_MASK)
        {
            uart_rs232[UART_RS232_REG_TX_DATA] = (uint32_t)(state->buffer[state->tx_index++]);
            if (state->tx_index >= state->tx_size)
            {
                state->tx_index = 0;
                state->tx_size = 0;
            }
        }        
    }
}



uint16_t protocolCrc(uint8_t *msg, uint16_t size)
{
	uint16_t crc = 0;
	while( --size>4 )
		crc += msg[size];
	return crc;
}//protocolCrc



void ts_start(uint32_t *ts)
{
    *ts = get_time();
}

void ts_stop(uint32_t *ts)
{
    *ts = get_time()-*ts;
}



uint32_t get_time_latest = 0;
uint32_t get_time_rollover_cnt = 0;

uint64_t get_time64()
{
    uint64_t ret;
    uint32_t cur_time = get_time();

    if (cur_time <= get_time_latest)
        get_time_rollover_cnt++;

    get_time_latest = cur_time;

    ret = ((uint64_t)get_time_rollover_cnt<<32) | cur_time;

    //((uint32_t*)&ret)[0] = cur_time;
    //((uint32_t*)&ret)[1] = get_time_rollover_cnt; 

    return ret;
}


uint32_t ts_freq_to_cycles(uint16_t freq_hz)
{
    uint32_t cycles = 1;
    if (freq_hz != 0)
        cycles = 1000*1000*1000/CPU_CLK_PERIOD_NS/freq_hz;
    return cycles;
}

uint16_t ts_cycles_to_freq(uint32_t cycles)
{
    uint32_t freq_hz = 0;
    if (cycles != 0)
        freq_hz = 1000*1000*1000/CPU_CLK_PERIOD_NS/cycles;
    if (freq_hz > (1<<16)-1)
        freq_hz = (1<<16)-1;
    return (uint16_t)freq_hz;
}

uint8_t ts_is_elapsed(uint32_t ts_start, uint32_t period)
{
    if ((get_time()-ts_start) >= period)
        return 1;
    else
        return 0;
}

void ts_wait_until_elapsed(uint32_t ts_start, uint32_t period)
{
    uint32_t elapsed = get_time()-ts_start;
    uint32_t wait_cycles = 0;
    if (elapsed < period)
        wait_cycles = period-elapsed;

    delay(wait_cycles);
}

/*
*
 * limit(x, min, max) does the following:
 * if x is between min and max, return x
 * if x < min, return min
 * if x > max, return max
--
*/
float limit(float x, float min, float max) 
{
    if (x < min) //(x_integral >= x_min && x_integral <= x_max)
    {
        return min;
    }
    else if (x > max)
    {
        return max;
    }
    return x;
}



/*

-- satlimit(x, min, max) does the following:
 * if x is between min and max, return (x,0)
 * if x < min, return (min, -1)
 * if x > max, return (max, +1)
 */

void satlimit(float x, float min, float max, float* x_integral,float* sat) 
{
    if (x < min) //(x_integral >= x_min && x_integral <= x_max)
    {
        *x_integral = min;
        *sat = -1; 
    }
    else if (x > max)
    {
        *x_integral = max;
        *sat = 1; 
    }
    else
    {
        *x_integral = x;
        *sat = 0; 
    }
}

 
size_t strlen(const char *s) {
    size_t i;
    for (i = 0; s[i] != '\0'; i++) ;
    return i;
}


 /* reverse:  reverse string s in place */
 void reverse(char s[])
 {
     int i, j;
     char c;
 
     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }





// Implementation of itoa()
char* itoa(int num, char* str, int base)
{
    int i = 0;
    unsigned char isNegative = 0;
 
    /* Handle 0 explicitely, otherwise empty string is printed for 0 */
    if (num == 0)
    {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }
 
    // In standard itoa(), negative numbers are handled only with 
    // base 10. Otherwise numbers are considered unsigned.
    if (num < 0 && base == 10)
    {
        isNegative = 1;
        num = -num;
    }
 
    // Process individual digits
    while (num != 0 && i < 10)
    {
        int rem = num % base;
        str[i++] = (rem > 9)? (rem-10) + 'a' : rem + '0';
        num = num/base;
    }
 
    // If number is negative, append '-'
    if (isNegative)
        str[i++] = '-';
 
    str[i] = '\0'; // Append string terminator
 
    // Reverse the string
    reverse(str);
 
    return str;
}




void print_int(int i,uint8_t ret)
{
    char buf[32];
    
    itoa(i,buf,10);
    int s = strlen(buf);
    if (ret)
        buf[s++] = '\n';
    buf[s] = '\0';
    jtaguart_puts(buf);
}

void print_float(float f,uint8_t ret)
{
    int a = (int)f;
    int b = (int)(f*1000)-a*1000;

    if (a == 0 && b < 0)
    {
        jtaguart_putc('-');
    }
    if (b < 0)
        b = -1*b;

    print_int(a,0);
    jtaguart_putc('.');
    print_int(b/100%10,0);
    print_int(b/10%10,0);
    print_int(b%10,ret);

    
}

void *memset(void *dst, int value, size_t size)
{
    if (size)
    {
        uint8_t* d = dst;
        do
        {
            *d++=value;
        } while (--size);
    }
    return dst;
}


