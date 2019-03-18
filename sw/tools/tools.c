
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

 #include "tools.h"


volatile uint32_t* uart_jtag = (volatile uint32_t*)(0x01000400);

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

uint32_t uart_rs232_rx_frame(uint8_t *data, uint32_t timeout_sof, uint32_t timeout_eof)
{
    uint32_t len = 0;
    uint8_t error;

    error = uart_rs232_rx(&data[len],timeout_sof);

    if (error)
        return 0;

    do {
        len++;
        error = uart_rs232_rx(&data[len],timeout_eof);
    } while (error == 0);

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


#define M_PI           3.14159265358979323846

typedef union
{
  float value;
  uint32_t word;
} ieee_float_shape_type;

/* Get a 32 bit int from a float.  */
#ifndef GET_FLOAT_WORD
# define GET_FLOAT_WORD(i,d)					\
do {								\
  ieee_float_shape_type gf_u;					\
  gf_u.value = (d);						\
  (i) = gf_u.word;						\
} while (0)
#endif

/* Set a float from a 32 bit int.  */
#ifndef SET_FLOAT_WORD
# define SET_FLOAT_WORD(d,i)					\
do {								\
  ieee_float_shape_type sf_u;					\
  sf_u.word = (i);						\
  (d) = sf_u.value;						\
} while (0)
#endif


/* k_cosf.c -- float version of k_cos.c
 * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
 */

/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */



static const float
one =  1.0000000000e+00, /* 0x3f800000 */
C1  =  4.1666667908e-02, /* 0x3d2aaaab */
C2  = -1.3888889225e-03, /* 0xbab60b61 */
C3  =  2.4801587642e-05, /* 0x37d00d01 */
C4  = -2.7557314297e-07, /* 0xb493f27c */
C5  =  2.0875723372e-09, /* 0x310f74f6 */
C6  = -1.1359647598e-11; /* 0xad47d74e */

float __kernel_cosf(float x, float y)
{
	float a,hz,z,r,qx;
	int32_t ix;
	GET_FLOAT_WORD(ix,x);
	ix &= 0x7fffffff;			/* ix = |x|'s high word*/
	if(ix<0x32000000) {			/* if x < 2**27 */
	    if(((int)x)==0) return one;		/* generate inexact */
	}
	z  = x*x;
	r  = z*(C1+z*(C2+z*(C3+z*(C4+z*(C5+z*C6)))));
	if(ix < 0x3e99999a) 			/* if |x| < 0.3 */
	    return one - ((float)0.5*z - (z*r - x*y));
	else {
	    if(ix > 0x3f480000) {		/* x > 0.78125 */
		qx = (float)0.28125;
	    } else {
	        SET_FLOAT_WORD(qx,ix-0x01000000);	/* x/4 */
	    }
	    hz = (float)0.5*z-qx;
	    a  = one-qx;
	    return a - (hz - (z*r-x*y));
	}
}



/* k_sinf.c -- float version of k_sin.c
 * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
 */

/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

static const float
half =  5.0000000000e-01,/* 0x3f000000 */
S1  = -1.6666667163e-01, /* 0xbe2aaaab */
S2  =  8.3333337680e-03, /* 0x3c088889 */
S3  = -1.9841270114e-04, /* 0xb9500d01 */
S4  =  2.7557314297e-06, /* 0x3638ef1b */
S5  = -2.5050759689e-08, /* 0xb2d72f34 */
S6  =  1.5896910177e-10; /* 0x2f2ec9d3 */



float __kernel_sinf(float x, float y, int iy)
{
	float z,r,v;
	int32_t ix;
	GET_FLOAT_WORD(ix,x);
	ix &= 0x7fffffff;			/* high word of x */
	if(ix<0x32000000)			/* |x| < 2**-27 */
	  {
	    //math_check_force_underflow (x);
	    if ((int) x == 0)
	      return x;		/* generate inexact */
	  }
	z	=  x*x;
	v	=  z*x;
	r	=  S2+z*(S3+z*(S4+z*(S5+z*S6)));
	if(iy==0) return x+v*(S1+z*r);
	else      return x-((z*(half*y-v*r)-y)-v*S1);
}

/* s_sinf.c -- float version of s_sin.c.
 * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
 */

/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

float __sinf(float x)
{
	//float y[2];
    float z=0.0;
	int32_t ix;
    //int32_t n;

	GET_FLOAT_WORD(ix,x);

    uint8_t neg = ix < 0;

    //print_int(ix,1);

    /* |x| ~< pi/4 */
	ix &= 0x7fffffff;
	if(ix <= 0x3f490fd8) return __kernel_sinf(x,z,0);

    /* sin(Inf or NaN) is NaN */
	else if (ix>=0x7f800000) {
	  //if (ix == 0x7f800000)
	    //__set_errno (EDOM);
	  return x-x;
	}

    /* argument reduction needed */
	else {
        //PI/4   = 1061752795
        //PI/2   = 1070141403
        //3*PI/4 = 1075235812
        //PI     = 1078530011
        if (neg == 0)
        {
            if (ix < 1070141403)
            {
                return __kernel_cosf(M_PI/2-x,0.0);
            } 
            else if (ix < 1075235812)
            {
                return __kernel_cosf(x-M_PI/2,0.0);
            }
            else
            {
                return __kernel_sinf(M_PI-x,z,0);
            }    
        } else {
            if (ix < 1070141403)
            {
                return -__kernel_cosf(M_PI/2+x,0.0);
            } 
            else if (ix < 1075235812)
            {
                return -__kernel_cosf(-x-M_PI/2,0.0);
            }
            else
            {
                return -__kernel_sinf(M_PI+x,z,0);
            }    
        }   
        /*
	    n = __ieee754_rem_pio2f(x,y);
	    switch(n&3) {
		case 0: return  __kernel_sinf(y[0],y[1],1);
		case 1: return  __kernel_cosf(y[0],y[1]);
		case 2: return -__kernel_sinf(y[0],y[1],1);
		default:
			return -__kernel_cosf(y[0],y[1]); 
	    } */
	}
}

/* s_cosf.c -- float version of s_cos.c.
 * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
 */

/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */
float __cosf(float x)
{
	//float y[2];
    float z=0.0;
	int32_t ix;
    //int32_t n;

	GET_FLOAT_WORD(ix,x);

    uint8_t neg = ix < 0;

    /* |x| ~< pi/4 */
	ix &= 0x7fffffff;
	if(ix <= 0x3f490fd8) return __kernel_cosf(x,z);

    /* cos(Inf or NaN) is NaN */
	else if (ix>=0x7f800000) {
	  //if (ix == 0x7f800000)
	    //__set_errno (EDOM);
	  return x-x;
	}

    /* argument reduction needed */
	else {
        //PI/4   = 1061752795
        //PI/2   = 1070141403
        //3*PI/4 = 1075235812
        //PI     = 1078530011
        if (neg == 1)
        {
            x = -x;
        }

        if (ix < 1070141403)
        {
            return __kernel_sinf(M_PI/2-x,0.0,0);
        } 
        else if (ix < 1075235812)
        {
            return -__kernel_sinf(x-M_PI/2,0.0,0);
        }
        else
        {
            return -__kernel_cosf(M_PI-x,0.0);
        }       

        /*
	    n = 0;//__ieee754_rem_pio2f(x,y);
	    switch(n&3) {
		case 0: return  __kernel_cosf(y[0],y[1]);
		case 1: return -__kernel_sinf(y[0],y[1],1);
		case 2: return -__kernel_cosf(y[0],y[1]);
		default:
		        return  __kernel_sinf(y[0],y[1],1);
	    } */
	}
}


/* e_sqrtf.c -- float version of e_sqrt.c.
 * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
 */

/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

static	const float	fone	= 1.0,tiny=1.0e-30;

float
__ieee754_sqrtf(float x)
{
	float z;
	int32_t sign = (int)0x80000000;
	int32_t ix,s,q,m,t,i;
	uint32_t r;

	GET_FLOAT_WORD(ix,x);

    /* take care of Inf and NaN */
	if((ix&0x7f800000)==0x7f800000) {
	    return x*x+x;		/* sqrt(NaN)=NaN, sqrt(+inf)=+inf
					   sqrt(-inf)=sNaN */
	}
    /* take care of zero */
	if(ix<=0) {
	    if((ix&(~sign))==0) return x;/* sqrt(+-0) = +-0 */
	    else if(ix<0)
		return (x-x)/(x-x);		/* sqrt(-ve) = sNaN */
	}
    /* normalize x */
	m = (ix>>23);
	if(m==0) {				/* subnormal x */
	    for(i=0;(ix&0x00800000)==0;i++) ix<<=1;
	    m -= i-1;
	}
	m -= 127;	/* unbias exponent */
	ix = (ix&0x007fffff)|0x00800000;
	if(m&1)	/* odd m, double x to make it even */
	    ix += ix;
	m >>= 1;	/* m = [m/2] */

    /* generate sqrt(x) bit by bit */
	ix += ix;
	q = s = 0;		/* q = sqrt(x) */
	r = 0x01000000;		/* r = moving bit from right to left */

	while(r!=0) {
	    t = s+r;
	    if(t<=ix) {
		s    = t+r;
		ix  -= t;
		q   += r;
	    }
	    ix += ix;
	    r>>=1;
	}

    /* use floating add to find out rounding direction */
	if(ix!=0) {
	    z = fone-tiny; /* trigger inexact flag */
	    if (z>=fone) {
		z = fone+tiny;
		if (z>fone)
		    q += 2;
		else
		    q += (q&1);
	    }
	}
	ix = (q>>1)+0x3f000000;
	ix += (m <<23);
	SET_FLOAT_WORD(z,ix);
	return z;
}

/* s_fabsf.c -- float version of s_fabs.c.
 * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
 */

/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

float __fabsf(float x)
{
  return __builtin_fabsf (x);
}


/* s_atanf.c -- float version of s_atan.c.
 * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
 */

/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */
#define FLT_MIN 1.0E-37
#define DBL_MIN 1.0E-37
#define LDBL_MIN 1.0E-37

# define math_force_eval(x) \
({ __typeof (x) __x = (x); __asm __volatile__ ("" : : "m" (__x)); })

#define __MATH_TG(TG_ARG, FUNC, ARGS)                                  \
  (sizeof (TG_ARG) == sizeof (float) ? FUNC ## f ARGS : FUNC ARGS)


#define fabs_tg(x) __MATH_TG ((x), (__typeof (x)) __builtin_fabs, (x))
#define min_of_type(type) __builtin_choose_expr         \
  (__builtin_types_compatible_p (type, float),          \
   FLT_MIN,                                             \
   __builtin_choose_expr                                \
   (__builtin_types_compatible_p (type, double),        \
    DBL_MIN, LDBL_MIN))



/* If X (which is not a NaN) is subnormal, force an underflow
   exception.  */
#define math_check_force_underflow(x)                           \
  do                                                            \
    {                                                           \
      __typeof (x) force_underflow_tmp = (x);                   \
      if (fabs_tg (force_underflow_tmp)                         \
          < min_of_type (__typeof (force_underflow_tmp)))       \
        {                                                       \
          __typeof (force_underflow_tmp) force_underflow_tmp2   \
            = force_underflow_tmp * force_underflow_tmp;        \
          math_force_eval (force_underflow_tmp2);               \
        }                                                       \
    }                                                           \
  while (0)



static const float atanhi[] = {
  4.6364760399e-01, /* atan(0.5)hi 0x3eed6338 */
  7.8539812565e-01, /* atan(1.0)hi 0x3f490fda */
  9.8279368877e-01, /* atan(1.5)hi 0x3f7b985e */
  1.5707962513e+00, /* atan(inf)hi 0x3fc90fda */
};

static const float atanlo[] = {
  5.0121582440e-09, /* atan(0.5)lo 0x31ac3769 */
  3.7748947079e-08, /* atan(1.0)lo 0x33222168 */
  3.4473217170e-08, /* atan(1.5)lo 0x33140fb4 */
  7.5497894159e-08, /* atan(inf)lo 0x33a22168 */
};

static const float aT[] = {
  3.3333334327e-01, /* 0x3eaaaaaa */
 -2.0000000298e-01, /* 0xbe4ccccd */
  1.4285714924e-01, /* 0x3e124925 */
 -1.1111110449e-01, /* 0xbde38e38 */
  9.0908870101e-02, /* 0x3dba2e6e */
 -7.6918758452e-02, /* 0xbd9d8795 */
  6.6610731184e-02, /* 0x3d886b35 */
 -5.8335702866e-02, /* 0xbd6ef16b */
  4.9768779427e-02, /* 0x3d4bda59 */
 -3.6531571299e-02, /* 0xbd15a221 */
  1.6285819933e-02, /* 0x3c8569d7 */
};

static const float
//one   = 1.0,
huge   = 1.0e30;

float __atanf(float x)
{
	float w,s1,s2,z;
	int32_t ix,hx,id;

	GET_FLOAT_WORD(hx,x);
	ix = hx&0x7fffffff;
	if(ix>=0x4c000000) {	/* if |x| >= 2^25 */
	    if(ix>0x7f800000)
		return x+x;		/* NaN */
	    if(hx>0) return  atanhi[3]+atanlo[3];
	    else     return -atanhi[3]-atanlo[3];
	} if (ix < 0x3ee00000) {	/* |x| < 0.4375 */
	    if (ix < 0x31000000) {	/* |x| < 2^-29 */
		math_check_force_underflow (x);
		if(huge+x>fone) return x;	/* raise inexact */
	    }
	    id = -1;
	} else {
	x = __fabsf(x);
	if (ix < 0x3f980000) {		/* |x| < 1.1875 */
	    if (ix < 0x3f300000) {	/* 7/16 <=|x|<11/16 */
		id = 0; x = ((float)2.0*x-fone)/((float)2.0+x);
	    } else {			/* 11/16<=|x|< 19/16 */
		id = 1; x  = (x-fone)/(x+fone);
	    }
	} else {
	    if (ix < 0x401c0000) {	/* |x| < 2.4375 */
		id = 2; x  = (x-(float)1.5)/(fone+(float)1.5*x);
	    } else {			/* 2.4375 <= |x| < 2^66 */
		id = 3; x  = -(float)1.0/x;
	    }
	}}
    /* end of argument reduction */
	z = x*x;
	w = z*z;
    /* break sum from i=0 to 10 aT[i]z**(i+1) into odd and even poly */
	s1 = z*(aT[0]+w*(aT[2]+w*(aT[4]+w*(aT[6]+w*(aT[8]+w*aT[10])))));
	s2 = w*(aT[1]+w*(aT[3]+w*(aT[5]+w*(aT[7]+w*aT[9]))));
	if (id<0) return x - x*(s1+s2);
	else {
	    z = atanhi[id] - ((x*(s1+s2) - atanlo[id]) - x);
	    return (hx<0)? -z:z;
	}
}

/* e_atan2f.c -- float version of e_atan2.c.
 * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
 */

/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

static const float
//tiny  = 1.0e-30,
zero  = 0.0,
pi_o_4  = 7.8539818525e-01,  /* 0x3f490fdb */
pi_o_2  = 1.5707963705e+00,  /* 0x3fc90fdb */
pi      = 3.1415927410e+00,  /* 0x40490fdb */
pi_lo   = -8.7422776573e-08; /* 0xb3bbbd2e */

float
__ieee754_atan2f (float y, float x)
{
	float z;
	int32_t k,m,hx,hy,ix,iy;

	GET_FLOAT_WORD(hx,x);
	ix = hx&0x7fffffff;
	GET_FLOAT_WORD(hy,y);
	iy = hy&0x7fffffff;
	if((ix>0x7f800000)||
	   (iy>0x7f800000))	/* x or y is NaN */
	   return x+y;
	if(hx==0x3f800000) return __atanf(y);   /* x=1.0 */
	m = ((hy>>31)&1)|((hx>>30)&2);	/* 2*sign(x)+sign(y) */

    /* when y = 0 */
	if(iy==0) {
	    switch(m) {
		case 0:
		case 1: return y;	/* atan(+-0,+anything)=+-0 */
		case 2: return  pi+tiny;/* atan(+0,-anything) = pi */
		case 3: return -pi-tiny;/* atan(-0,-anything) =-pi */
	    }
	}
    /* when x = 0 */
	if(ix==0) return (hy<0)?  -pi_o_2-tiny: pi_o_2+tiny;

    /* when x is INF */
	if(ix==0x7f800000) {
	    if(iy==0x7f800000) {
		switch(m) {
		    case 0: return  pi_o_4+tiny;/* atan(+INF,+INF) */
		    case 1: return -pi_o_4-tiny;/* atan(-INF,+INF) */
		    case 2: return  (float)3.0*pi_o_4+tiny;/*atan(+INF,-INF)*/
		    case 3: return (float)-3.0*pi_o_4-tiny;/*atan(-INF,-INF)*/
		}
	    } else {
		switch(m) {
		    case 0: return  zero  ;	/* atan(+...,+INF) */
		    case 1: return -zero  ;	/* atan(-...,+INF) */
		    case 2: return  pi+tiny  ;	/* atan(+...,-INF) */
		    case 3: return -pi-tiny  ;	/* atan(-...,-INF) */
		}
	    }
	}
    /* when y is INF */
	if(iy==0x7f800000) return (hy<0)? -pi_o_2-tiny: pi_o_2+tiny;

    /* compute y/x */
	k = (iy-ix)>>23;
	if(k > 60) z=pi_o_2+(float)0.5*pi_lo;	/* |y/x| >  2**60 */
	else if(hx<0&&k<-60) z=0.0;	/* |y|/x < -2**60 */
	else z=__atanf(__fabsf(y/x));	/* safe to do y/x */
	switch (m) {
	    case 0: return       z  ;	/* atan(+,+) */
	    case 1: {
		      uint32_t zh;
		      GET_FLOAT_WORD(zh,z);
		      SET_FLOAT_WORD(z,zh ^ 0x80000000);
		    }
		    return       z  ;	/* atan(-,+) */
	    case 2: return  pi-(z-pi_lo);/* atan(+,-) */
	    default: /* case 3 */
		    return  (z-pi_lo)-pi;/* atan(-,-) */
	}
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


