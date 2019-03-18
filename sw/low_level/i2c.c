

#include <tools.h>
#include <i2c.h>

/******************************************************************************
*                                                                             *
* License Agreement                                                           *
*                                                                             *
* Copyright (c) 2016 Altera Corporation, San Jose, California, USA.           *
* All rights reserved.                                                        *
*                                                                             *
* Permission is hereby granted, free of charge, to any person obtaining a     *
* copy of this software and associated documentation files (the "Software"),  *
* to deal in the Software without restriction, including without limitation   *
* the rights to use, copy, modify, merge, publish, distribute, sublicense,    *
* and/or sell copies of the Software, and to permit persons to whom the       *
* Software is furnished to do so, subject to the following conditions:        *
*                                                                             *
* The above copyright notice and this permission notice shall be included in  *
* all copies or substantial portions of the Software.                         *
*                                                                             *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     *
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER         *
* DEALINGS IN THE SOFTWARE.                                                   *
*                                                                             *
* This agreement shall be governed in all respects by the laws of the State   *
* of California and by the laws of the United States of America.              *
*                                                                             *
******************************************************************************/



#define ALT_AVALON_I2C_TFR_CMD_REG                          0
#define ALT_AVALON_I2C_TFR_CMD_STA_OFST                     (9)
#define ALT_AVALON_I2C_TFR_CMD_STA_MSK                      (1 << ALT_AVALON_I2C_TFR_CMD_STA_OFST)
#define ALT_AVALON_I2C_TFR_CMD_STO_OFST                     (8)
#define ALT_AVALON_I2C_TFR_CMD_STO_MSK                      (1 << ALT_AVALON_I2C_TFR_CMD_STO_OFST)
#define ALT_AVALON_I2C_TFR_CMD_AD_OFST                      (1)
#define ALT_AVALON_I2C_TFR_CMD_AD_MSK                       (0x7f << ALT_AVALON_I2C_TFR_CMD_AD_OFST)
#define ALT_AVALON_I2C_TFR_CMD_RW_D_OFST                    (0)
#define ALT_AVALON_I2C_TFR_CMD_RW_D_MSK                     (1 << ALT_AVALON_I2C_TFR_CMD_RW_D_OFST)   
#define ALT_AVALON_I2C_TFR_CMD_ALL_BITS_MSK                 (0x3ff) 

#define ALT_AVALON_I2C_RX_DATA_REG                          1

#define ALT_AVALON_I2C_CTRL_REG                             2
#define ALT_AVALON_I2C_CTRL_RX_DATA_FIFO_THD_OFST           (4)
#define ALT_AVALON_I2C_CTRL_RX_DATA_FIFO_THD_MSK            (3 << ALT_AVALON_I2C_CTRL_RX_DATA_FIFO_THD_OFST)
#define ALT_AVALON_I2C_CTRL_TFR_CMD_FIFO_THD_OFST           (2)
#define ALT_AVALON_I2C_CTRL_TFR_CMD_FIFO_THD_MSK            (3 << ALT_AVALON_I2C_CTRL_TFR_CMD_FIFO_THD_OFST)
#define ALT_AVALON_I2C_CTRL_BUS_SPEED_OFST                  (1)
#define ALT_AVALON_I2C_CTRL_BUS_SPEED_MSK                   (1 << ALT_AVALON_I2C_CTRL_BUS_SPEED_OFST)
#define ALT_AVALON_I2C_CTRL_EN_OFST                         (0)
#define ALT_AVALON_I2C_CTRL_EN_MSK                          (1 << ALT_AVALON_I2C_CTRL_EN_OFST)

#define ALT_AVALON_I2C_STATUS_REG                           5
#define ALT_AVALON_I2C_TFR_CMD_FIFO_LVL_REG                 6
#define ALT_AVALON_I2C_RX_DATA_FIFO_LVL_REG                 7

#define IORD(base,reg) (base[reg])
#define IOWR(base,reg,data) (base[reg] = data)
#define IORMW(base, reg, data, mask)                           (base[reg] = (base[reg] & (~mask)) | (data & mask))

#define ALT_AVALON_I2C_SCL_LOW_REG                          8
#define ALT_AVALON_I2C_SCL_HIGH_REG                         9
#define ALT_AVALON_I2C_SDA_HOLD_REG                         0xa

#define ALT_AVALON_I2C_ISR_TX_READY_OFST                   (0)
#define ALT_AVALON_I2C_ISR_TX_READY_MSK                    (1 << ALT_AVALON_I2C_ISR_TX_READY_OFST)

#define ALT_AVALON_I2C_ISR_NACK_DET_OFST                   (2)
#define ALT_AVALON_I2C_ISR_NACK_DET_MSK                    (1 << ALT_AVALON_I2C_ISR_NACK_DET_OFST)

#define ALT_AVALON_I2C_ISR_REG                              4


#define ALT_AVALON_I2C_ISR_REG                              4
#define ALT_AVALON_I2C_ISR_RX_OVER_OFST                    (4)
#define ALT_AVALON_I2C_ISR_RX_OVER_MSK                     (1 << ALT_AVALON_I2C_ISR_RX_OVER_OFST)
#define ALT_AVALON_I2C_ISR_ARBLOST_DET_OFST                (3)
#define ALT_AVALON_I2C_ISR_ARBLOST_DET_MSK                 (1 << ALT_AVALON_I2C_ISR_ARBLOST_DET_OFST)
#define ALT_AVALON_I2C_ISR_NACK_DET_OFST                   (2)
#define ALT_AVALON_I2C_ISR_NACK_DET_MSK                    (1 << ALT_AVALON_I2C_ISR_NACK_DET_OFST)
#define ALT_AVALON_I2C_ISR_RX_READY_OFST                   (1)
#define ALT_AVALON_I2C_ISR_RX_READY_MSK                    (1 << ALT_AVALON_I2C_ISR_RX_READY_OFST)
#define ALT_AVALON_I2C_ISR_TX_READY_OFST                   (0)
#define ALT_AVALON_I2C_ISR_TX_READY_MSK                    (1 << ALT_AVALON_I2C_ISR_TX_READY_OFST)
#define ALT_AVALON_I2C_ISR_ALLINTS_MSK                     (ALT_AVALON_I2C_ISR_RX_OVER_MSK | ALT_AVALON_I2C_ISR_ARBLOST_DET_MSK |  \
                                                               ALT_AVALON_I2C_ISR_NACK_DET_MSK | ALT_AVALON_I2C_ISR_RX_READY_MSK | \
                                                               ALT_AVALON_I2C_ISR_TX_READY_MSK)
#define ALT_AVALON_I2C_ISR_ALL_CLEARABLE_INTS_MSK          (ALT_AVALON_I2C_ISR_RX_OVER_MSK | ALT_AVALON_I2C_ISR_ARBLOST_DET_MSK |  \
                                                               ALT_AVALON_I2C_ISR_NACK_DET_MSK)    



#define ALT_AVALON_I2C_ISR_ARBLOST_DET_OFST                (3)
#define ALT_AVALON_I2C_ISR_ARBLOST_DET_MSK                 (1 << ALT_AVALON_I2C_ISR_ARBLOST_DET_OFST)

#define ALT_AVALON_I2C_STATUS_CORE_STATUS_OFST              (0)
#define ALT_AVALON_I2C_STATUS_CORE_STATUS_MSK               (1 << ALT_AVALON_I2C_STATUS_CORE_STATUS_OFST)

void i2c_master_enable(volatile uint32_t *base, uint8_t enable)
{
    if (enable)
    {
        IORMW(base,ALT_AVALON_I2C_CTRL_REG,ALT_AVALON_I2C_CTRL_EN_MSK,ALT_AVALON_I2C_CTRL_EN_MSK);
        IOWR(base,ALT_AVALON_I2C_ISR_REG,ALT_AVALON_I2C_ISR_ALL_CLEARABLE_INTS_MSK);
    }
    else
        IORMW(base,ALT_AVALON_I2C_CTRL_REG,0,ALT_AVALON_I2C_CTRL_EN_MSK);
}

void i2c_master_configure(volatile uint32_t *base)
{

    int i;

   // disable
   IORMW(base,ALT_AVALON_I2C_CTRL_REG,0,ALT_AVALON_I2C_CTRL_EN_MSK);

   delay(100);

//config set

     // <lcount> = <internal clock> / 2 * <speed, Hz> 
    uint16_t scl_lcnt = 50000000 / (100000 << 1);
    uint16_t scl_hcnt;
    uint16_t sda_cnt;
#define ALT_AVALON_I2C_DIFF_LCNT_HCNT 10

    scl_hcnt = scl_lcnt + ALT_AVALON_I2C_DIFF_LCNT_HCNT;
    scl_lcnt = scl_lcnt - ALT_AVALON_I2C_DIFF_LCNT_HCNT;
    sda_cnt  = scl_lcnt - (scl_lcnt / 2);


    //ALT_AVALON_I2C_SPEED_STANDARD   = 0, FAST = 1
    #define I2C_SPEED_STANDARD 0x0
    IORMW(base,ALT_AVALON_I2C_CTRL_REG,(I2C_SPEED_STANDARD) << ALT_AVALON_I2C_CTRL_BUS_SPEED_OFST,ALT_AVALON_I2C_CTRL_BUS_SPEED_MSK);

    IOWR(base,ALT_AVALON_I2C_SCL_HIGH_REG,scl_hcnt);
    IOWR(base,ALT_AVALON_I2C_SCL_LOW_REG,scl_lcnt);
    IOWR(base,ALT_AVALON_I2C_SDA_HOLD_REG,sda_cnt);

}

uint8_t i2c_master_check_nack(volatile uint32_t *base)
{    

    if (IORD(base,ALT_AVALON_I2C_ISR_REG) & ALT_AVALON_I2C_ISR_NACK_DET_MSK)
    {
        //print_int(IORD(base,ALT_AVALON_I2C_ISR_REG),1);
        return 1;
    }
    return 0;
}

uint8_t i2c_master_check_arblost(volatile uint32_t *base)
{      
    if (IORD(base,ALT_AVALON_I2C_ISR_REG) & ALT_AVALON_I2C_ISR_ARBLOST_DET_MSK)
    {
        return 1;
    }
    return 0;
}


uint8_t i2c_master_is_busy(volatile uint32_t *base)
{

    if (IORD(base,ALT_AVALON_I2C_STATUS_REG) & ALT_AVALON_I2C_STATUS_CORE_STATUS_MSK)
    {
       return 1;
    }

    return 0;
}



uint8_t i2c_master_cmd_write(volatile uint32_t *base,uint8_t value, uint8_t issue_restart, uint8_t issue_stop)
{
    uint32_t timeout = 0;

    while ((IORD(base,ALT_AVALON_I2C_ISR_REG) & ALT_AVALON_I2C_ISR_TX_READY_MSK)==0) 
    {
        timeout++;
        delay(100);

        if (timeout >= 100000)
        {
            int i;
            /*jtaguart_puts("timeout:\n");
            for (i=0;i<16;i++)
                print_int(IORD(base,i),1);*/

            return 1;
        }

    }

    IOWR(base,ALT_AVALON_I2C_TFR_CMD_REG,value |
                                     (issue_restart << ALT_AVALON_I2C_TFR_CMD_STA_OFST) |
                                     (issue_stop << ALT_AVALON_I2C_TFR_CMD_STO_OFST));
    return i2c_master_check_nack(base) || i2c_master_check_arblost(base);
}


uint8_t i2c_master_cmd_read(volatile uint32_t *base, uint8_t *data)
{
    uint8_t status = 0;
    uint32_t timeout = 100000;


    while (IORD(base,ALT_AVALON_I2C_RX_DATA_FIFO_LVL_REG) == 0)
    {
      if (timeout<10) delay(10000);
      if (--timeout == 0)
      {
        status = 1;
        break;
      }
    }

    *data = (uint8_t)IORD(base,ALT_AVALON_I2C_RX_DATA_REG);

    return status;       
}


                  
/*This function issues a write command and transmits data to the I2C bus. */
uint8_t i2c_master_transmit(volatile uint32_t *base,
                                      uint8_t  addr,
                                      uint8_t *buffer,
                                      uint32_t size,
                                      uint8_t  issue_restart,
                                      uint8_t  issue_stop)
{
    uint8_t status = 0;
    uint8_t timeout=size * 10000;
    
    if (size==0)
    {
      return 0;
    }
    
    /*if a new transaction, enable ip and clear int status*/
    if (!issue_restart) 
    {
      /*enable the ip.  The ip is disabled and enabled for each transaction.*/
      i2c_master_enable(base,1);

    }

    /*Start Write, transmit address. */
    status = i2c_master_cmd_write(base,(addr << 1) | I2C_WRITE,issue_restart, 0);
      
    if (status == 0)
    {
        while ((size > 1) && (status == 0))
        {
            status = i2c_master_cmd_write(base, *buffer, I2C_NO_RESTART, I2C_NO_STOP);
            
            ++buffer;
            --size;
        }

        /* Last byte */
        if (status == 0)
        {
            status = i2c_master_cmd_write(base, *buffer, I2C_NO_RESTART, issue_stop);

            ++buffer;
            --size;
        }
    }
    
    /*if end of transaction, wait until the ip is idle then disable the ip*/
    if ((issue_stop) || (status != 0)) 
    {

        while (i2c_master_is_busy(base))
        {
            if (timeout<10) delay(10000);
            if (--timeout == 0)
            {
               status = 1;
               break;
            }
        }
     
        /*check for a nack error*/
        status += i2c_master_check_nack(base);
        
        /*disable the ip.  The ip is disabled and enabled for each transaction.*/
        i2c_master_enable(base,0);
    }


    return status;
}



/*This function receives one or more data bytes transmitted from a slave in 
 * response to read requests issued from this master. */
uint8_t i2c_master_receive(volatile uint32_t *base,
                                    uint8_t   addr,
                                    uint8_t  *buffer,
                                    uint32_t  size,
                                    uint8_t   issue_restart,
                                    uint8_t   issue_stop)
{
    uint8_t status = 0;
    uint32_t timeout;
    uint32_t bytes_read=0;
    uint32_t bytes_written=0;
    uint32_t temp_bytes_read=0;
    
    if (size==0)
    {
      return 0;
    }
    
    /*if a new transaction, enable ip and clear int status*/
    if (!issue_restart) 
    {
      /*enable the ip.  The ip is disabled and enabled for each transaction.*/
      i2c_master_enable(base,1);
    }

    /*Start Write, transmit address. */
    status = i2c_master_cmd_write(base,(addr << 1) | I2C_READ,issue_restart, 0);

    if (status == 0)
    {
        while ((bytes_written < (size-1)) && (status == 0))
        {
            status = i2c_master_cmd_write(base, 0x00, I2C_NO_RESTART, I2C_NO_STOP);
            bytes_written++;
            if (status == 0)
            {
               status = i2c_master_cmd_read(base, buffer);
               buffer+=temp_bytes_read;
               bytes_read+=temp_bytes_read;
            }
        }

        /* Last byte */
        if (status == 0)
        {
            status = i2c_master_cmd_write(base, 0x00, I2C_NO_RESTART, issue_stop);
        }
    }
    
    while ((bytes_read < size) && (status==0)) 
    {
        status = i2c_master_cmd_read(base, buffer);
        buffer++;
        bytes_read++;
    }

    /*if end of transaction, wait until the ip is idle then disable the ip*/
    if ((issue_stop) || (status != 0)) 
    {
        timeout=10000 * size;
        while (i2c_master_is_busy(base))
        {
            if (timeout<10) delay(10000);
            if (--timeout == 0)
            {
               status = 1;
               break;
            }
        }

        /*check for a nack error*/
        status += i2c_master_check_nack(base);
        
        /*disable the ip.  The ip is disabled and enabled for each transaction.*/
        i2c_master_enable(base,0);
    }

    return status;
}


uint8_t i2c_master_tx(volatile uint32_t *base,
                               uint8_t   addr,
                               uint8_t  *buffer,
                               uint32_t  size)
{
    uint8_t status;
    uint32_t retry=20;  
    
    while (retry--)
    {
      if (retry<10) delay(10000);

      {
         status = i2c_master_transmit(base,addr, buffer, size, I2C_NO_RESTART, I2C_STOP);
      }
      if (status!=0) continue;
      break;
    }

    return status;
}        



/*transmit, restart, recieve function using retry and optionally interrupts */
uint8_t i2c_master_tx_rx(volatile uint32_t *base,
                                             uint8_t   addr,
                                             uint8_t  *txbuffer,
                                             uint32_t  txsize,
                                             uint8_t  *rxbuffer,
                                             uint32_t  rxsize)                                       
{
    uint8_t status;
    uint32_t retry=20;  
    
    {
      while (retry--) 
      {
        if (retry<10) delay(10000);      
        status = i2c_master_transmit(base,addr, txbuffer, txsize, I2C_NO_RESTART, I2C_NO_STOP);     
        if (status!=0) continue;

        status = i2c_master_receive(base,addr, rxbuffer, rxsize, I2C_RESTART, I2C_STOP);     
        if (status!=0) continue;
  
        break;
      }
    }
    
    return status;
}           

