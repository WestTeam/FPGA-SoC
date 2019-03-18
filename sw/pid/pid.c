
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

 #include "tools.h"

#include <quadramp.h>

typedef struct pid_mapping
{
    uint8_t reset;
    uint8_t pgm_id;
    uint8_t arg[2];
    uint16_t freq_hz_cfg;
    uint16_t freq_hz_latest;
    uint8_t enable;
    uint8_t override;
    uint8_t inverted;
    uint8_t unused;
    float P;
    float I;
    float D;
    float speed;
    float acc;
    uint32_t sat;
    union {
        float f_measure;
        int32_t measure;
    };
    union {
        float f_target;
        int32_t target;
    };
    int32_t output;
} __attribute__((packed)) pid_mapping_t;


typedef struct pid_data
{
    // OUTPUT PROCESSED
    uint8_t enabled;
    uint8_t inverted;

    float f_target;
    int32_t target;

    float f_measure;

    float Kp;
    float Ki2;
    float Kd;
    float speed;
    float acc;
    uint32_t sat;


    float error;
    float saturated;
    float x_integral;
    float x_min;
    float x_max;

    float x;

    struct quadramp_filter qr;
   
}pid_data_t;


void pid_init(pid_data_t* data)
{
    memset((void*)data,0,sizeof(*data));
    quadramp_init(&data->qr);
}



void pid_stop(pid_data_t* data,volatile pid_mapping_t* regs)
{
    data->enabled = 0;                    
    data->x_integral = 0;
    data->saturated = 0;    
    data->error = 0;
    data->x = 0;
    regs->output = 0;
}


void pid_update_input(pid_data_t* data,volatile pid_mapping_t* regs)
{
    
    if (regs->arg[0] == 0)
    {
//        if (debug == 2)
//        {
//            print_float(data->f_target,1);
//            print_float(regs->f_measure,1);
//        }
        data->f_target = regs->f_target;
        data->f_measure = regs->f_measure;
        //data->error = data->f_target - data->f_measure;

    }
    else    
    {
//        if (debug == 2)
//        {
//            print_int((int)data->target,1);
//            print_int((int)regs->measure,1);
//        }
        data->target = regs->target;
        data->f_target = (float)regs->target;
        data->f_measure = (float)regs->measure;
        //int32_t vi;
        //vi = data->target-regs->measure;
        //data->error = (float)vi;
    }
    

}



void pid_start(pid_data_t* data,volatile pid_mapping_t* regs)
{
    data->enabled = 1;
    pid_update_input(data,regs);
    quadramp_reset(&data->qr);
    quadramp_set_position(&data->qr,data->f_measure);
}


void pid_update_config(pid_data_t* data,volatile pid_mapping_t* regs)
{
    
    uint8_t enable = regs->enable;
    float P = regs->P;
    float I = regs->I;
    float D = regs->D;
    float speed = regs->speed;
    float acc = regs->acc;
    uint32_t sat = regs->sat;  

    data->inverted = regs->inverted;

    if (P != data->Kp || I != data->Ki2 || D != data->Kd)
    {
        pid_stop(data,regs);
        data->Kp = P;
        data->Ki2 = I;
        data->Kd = D;
        if (enable == 1)
            pid_start(data,regs);
    }

    if (speed != data->speed || acc != data->acc)
    {
        data->speed = speed;
        data->acc = acc;

        quadramp_set_2nd_order_vars(&data->qr,acc,acc);
        quadramp_set_1st_order_vars(&data->qr,speed,speed);
/*
        print_float(data->f_target,0);
        jtaguart_puts("|");
        print_float(data->qr.previous_out,0);
        jtaguart_puts("|");
        print_float(speed,0);
        jtaguart_puts("|");
        print_float(acc,1);
*/
    }

    if (enable == 1 && data->enabled == 0)
        pid_start(data,regs);

    if (enable == 0 && data->enabled == 1)
        pid_stop(data,regs);

    if (sat != data->sat)
    {
        data->sat = sat;
        data->x_max = (float)sat;
        data->x_min = -(float)sat;               
    }
}





void pid_processing(pid_data_t* data, float target)
{
          
/*
        if ((sat < 0 && e < 0) || (sat > 0 && e > 0))
         ; 
        -- do nothing if there is saturation, and error is in the same direction;
        -- if you're careful you can implement as "if (sat*e > 0)"
        --
        else
         x_integral = x_integral + Ki2*e;
        (x_integral,sat) = satlimit(x_integral, x_minimum, x_maximum);
        x = limit(Kp*e + x_integral, x_minimum, x_maximum)
*/
    
//    if (debug == 2)
//        print_int((int)pid.error,1);

    data->error = target - data->f_measure;

    if (data->saturated*data->error > 0)
    {

    }
    else
    {
        data->x_integral = data->x_integral + data->Ki2*data->error;
        satlimit(data->x_integral,data->x_min,data->x_max, &data->x_integral,&data->saturated); 
    }


//    if (debug == 2)
//        print_int((int)pid.x_integral,1);
   
    data->x = limit(data->Kp*data->error+data->x_integral,data->x_min,data->x_max);

//    if (debug == 2)
//        print_int((int)pid.x,1);

}

void pid_update_output(pid_data_t* data,volatile pid_mapping_t* regs)
{
    if (data->inverted)
        regs->output = -(int32_t)data->x;
    else
        regs->output = (int32_t)data->x;
}


typedef struct {
    ProtocolHeader hdr;
    uint32_t timestamp; // 1 = 20ns
    float speed;
    float acc;
    float target;
    float filtered_target; // after quadramp
    float measure;
    float output;
} __attribute__((packed)) pid_debug;




volatile int* pio_n = (volatile int*)(0x01000000);

int main()
{
    volatile pid_mapping_t* regs = (pid_mapping_t*)pio_n;
    memset((void*)regs,0,sizeof(*regs));

    pid_data_t pid;
    uart_tx_state tx_state;
    pid_debug tx_buffer;

    float scale = 100.00;

    char chr;    

    uint8_t debug = 0;

    uint32_t ts[2];
    #define TS_UPDATE 0
    #define TS_DURATION 1

    
    uint16_t freq_hz = 0;
    uint32_t period_cycles = 0;
    uint32_t period_latest;

    ts_start(&ts[TS_UPDATE]);

    uart_rs232_configure(50000000/1000000);

    uart_rs232_buffer_init(&tx_state,(uint8_t*)&tx_buffer,sizeof(tx_buffer));

    tx_buffer.hdr.fanion = PROTOCOL_FANION;
    tx_buffer.hdr.size = sizeof(pid_debug);
    tx_buffer.hdr.crc = 0;
    tx_buffer.hdr.id = 1;

    for(;;) {

        uart_rs232_buffer_tx_process(&tx_state);

        pid_update_config(&pid,regs);

        if (freq_hz != regs->freq_hz_cfg)
        {
            freq_hz = regs->freq_hz_cfg;
            period_cycles = ts_freq_to_cycles(freq_hz);
        }

        if (pid.enabled == 1)//(regs->period_cfg != 0)
        {

            if (ts_is_elapsed(ts[TS_UPDATE],(uint32_t)5000))//period_cycles))
            {
                ts_start(&ts[TS_DURATION]);
                ts[TS_UPDATE] = ts[TS_DURATION];

                pid_update_input(&pid,regs);

                float f_target_filtered = quadramp_do_filter(&pid.qr,pid.f_target);

                pid_processing(&pid,f_target_filtered);

                pid_update_output(&pid,regs);


/*
    float speed;
    float acc;
    float target;
    float filtered_target; // after quadramp
    float measure;
    float output;
*/
                if (tx_state.tx_size == 0)
                {
                    tx_buffer.timestamp = ts[TS_UPDATE];
                    tx_buffer.speed = pid.speed;
                    tx_buffer.acc   = pid.acc;
                    tx_buffer.target = pid.f_target;
                    tx_buffer.filtered_target = f_target_filtered;
                    tx_buffer.measure = pid.f_measure;
                    tx_buffer.output = pid.x;

                    tx_buffer.hdr.crc = protocolCrc((uint8_t*)&tx_buffer,sizeof(pid_debug));
                    
                    uart_rs232_buffer_tx(&tx_state,sizeof(pid_debug));
                }

                ts_stop(&ts[TS_DURATION]);
                period_latest = ts[TS_DURATION];
                regs->freq_hz_latest = ts_cycles_to_freq(period_latest);    
  
            }
        } else {
            ts_start(&ts[TS_UPDATE]);
            regs->output = 0;
        }


        chr = jtaguart_getchar();
        if (chr != 0)
        {
            switch (chr)
            {

                case 'r':
                    // reset the current module
                    return 0;
/*
                case 'e':
                    pid_stop(&pid,regs);

                    //jtaguart_puts("----- PID DISABLED ----\n");
                    break;

                case 'E':
                    pid_start(&pid,regs);
                    //jtaguart_puts("----- PID ENABLED ----\n");
                    break;
*/
                case 'p':
/*
                    jtaguart_puts("----- PID Internals ----\n");
                    if (regs->arg[0] == 0)
                    {
                        print_float(pid.f_target,1);                        
                        print_float(regs->f_measure,1);                        
                    } else {
                        print_int(pid.target,1);                        
                        print_int(regs->measure,1); 
                    }
                    print_float(pid.qr.previous_out,1);                        
                    print_float(pid.qr.previous_var,1);                        
                    print_int(regs->enable,1);
                    print_int(regs->override,1);
                    print_float(regs->speed,1);
                    print_float(regs->acc,1);
                    print_int(regs->sat,1);

                    print_float(pid.error,1);
                    print_float(pid.x_integral,1);
                    print_float(pid.saturated,1);
                    print_float(pid.x,1);
                    print_int(period_latest,1);
*/
                    break;

/*                case 'd':
                    if (debug)
                        debug--;
                    break;

                case 'D':
                    debug++;
                    break;

                case 's':
                    scale/=10.0;
                    print_float(scale,1);
                    break;
                case 'S':
                    scale*=10.0;
                    print_float(scale,1);
                    break; 

                case 'k':
                    pid.Kp-=scale;
                    print_float(pid.Kp,1);
                    break;
                case 'K':
                    pid.Kp+=scale;
                    print_float(pid.Kp,1);
                    break; 


                case 'i':
                    pid.Ki2-=scale;
                    print_float(pid.Ki2,1);
                    break;
                case 'I':
                    pid.Ki2+=scale;
                    print_float(pid.Ki2,1);
                    break; 


                case 't':
                    if (regs->arg[0] == 0)
                    {
                        pid.f_target-=scale;
                        print_float(pid.f_target,1);
                    } else {
                        pid.target-=(uint32_t)scale;
                        print_int(pid.target,1);
                    }
                    break;

                case 'T':
                    if (regs->arg[0] == 0)
                    {
                        pid.f_target+=scale;
                        print_float(pid.f_target,1);
                    } else {
                        pid.target+=(uint32_t)scale;
                        print_int(pid.target,1);
                    }
                    break;

                case 'o':
                    pid.inverted=~pid.inverted;
                    if (pid.inverted)
                        jtaguart_puts("Output inverted\n");               
                    else
                        jtaguart_puts("Output normal\n");                                   
                    break;
*/

            }

        }

	}
}

