
 #include <string.h>
 #include <stdint.h>
 #include <limits.h>

 #include "tools.h"

#define M_PI           3.14159265358979323846

#define DEG(x) ((x) * (180.0 / M_PI))

typedef struct odo_mapping
{
    uint8_t reset;
    uint8_t pgm_id;
    uint8_t arg[2];
    uint16_t freq_hz_cfg;
    uint16_t freq_hz_latest;
    float c_wheel_axe_mm;
    float c_wheel_mm_per_tick[2];
    float m_wheel_axe_mm;
    float m_wheel_mm_per_tick[2];
    union {
        uint16_t c_qei_value[2];
        float odo_sum_m_distance;
    };
    union {
        uint16_t m_qei_value[2];
        float odo_sum_m_angle;
    };
    float odo_sum_c_distance;
    float odo_sum_c_angle;
    uint8_t odo_pos_valid;
    uint8_t odo_pos_id;
    int16_t odo_pos_teta;
    int16_t odo_pos_x;
    int16_t odo_pos_y;
    float   odo_pos_sum_distance;
    float   odo_pos_sum_angle;
} __attribute__((packed)) odo_mapping_t;


#define LEFT 0
#define RIGHT 1

typedef struct wheel_config
{
    float axe_mm;
    float axe_mm_inv; // to avoid live division
    float mm_per_tick[2]; // used to compute quickly the distance
} wheel_config_t;


typedef struct odo_state
{
    // INPUT
    uint16_t qei_latest[2];

    // INTERNALS
    int32_t  qei_sum[2]; // sum from the init
    int32_t  qei_sum_latest[2]; // latest processed value for position

    // CONFIGS
    wheel_config_t wheel;

    // OUTPUT RAW
    float dist_sum;
    float angle_sum;

} odo_state_t;

typedef struct odo_data
{
    odo_state_t cs;
    odo_state_t ms;

    uint16_t freq_hz;
    uint32_t period_cycles;
    
    // OUTPUT PROCESSED
    float x;
    float y;
    float teta_rad; // modulo PI
    int16_t teta_deg; // (1 = 0.1deg)
    uint8_t id;


} odo_data_t;


void odometry_init(volatile odo_mapping_t* regs, odo_data_t* data)
{
    // init structures
    memset((void*)data,0,sizeof(*data));
    memset((void*)regs,0,sizeof(*regs));

    print_int(regs->m_qei_value[0],1);

    // read current QEI
    data->ms.qei_latest[0] = regs->m_qei_value[0];
    data->ms.qei_latest[1] = regs->m_qei_value[1];

    data->cs.qei_latest[0] = regs->c_qei_value[0];
    data->cs.qei_latest[1] = regs->c_qei_value[1];

    data->ms.wheel.axe_mm = 274.0;
    data->ms.wheel.axe_mm_inv = 1.0/data->ms.wheel.axe_mm;
    data->ms.wheel.mm_per_tick[0] = 1.08*72.0*M_PI/(1024.0*728.0/45.0);
    data->ms.wheel.mm_per_tick[1] = 1.08*72.0*M_PI/(1024.0*728.0/45.0);

    data->cs.wheel.axe_mm = 188.0;
    data->cs.wheel.axe_mm_inv = 1.0/data->cs.wheel.axe_mm;
    data->cs.wheel.mm_per_tick[0] = 32.0*M_PI/(1024.0);
    data->cs.wheel.mm_per_tick[1] = 32.0*M_PI/(1024.0);
 
    
}


void odometry_update_config(odo_data_t* data,volatile odo_mapping_t* regs)
{
    uint8_t i;

    if (regs->m_wheel_axe_mm != 0.0 && regs->m_wheel_axe_mm != data->ms.wheel.axe_mm)
    {
        data->ms.wheel.axe_mm = regs->m_wheel_axe_mm;
        data->ms.wheel.axe_mm_inv = 1.0/regs->m_wheel_axe_mm;
    }
    for (i=0;i<2;i++)
    {
        if (regs->m_wheel_mm_per_tick[i] != 0.0 && regs->m_wheel_mm_per_tick[i] != data->ms.wheel.mm_per_tick[i])
            data->ms.wheel.mm_per_tick[i] = regs->m_wheel_mm_per_tick[i];
    }

    if (regs->c_wheel_axe_mm != 0.0 && regs->c_wheel_axe_mm != data->cs.wheel.axe_mm)
    {
        data->cs.wheel.axe_mm = regs->c_wheel_axe_mm;
        data->cs.wheel.axe_mm_inv = 1.0/regs->c_wheel_axe_mm;
    }
    for (i=0;i<2;i++)
    {
        if (regs->c_wheel_mm_per_tick[i] != 0.0 && regs->c_wheel_mm_per_tick[i] != data->cs.wheel.mm_per_tick[i])
            data->cs.wheel.mm_per_tick[i] = regs->c_wheel_mm_per_tick[i];
    }
}




void odo_update_state(odo_state_t* state,volatile uint16_t* regs_qei, volatile float* regs_sum_dist, volatile float *regs_sum_angle,uint8_t inv)
{
    uint16_t qei[2];

    qei[0] = regs_qei[0];
    qei[1] = regs_qei[1];
    if (qei[0] != state->qei_latest[0] || qei[1] != state->qei_latest[1])
    {
        int32_t diff[2];
        

        diff[0] = (int32_t)qei[0]-(int32_t)state->qei_latest[0];
        if (diff[0] > (1<<15))
            diff[0] -= (1<<16);
        if (diff[0] < -(1<<15))
            diff[0] += (1<<16);
 
        diff[1] = (int32_t)qei[1]-(int32_t)state->qei_latest[1];
        if (diff[1] > (1<<15))
            diff[1] -= (1<<16);
        if (diff[1] < -(1<<15))
            diff[1] += (1<<16);

        if (inv==0)
        {
            // compute diff on each
            diff[0] = -diff[0];
            diff[1] = diff[1];
        } else {
            diff[0] = diff[0];
            diff[1] = -diff[1];
        }
        // save current value for next computation
        state->qei_latest[0] = qei[0];
        state->qei_latest[1] = qei[1];

        // update the total sum on each side
        state->qei_sum[0] += (int32_t)diff[0];
        state->qei_sum[1] += (int32_t)diff[1];

        // update SUM distance
        state->dist_sum += ((float)diff[0]*state->wheel.mm_per_tick[0] + (float)diff[1]*state->wheel.mm_per_tick[1])*0.5;

        // output the value
        *regs_sum_dist = state->dist_sum;

        // update SUM angle
        state->angle_sum = ((float)state->qei_sum[1]*state->wheel.mm_per_tick[1]-(float)state->qei_sum[0]*state->wheel.mm_per_tick[0])*state->wheel.axe_mm_inv;

        // output the value
        *regs_sum_angle = state->angle_sum;
    } 

}

float rad_mod_pi(float teta)
{
    float ret = teta;
    while (ret < -M_PI)
        ret += (M_PI*2);

    while (ret > M_PI)
        ret -= (M_PI*2);

    return ret;
}

typedef struct {
    ProtocolHeader hdr;
    uint32_t timestamp; // 1 = 20ns
    // 0 = Left, 1 = Right
    int32_t c_qei_sum[2]; // wheel encoder 
    int32_t m_qei_sum[2]; // motor encoder
    float x;
    float y;
    float teta_rad;
} __attribute__((packed)) odo_debug;



volatile int* pio_n = (volatile int*)(0x01000000);

int main()
{
    volatile odo_mapping_t* regs = (odo_mapping_t*)pio_n;
    odo_data_t odo;

    uart_tx_state tx_state;
    odo_debug tx_buffer;

    uint32_t ts[2];
    #define TS_UPDATE 0
    #define TS_DURATION 1

    uint32_t period_latest = 0;

    odometry_init(regs,&odo);

    ts_start(&ts[TS_UPDATE]);

    uart_rs232_configure(50000000/1000000);

    uart_rs232_buffer_init(&tx_state,(uint8_t*)&tx_buffer,sizeof(tx_buffer));

    
    tx_buffer.hdr.fanion = PROTOCOL_FANION;
    tx_buffer.hdr.size = sizeof(odo_debug);
    tx_buffer.hdr.crc = 0;
    tx_buffer.hdr.id = 0;

    //i2c_master_configure();
    //i2c_test();

    

    // odometry v2;
    for(;;) {

        //uart_rs232_tx('a');

        uart_rs232_buffer_tx_process(&tx_state);

        // update motor encoders 
        odo_update_state(&odo.ms,&regs->m_qei_value[0],&regs->odo_sum_m_distance,&regs->odo_sum_m_angle,0);
        // update coding wheel encoders
        odo_update_state(&odo.cs,&regs->c_qei_value[0],&regs->odo_sum_c_distance,&regs->odo_sum_c_angle,1);

        if (odo.freq_hz != regs->freq_hz_cfg)
        {
            jtaguart_puts("hoho\n");
            odo.freq_hz = regs->freq_hz_cfg;
            odo.period_cycles = ts_freq_to_cycles(odo.freq_hz);
        }


        // check if timer is elapsed to run the position update
        if (1)//(regs->period_cfg != 0)
        {
            if (ts_is_elapsed(ts[TS_UPDATE],(uint32_t)500000))//odo.period_cycles))
            {


/*
 - sl (diff): delta distance parcourue roue gauche
 - sr (diff): delta distance parcourue roue droite
 - alpha = teta(i-1) + (sr-sl)/(2L)
 - d = (sr+sl)/2
 - x(i) = x(i-1) + d.cos(alpha)
 - y(i) = y(i-1) + d.sin(alpha)
 - teta(i) = (sum(sr) - sum(sl))/L
*/
                if (odo.cs.qei_sum[0] != odo.cs.qei_sum_latest[0] || odo.cs.qei_sum[1] != odo.cs.qei_sum_latest[1])
                {
                    ts_start(&ts[TS_DURATION]);
                    ts[TS_UPDATE] = ts[TS_DURATION];
                                    
                    // processing

                    int32_t diff[2];
                    // compute diff on each
                    diff[0] = odo.cs.qei_sum[0]-odo.cs.qei_sum_latest[0];
                    diff[1] = odo.cs.qei_sum[1]-odo.cs.qei_sum_latest[1];

                    float dist;
                    float alpha;

                    dist = ((float)diff[0]*odo.cs.wheel.mm_per_tick[0] + (float)diff[1]*odo.cs.wheel.mm_per_tick[1])*0.5;
                    
                    alpha = odo.teta_rad;
                    alpha+= ((float)diff[1]*odo.cs.wheel.mm_per_tick[1] - (float)diff[0]*odo.cs.wheel.mm_per_tick[0])*0.5*odo.cs.wheel.axe_mm_inv;
                    alpha = rad_mod_pi(alpha);

                    odo.x += dist * __cosf(alpha);
                    odo.y += dist * __sinf(alpha);

                    odo.teta_rad = rad_mod_pi(odo.cs.angle_sum);
                    odo.teta_deg = (int16_t)(DEG(odo.teta_rad)*10.0);

                    regs->odo_pos_valid = 0;
                    regs->odo_pos_id = odo.id++;
                    regs->odo_pos_teta = odo.teta_deg;
                    regs->odo_pos_x = (int16_t)odo.x;
                    regs->odo_pos_y = (int16_t)odo.y;
                    regs->odo_pos_valid = 1;

                    regs->odo_pos_sum_distance = odo.cs.dist_sum;
                    regs->odo_pos_sum_angle = odo.cs.angle_sum;

                    
                    odo.cs.qei_sum_latest[0] = odo.cs.qei_sum[0];
                    odo.cs.qei_sum_latest[1] = odo.cs.qei_sum[1];

                    tx_buffer.timestamp = ts[TS_UPDATE];
                    tx_buffer.c_qei_sum[0] = odo.cs.qei_sum[0];
                    tx_buffer.c_qei_sum[1] = odo.cs.qei_sum[1];
                    tx_buffer.m_qei_sum[0] = odo.ms.qei_sum[0];
                    tx_buffer.m_qei_sum[1] = odo.ms.qei_sum[1];
                    tx_buffer.x = odo.x;
                    tx_buffer.y = odo.y;
                    tx_buffer.teta_rad = odo.cs.angle_sum;

                    tx_buffer.hdr.crc = protocolCrc((uint8_t*)&tx_buffer,sizeof(odo_debug));
                    
                    uart_rs232_buffer_tx(&tx_state,sizeof(odo_debug));

                    odometry_update_config(&odo,regs);

                    ts_stop(&ts[TS_DURATION]);
                    period_latest = ts[TS_DURATION];
                    regs->freq_hz_latest = ts_cycles_to_freq(period_latest);
                } else {
                    // we restart the timer
                    ts_start(&ts[TS_UPDATE]);
                }
                
                // check config changes (if any)
            }
            
        } else {
            ts_start(&ts[TS_UPDATE]);
        }


        // DEBUG
        char chr;

        chr = jtaguart_getchar();
        if (chr != 0)
        {
            switch (chr)
            {

                case 'p':
                    jtaguart_puts("M W\n");
                    jtaguart_puts("QEI:\n");
                    print_int((int32_t)odo.ms.qei_latest[0],1);
                    print_int((int32_t)odo.ms.qei_latest[1],1);
                    jtaguart_puts("QEI M SUM:\n");
                    print_int((int32_t)odo.ms.qei_sum[0],1);
                    print_int((int32_t)odo.ms.qei_sum[1],1);
                    jtaguart_puts("D:\n");
                    print_float(odo.ms.dist_sum,1);
                    jtaguart_puts("A:\n");
                    print_float(odo.ms.angle_sum,1);
                    jtaguart_puts("C W\n");
                    jtaguart_puts("QEI:\n");
                    print_int((int32_t)odo.cs.qei_latest[0],1);
                    print_int((int32_t)odo.cs.qei_latest[1],1);
                    jtaguart_puts("QEI M SUM:\n");
                    print_int((int32_t)odo.cs.qei_sum[0],1);
                    print_int((int32_t)odo.cs.qei_sum[1],1);
                    jtaguart_puts("D:\n");
                    print_float(odo.cs.dist_sum,1);
                    jtaguart_puts("A:\n");
                    print_float(odo.cs.angle_sum,1);
                    jtaguart_puts("--------- POS --------\n");
                    jtaguart_puts("X/Y/Angle(rad)/Angle(Deg):\n");
                    print_float(odo.x,1);
                    print_float(odo.y,1);
                    print_float(odo.teta_rad,1);
                    print_float((float)odo.teta_deg/10.0,1);
                    jtaguart_puts("Period:\n");
                    print_int((int32_t)period_latest,1);
                    jtaguart_puts("ID Pos:\n");
                    print_int((int32_t)odo.id,1);
                    break;

                case 'r':
                    // reset the current module
                    return 0;

            }
        }    
    

    }

    return 0;
}
