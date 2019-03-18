
#include <string.h>
#include <stdint.h>
#include <limits.h>

#include <2wheels/trajectory_manager.h>
#include <2wheels/trajectory_manager_core.h>
#include <control_system_manager.h>
#include <2wheels/robot_system.h>
#include <2wheels/position_manager.h>
#include <quadramp.h>

#include "tools.h"

#define M_PI           3.14159265358979323846

#define DEG(x) ((x) * (180.0 / M_PI))

struct pid_regs {
    uint8_t enable; // OUT DATA
    uint8_t unused[3];
    float speed; // OUT  DATA
    float acc; // OUT DATA
    float target; // OUT DATA
};

typedef struct trajectory_mapping
{
    uint8_t reset;
    uint8_t pgm_id;
    uint8_t arg[2];
    uint16_t freq_hz_cfg; // IN
    uint16_t freq_hz_latest; // OUT

    union {
        struct odo {
            float sum_m_distance; // IN DATA
            float sum_m_angle; // IN  DATA
            float sum_c_distance; // IN  DATA
            float sum_c_angle; // IN  DATA
            uint8_t pos_valid; // IN  DATA
            uint8_t pos_id; // IN  DATA
            int16_t pos_teta; // IN  DATA
            int16_t pos_x; // IN  DATA
            int16_t pos_y; // IN  DATA
            float   pos_sum_distance; // IN  DATA
            float   pos_sum_angle; // IN  DATA

        } odo;
        struct pid {
            struct pid_regs distance;
            struct pid_regs angle;
        } pid;
    };

    //CMD_TYPE = TRAJ / CFG_SPEED / CFG_ANGLE;
    // TRAJ_TYPE = ....
  
    #define CMD_TYPE_TRAJ 0x0
    #define CMD_TYPE_CFG_DISTANCE 0x1
    #define CMD_TYPE_CFG_ANGLE 0x2
    #define CMD_TYPE_CFG_WND 0x3


    uint8_t cmd_valid; // IN
    uint8_t cmd_id; // IN
    uint8_t cmd_type; // IN
    uint8_t cmd_order_type;
    union {
        struct cfg
        {
            float speed; // IN
            float acc; // IN
        } cfg; 
        struct wnd
        {
            float distance; // IN
            float angle_deg; // IN
            float angle_start_deg; // IN
        } wnd; 
        struct order_pos
        {
            float teta; // IN 
            float x; // IN 
            float y; // IN
        } order_pos;        

        struct order_ad
        {
            float angle_deg; // IN 
            float distance; // IN
            uint8_t correction; // if 0, coding wheel ignored 
        } order_ad;

    } cmd;

    uint8_t traj_cmd_ack; // OUT
    uint8_t traj_state; // OUT
    uint8_t traj_blocked; // OUT
    uint8_t traj_in_window; // OUT

    //IN = 13
    //OUT = 8


} __attribute__((packed)) trajectory_mapping_t;


#define LEFT 0
#define RIGHT 1



typedef struct trajectory_data
{
    struct robot_system rs;
    struct robot_position pos;
    struct cs cs_a;
    struct cs cs_d;
    struct quadramp_filter qr_a;
    struct quadramp_filter qr_d;
    struct trajectory trj;

    // latest position received
    uint8_t pos_valid; 
    uint8_t pos_id; 

    // latest cmd received
    uint8_t cmd_valid;
    uint8_t cmd_id;


    uint16_t freq_hz;
    uint32_t period_cycles;
    
    // OUTPUT PROCESSED
    float x;
    float y;
    float teta_rad; // modulo PI
    int16_t teta_deg; // (1 = 0.1deg)
    uint8_t id;


} trajectory_data_t;


void _trajectory_init(volatile trajectory_mapping_t* regs, trajectory_data_t* data)
{
    // init structures
    memset((void*)data,0,sizeof(*data));
    memset((void*)regs,0,sizeof(*regs));

    trajectory_init(&data->trj, 100.0);
    trajectory_set_cs(&data->trj,&data->cs_d,&data->cs_a);
    data->cs_d.consign_filter_params = &data->qr_d;
    data->cs_a.consign_filter_params = &data->qr_a;
    trajectory_set_robot_params(&data->trj,&data->rs,&data->pos);
    trajectory_set_windows(&data->trj,5.0,2.0,3.0);

}


void trajectory_update_position(volatile trajectory_mapping_t* regs, trajectory_data_t* data)
{    
    data->rs.virtual_encoders.distance = regs->odo.sum_m_distance;
    data->rs.virtual_encoders.angle    = regs->odo.sum_m_angle;

    data->rs.pext_prev.distance = regs->odo.sum_c_distance;//pos_sum_distance;
    data->rs.pext_prev.angle    = regs->odo.sum_c_angle;//pos_sum_angle;

    uint8_t pos_id = regs->odo.pos_id;
    if (regs->odo.pos_valid == 1 && (pos_id != data->pos_id || data->pos_valid == 0))
    {
        //print_int(pos_id,1);
        //print_int(data->pos_id,1);
        //print_int(regs->odo.pos_id,1);
        
        int16_t pos_teta = regs->odo.pos_teta; 
        int16_t pos_x    = regs->odo.pos_x; 
        int16_t pos_y    = regs->odo.pos_y; 
        float   pos_sum_distance = regs->odo.pos_sum_distance;
        float   pos_sum_angle    = regs->odo.pos_sum_angle;
        // we check again the pos_id is the same as well as valid, to make sure the data we read is ok.
        if (regs->odo.pos_id == pos_id && regs->odo.pos_valid == 1)
        {
            data->pos.pos_s16.x = pos_x;
            data->pos.pos_s16.y = pos_y;
            data->pos.pos_s16.a = pos_teta/10;

            data->pos.pos_d.x = (float)pos_x;
            data->pos.pos_d.y = (float)pos_y;
            float fpos_teta = (float)pos_teta;
            //print_float(fpos_teta,1);
            data->pos.pos_d.a = fpos_teta * (M_PI / 1800.0);



            data->pos_valid = 1;
            data->pos_id = pos_id;

        }
    }

}



void trajectory_update_cmd(volatile trajectory_mapping_t* regs, trajectory_data_t* data)
{
    uint8_t cmd_id = regs->cmd_id;

    if (regs->cmd_valid == 1 && (cmd_id != data->cmd_id || data->cmd_valid == 0))
    {
        //print_int(cmd_id,1);
        uint8_t cmd_type = regs->cmd_type;
        uint8_t cmd_order_type = regs->cmd_order_type;
        float wnd_distance = regs->cmd.wnd.distance;
        float wnd_angle_deg= regs->cmd.wnd.angle_deg;
        float wnd_angle_start_deg= regs->cmd.wnd.angle_start_deg;


        float speed     = regs->cmd.cfg.speed;
        float acc       = regs->cmd.cfg.acc;

        float teta      = regs->cmd.order_pos.teta;
        float x         = regs->cmd.order_pos.x;
        float y         = regs->cmd.order_pos.y;

        float distance  = regs->cmd.order_ad.distance;
        float angle_deg = regs->cmd.order_ad.angle_deg;
        uint8_t correction = regs->cmd.order_ad.correction;

        // we check again the pos_id is the same as well as valid, to make sure the data we read is ok.
        if (regs->cmd_id == cmd_id && regs->cmd_valid == 1)
        {
            // depending on the command type, we save the data:
            switch (cmd_type) 
            {
#define TRAJ_DISABLE 0
#define TRAJ_ENABLE 1
#define TRAJ_STOP 2
#define TRAJ_HARDSTOP 3
#define TRAJ_D_REL 4
#define TRAJ_ONLY_D_REL 5
#define TRAJ_A_REL 6
#define TRAJ_A_ABS 7
#define TRAJ_ONLY_A_REL 8
#define TRAJ_ONLY_A_ABS 9
#define TRAJ_D_A_REL 10
#define TRAJ_TURNTO_XY 11
#define TRAJ_TURNTO_XY_BEHIND 12
#define TRAJ_GOTO_XY_ABS 13
#define TRAJ_GOTO_FORWARD_XY_ABS 14
#define TRAJ_GOTO_BACKWARD_XY_ABS 15
#define TRAJ_GOTO_D_A_REL 16
#define TRAJ_GOTO_XY_REL 17



                case CMD_TYPE_TRAJ:
                    switch (cmd_order_type)
                    {
                        case TRAJ_DISABLE:
                            regs->pid.distance.enable = 0;
                            regs->pid.angle.enable = 0;
                            break;
                        case TRAJ_ENABLE:
                            trajectory_stop(&data->trj);
                            regs->pid.distance.enable = 1;
                            regs->pid.angle.enable = 1;
                            break;
                        case TRAJ_STOP:
                            trajectory_stop(&data->trj);
                            break;
                        case TRAJ_HARDSTOP:
                            trajectory_hardstop(&data->trj);
                            break;
                        case TRAJ_D_REL:
                            trajectory_d_rel(&data->trj,distance,correction);
                            break;
                        case TRAJ_ONLY_D_REL:
                            trajectory_only_d_rel(&data->trj,distance,correction);
                            break;
                        case TRAJ_A_REL:
                            trajectory_a_rel(&data->trj,angle_deg,correction); // DEG
                            break;
                        case TRAJ_A_ABS:
                            trajectory_a_abs(&data->trj,angle_deg,correction); // DEG
                            break;
                        case TRAJ_ONLY_A_REL:
                            trajectory_only_a_rel(&data->trj,angle_deg,correction); // DEG
                            break;
                        case TRAJ_ONLY_A_ABS:
                            trajectory_only_a_abs(&data->trj,angle_deg,correction); // DEG
                            break;
                        case TRAJ_D_A_REL:
                            trajectory_d_a_rel(&data->trj,distance,angle_deg,correction); // DEG
                            break;
                        case TRAJ_TURNTO_XY:
                            trajectory_turnto_xy(&data->trj,x,y);
                            break;
                        case TRAJ_TURNTO_XY_BEHIND:
                            trajectory_turnto_xy_behind(&data->trj,x,y);
                            break;
                        case TRAJ_GOTO_XY_ABS:
                            trajectory_goto_xy_abs(&data->trj,x,y);
                            break;
                        case TRAJ_GOTO_FORWARD_XY_ABS:
                            trajectory_goto_forward_xy_abs(&data->trj,x,y);
                            break;
                        case TRAJ_GOTO_BACKWARD_XY_ABS:
                            trajectory_goto_backward_xy_abs(&data->trj,x,y);
                            break;
                        case TRAJ_GOTO_D_A_REL:
                            trajectory_goto_d_a_rel(&data->trj,distance,angle_deg,correction); //DEG
                            break;
                        case TRAJ_GOTO_XY_REL:
                            trajectory_goto_xy_rel(&data->trj,x,y);
                            break;


                    }

                    break;
                case CMD_TYPE_CFG_DISTANCE:
                    data->trj.d_speed = speed;
                    data->trj.d_acc = acc;
                   break;

                case CMD_TYPE_CFG_ANGLE:
                    data->trj.a_speed = speed;
                    data->trj.a_acc = acc;
                    break;

                case CMD_TYPE_CFG_WND:
                    trajectory_set_windows(&data->trj,wnd_distance,wnd_angle_deg,wnd_angle_start_deg);
                    break;
            }

            data->cmd_valid = 1;
            data->cmd_id = cmd_id;
            regs->traj_cmd_ack = cmd_id+1;
        }
    }

}


void trajectory_update_target(volatile trajectory_mapping_t* regs, trajectory_data_t* data)
{
    
    regs->pid.distance.speed = data->qr_d.var_1st_ord_pos;//data->trj.d_speed;
    regs->pid.distance.acc = data->qr_d.var_2nd_ord_pos;//data->trj.d_acc;
    regs->pid.distance.target = data->cs_d.consign_value;

    regs->pid.angle.speed = data->qr_a.var_1st_ord_pos;//data->trj.a_speed;
    regs->pid.angle.acc = data->qr_a.var_2nd_ord_pos;//data->trj.a_acc;
    regs->pid.angle.target = data->cs_a.consign_value;

    regs->traj_state = data->trj.state;

    regs->traj_in_window = trajectory_in_window(&data->trj,data->trj.d_win,data->trj.a_win_rad);
}

void trajectory_check_blocked(volatile trajectory_mapping_t* regs, trajectory_data_t* data)
{
    static float prev_sum_m_distance = 0.0;
//    static float prev_sum_m_angle = 0.0;
    static float prev_sum_c_distance = 0.0;
//    static float prev_sum_c_angle = 0.0;
    static uint16_t cnt = 0;
    uint8_t blocked = 0;

    float m_diff;
    float c_diff;

    cnt++;

    if (cnt == 300)
    {
        cnt = 0;
        blocked = 0;
        if (regs->odo.sum_m_distance != prev_sum_m_distance)
        {
            m_diff = regs->odo.sum_m_distance - prev_sum_m_distance;
            c_diff = (regs->odo.sum_c_distance - prev_sum_c_distance)*10.0; 
            //print_float(m_diff,0);
            //jtaguart_puts("\t");
            //print_float(c_diff,1);

            if (fabs(m_diff) > fabs(c_diff) && fabs(m_diff) > 5.0)
            {
                blocked = 1;
            }
        }

        // we store the values for future use
        prev_sum_m_distance = regs->odo.sum_m_distance;
//        prev_sum_m_angle    = regs->odo.sum_m_angle;

        prev_sum_c_distance = regs->odo.sum_c_distance;
//        prev_sum_c_angle    = regs->odo.sum_c_angle;

        regs->traj_blocked = blocked;
    }
}


volatile int* pio_n = (volatile int*)(0x01000000);

int main()
{
    volatile trajectory_mapping_t* regs = (trajectory_mapping_t*)pio_n;
    trajectory_data_t traj;

    // DEBUG
    char chr;

    uint32_t ts[2];
    #define TS_UPDATE 0
    #define TS_DURATION 1

    uint32_t freq_latest = 0;
    uint32_t period_latest = 0;

    
    //print_float((float)regs->odo.pos_teta,1);
    //traj.pos.pos_d.a = ((float)regs->odo.pos_teta) * (M_PI / 1800.0);
    //print_float(traj.pos.pos_d.a,1);

    jtaguart_puts("Trajectory: Init\n");
/*    print_int(sizeof(*regs),1);
    print_int(sizeof(regs->odo),1);
    print_int(sizeof(regs->pid),1);
*/

    _trajectory_init(regs,&traj);


    ts_start(&ts[TS_UPDATE]);




    // trajectory manager;
    for(;;) {

        chr = jtaguart_getchar();

        // update position
        trajectory_update_position(regs,&traj);
                        
        // update order
        trajectory_update_cmd(regs,&traj); 

        // check if timer is elapsed to run the position update
        if (1)//(regs->period_cfg != 0)
        {

            if (traj.freq_hz != regs->freq_hz_cfg)
            {
                traj.freq_hz = regs->freq_hz_cfg;
                traj.period_cycles = ts_freq_to_cycles(traj.freq_hz);
            }

            if (ts_is_elapsed(ts[TS_UPDATE],(uint32_t)500000))//traj.period_cycles))
            {
                if (1)
                {
                    ts_start(&ts[TS_DURATION]);
                    ts[TS_UPDATE] = ts[TS_DURATION];                   

                    // processing
                    trajectory_manager_event((void*)&traj.trj);

                    trajectory_check_blocked(regs,&traj);

                    ts_stop(&ts[TS_DURATION]);
                    period_latest = ts[TS_DURATION];
                    freq_latest = ts_cycles_to_freq(period_latest);
                    regs->freq_hz_latest = freq_latest;

                    //chr = 'p';

                }

                // check config changes (if any)
            } 
            
        } else {
            ts_start(&ts[TS_UPDATE]);
        }

        // update output to send target / config to PIDs
        trajectory_update_target(regs,&traj);

#if 0
        if (chr != 0)
        {
            switch (chr)
            {

                case 'p':
                    jtaguart_puts("----- Trajectory Infos ----\n");
                    jtaguart_puts("(X/Y/A deg/A Rad)\n");
                    print_int((int32_t)traj.pos.pos_s16.x,1);
                    print_int((int32_t)traj.pos.pos_s16.y,1);
                    print_int((int32_t)traj.pos.pos_s16.a,1);
                    print_float(traj.trj.position->pos_d.a,1);
/*                    print_int(regs->odo.pos_teta,1);
                    print_int(regs->odo.pos_x,1);
                    print_int(regs->odo.pos_y,1);
                    print_float(regs->odo.sum_m_distance,1);
                    print_float(regs->odo.sum_m_angle,1);
                    print_float(regs->odo.sum_c_distance,1);
                    print_float(regs->odo.sum_c_angle,1);
                    print_float(regs->odo.pos_sum_distance,1);
                    print_float(regs->odo.pos_sum_angle,1);
                    jtaguart_puts("Freq Period\n");
                    print_int(freq_latest,1);
                    print_int((int32_t)period_latest,1);
                    print_int((int32_t)regs->freq_hz_cfg,1);
                    print_float(traj.trj.target.pol.angle,1);
                    
*/
                    print_float(traj.trj.d_win,1);
                    print_float(traj.trj.a_win_rad,1);
                    print_float(traj.trj.a_start_rad,1);

/*
                    jtaguart_puts("Dist - Speed/Acc/Target\n");  
                    print_float(traj.qr_d.var_1st_ord_pos,1);
                    print_float(traj.qr_d.var_2nd_ord_pos,1);
                    print_float(traj.cs_d.consign_value,1);

                    jtaguart_puts("Angle - Speed/Acc/Target\n");  
                    print_float(traj.qr_a.var_1st_ord_pos,1);
                    print_float(traj.qr_a.var_2nd_ord_pos,1);
                    print_float(traj.cs_a.consign_value,1);
*/

                    break;

                case 'r':
                    // reset the current module
                    return 0;
                    break;

            }
            chr = 0;
        }  
#endif      


    }

    return 0;
}
