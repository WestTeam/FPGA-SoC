library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;


library work;
use     work.types_pkg.all;
use     work.robot_layer_3_pkg.all;

entity robot_layer_3 is
    generic (
        CLK_FREQUENCY_HZ : positive;
        RegCnt : positive
    );
    port (
        clk                     : in  std_logic;             
        reset                   : in  std_logic;             

        regs_data_in_value      : out  std_logic_vector(RegCnt*32-1 downto 0) := (others => '0'); 
        regs_data_in_read       : in std_logic_vector(RegCnt-1 downto 0);                       
        regs_data_out_value     : in std_logic_vector(RegCnt*32-1 downto 0);                    
        regs_data_out_write     : in std_logic_vector(RegCnt-1 downto 0);

        ---------------------------------
        -------- TO/FROM LAYER 2 --------
        ---------------------------------

        sum_m_dist    : in std_logic_vector(32-1 downto 0);
        sum_m_angle   : in std_logic_vector(32-1 downto 0);

        sum_c_dist    : in std_logic_vector(32-1 downto 0);
        sum_c_angle   : in std_logic_vector(32-1 downto 0);


        pos_valid     : in std_logic;
        pos_id        : in std_logic_vector(8-1 downto 0);
        pos_teta      : in std_logic_vector(16-1 downto 0);
        pos_x         : in std_logic_vector(16-1 downto 0);
        pos_y         : in std_logic_vector(16-1 downto 0);
        pos_sum_dist  : in std_logic_vector(32-1 downto 0);
        pos_sum_angle : in std_logic_vector(32-1 downto 0);

        dist_en       : out std_logic;
        dist_acc      : out std_logic_vector(32-1 downto 0);
        dist_speed    : out std_logic_vector(32-1 downto 0);
        dist_target   : out std_logic_vector(32-1 downto 0);

        angle_en      : out std_logic;
        angle_acc     : out std_logic_vector(32-1 downto 0);
        angle_speed   : out std_logic_vector(32-1 downto 0);
        angle_target  : out std_logic_vector(32-1 downto 0)


    );
end entity;

architecture rtl of robot_layer_3 is

    component system is
        port (
            clk_clk            : in  std_logic                      := 'X';             -- clk
            pio_data_in_value  : in  std_logic_vector(511 downto 0) := (others => 'X'); -- data_in_value
            pio_data_in_read   : out std_logic_vector(15 downto 0);                     -- data_in_read
            pio_data_out_value : out std_logic_vector(511 downto 0);                    -- data_out_value
            pio_data_out_write : out std_logic_vector(15 downto 0);                     -- data_out_write
            reset_reset_n      : in  std_logic                      := 'X'              -- reset_n
        );
    end component system;


    signal w_reset_n : std_logic;


    signal w_regs_data_in_value      : std_logic_vector(RegCnt*32-1 downto 0);
    signal w_regs_data_in_value_mask : std_logic_vector(RegCnt*4-1 downto 0) := (others=>'0');


begin
	
    w_reset_n <= not reset;

    --! we return for read the same written data, expect for some bytes (noted masked) where we compute the value internally
    g_reg: for i in 0 to w_regs_data_in_value_mask'length-1 generate
        regs_data_in_value((i+1)*8-1 downto i*8) <= regs_data_out_value((i+1)*8-1 downto i*8) when w_regs_data_in_value_mask(i) = '0' else w_regs_data_in_value((i+1)*8-1 downto i*8);
    end generate;



    b_trajectory: block
        signal w_pio_data_in_value   :  std_logic_vector(511 downto 0) := (others=>'0');
        signal w_pio_data_in_read    :  std_logic_vector(15 downto 0);
        signal w_pio_data_out_value  :  std_logic_vector(511 downto 0);
        signal w_pio_data_out_write  :  std_logic_vector(15 downto 0);

        --! difference between external MM and CPU regs
        constant REGS_ORIGIN : natural := 2;

        constant REGS_PID_DISTANCE_OFFSET : natural := 2;
        constant REGS_PID_ANGLE_OFFSET : natural := REGS_PID_DISTANCE_OFFSET+4;
 
        constant REGS_ODO_OFFSET : natural := 2;
        constant REGS_TRAJ_OUT_OFFSET : natural := 14;
    begin


        w_regs_data_in_value_mask((2+REGS_ORIGIN)*4-1 downto (1+REGS_ORIGIN)*4) <= "0011";
        w_regs_data_in_value_mask((REGS_ORIGIN+REGS_TRAJ_OUT_OFFSET+2-1)*4-1 downto (REGS_ORIGIN+REGS_TRAJ_OUT_OFFSET-1)*4) <= (others=>'1');


        p_async: process(regs_data_out_value,w_pio_data_out_value,
                         sum_m_dist,sum_m_angle,sum_c_dist,sum_c_angle,pos_valid,pos_id,pos_teta,pos_x,pos_y,pos_sum_dist,pos_sum_angle
                        ) is
        begin
            w_pio_data_in_value(1*32-1 downto 0*32)      <= X"00000200";
            w_pio_data_in_value((16)*32-1 downto 1*32) <= regs_data_out_value((REGS_ORIGIN+15)*32-1 downto REGS_ORIGIN*32);

            --! we override the values for odometry data
            w_pio_data_in_value((REGS_ODO_OFFSET+8)*32-1 downto REGS_ODO_OFFSET*32) <= pos_sum_angle &
                                                                                       pos_sum_dist & 
                                                                                       pos_y & 
                                                                                       pos_x & 
                                                                                       pos_teta & 
                                                                                       pos_id & 
                                                                                       "0000000" & pos_valid & 
                                                                                       sum_c_angle & 
                                                                                       sum_c_dist & 
                                                                                       sum_m_angle & 
                                                                                       sum_m_dist;         
            w_regs_data_in_value((2+REGS_ORIGIN)*32-1 downto (1+REGS_ORIGIN)*32) <= w_pio_data_out_value((2)*32-1 downto (1)*32); 

            w_regs_data_in_value((REGS_TRAJ_OUT_OFFSET+REGS_ORIGIN+2-1)*32-1 downto (REGS_TRAJ_OUT_OFFSET+REGS_ORIGIN-1)*32) <= w_pio_data_out_value((REGS_TRAJ_OUT_OFFSET+2)*32-1 downto (REGS_TRAJ_OUT_OFFSET)*32); 
            
        end process;

        --w_odo_output(0) <= w_pio_data_out_value((1+8)*32-1 downto (0+8)*32); --! distance
        --w_odo_output(1) <= w_pio_data_out_value((2+8)*32-1 downto (1+8)*32); --! angle

        dist_en       <= w_pio_data_out_value((1+REGS_PID_DISTANCE_OFFSET)*32-1 downto (0+REGS_PID_DISTANCE_OFFSET)*32)((0+REGS_PID_DISTANCE_OFFSET)*32);
        dist_speed    <= w_pio_data_out_value((2+REGS_PID_DISTANCE_OFFSET)*32-1 downto (1+REGS_PID_DISTANCE_OFFSET)*32);
        dist_acc      <= w_pio_data_out_value((3+REGS_PID_DISTANCE_OFFSET)*32-1 downto (2+REGS_PID_DISTANCE_OFFSET)*32);
        dist_target   <= w_pio_data_out_value((4+REGS_PID_DISTANCE_OFFSET)*32-1 downto (3+REGS_PID_DISTANCE_OFFSET)*32);


        angle_en      <= w_pio_data_out_value((1+REGS_PID_ANGLE_OFFSET)*32-1 downto (0+REGS_PID_ANGLE_OFFSET)*32)((0+REGS_PID_ANGLE_OFFSET)*32);
        angle_speed   <= w_pio_data_out_value((2+REGS_PID_ANGLE_OFFSET)*32-1 downto (1+REGS_PID_ANGLE_OFFSET)*32);
        angle_acc     <= w_pio_data_out_value((3+REGS_PID_ANGLE_OFFSET)*32-1 downto (2+REGS_PID_ANGLE_OFFSET)*32);
        angle_target  <= w_pio_data_out_value((4+REGS_PID_ANGLE_OFFSET)*32-1 downto (3+REGS_PID_ANGLE_OFFSET)*32);



        --! disable warnings
        assert w_pio_data_in_read = w_pio_data_in_read;
        assert w_pio_data_out_write = w_pio_data_out_write;

        inst_trajectory_rv : component system
        port map (
            clk_clk                 => clk,
            reset_reset_n           => w_reset_n,
            pio_data_in_value       => w_pio_data_in_value,
            pio_data_in_read        => w_pio_data_in_read,
            pio_data_out_value      => w_pio_data_out_value,
            pio_data_out_write      => w_pio_data_out_write
        );

    end block;





end architecture;

