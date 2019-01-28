library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;


library work;
use     work.types_pkg.all;
use     work.robot_layer_2_pkg.all;

entity robot_layer_2 is
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
        -------- TO/FROM LAYER 1 --------
        ---------------------------------

        --------- UART ----------
        uart_tx       : out std_logic_vector(4-1 downto 0);
        uart_rx       : in  std_logic_vector(4-1 downto 0);



        motor_value   : out  int16_t(MOTOR_COUNT-1 downto 0);
        motor_current : in   int24_t(MOTOR_COUNT-1 downto 0);
        motor_fault   : in   std_logic_vector(MOTOR_COUNT-1 downto 0);

        qei_value     : in   int16_t(QEI_COUNT-1 downto 0);
        qei_ref       : in   std_logic_vector(QEI_COUNT-1 downto 0);

        ---------------------------------
        -------- TO/FROM LAYER 3 --------
        ---------------------------------

        sum_m_dist    : out std_logic_vector(32-1 downto 0);
        sum_m_angle   : out std_logic_vector(32-1 downto 0);

        sum_c_dist    : out std_logic_vector(32-1 downto 0);
        sum_c_angle   : out std_logic_vector(32-1 downto 0);


        pos_valid     : out std_logic;
        pos_id        : out std_logic_vector(8-1 downto 0);
        pos_teta      : out std_logic_vector(16-1 downto 0);
        pos_x         : out std_logic_vector(16-1 downto 0);
        pos_y         : out std_logic_vector(16-1 downto 0);
        pos_sum_dist  : out std_logic_vector(32-1 downto 0);
        pos_sum_angle : out std_logic_vector(32-1 downto 0);

        dist_en       : in std_logic;
        dist_acc      : in std_logic_vector(32-1 downto 0);
        dist_speed    : in std_logic_vector(32-1 downto 0);
        dist_target   : in std_logic_vector(32-1 downto 0);

        angle_en      : in std_logic;
        angle_acc     : in std_logic_vector(32-1 downto 0);
        angle_speed   : in std_logic_vector(32-1 downto 0);
        angle_target  : in std_logic_vector(32-1 downto 0)

    );
end entity;

architecture rtl of robot_layer_2 is

    component system is
        port (
            clk_clk            : in  std_logic                      := 'X';             -- clk
            pio_data_in_value  : in  std_logic_vector(511 downto 0) := (others => 'X'); -- data_in_value
            pio_data_in_read   : out std_logic_vector(15 downto 0);                     -- data_in_read
            pio_data_out_value : out std_logic_vector(511 downto 0);                    -- data_out_value
            pio_data_out_write : out std_logic_vector(15 downto 0);                     -- data_out_write
            reset_reset_n      : in  std_logic                      := 'X';              -- reset_n
			uart_0_external_rxd      : in  std_logic                      := 'X';             -- rxd
			uart_0_external_txd      : out std_logic                                          -- txd
        );
    end component system;


    signal w_reset_n : std_logic;


    signal w_regs_data_in_value      : std_logic_vector(RegCnt*32-1 downto 0);
    signal w_regs_data_in_value_mask : std_logic_vector(RegCnt*4-1 downto 0) := (others=>'0');

--    constant MSG_SIZE : natural := 1+2+4*2+2+2;

--    signal r_uart_tx_data  : std_logic_vector(MSG_SIZE*8-1 downto 0);
--    signal w_uart_tx_valid : std_logic;
--    signal w_uart_tx_busy  : std_logic;

    signal w_odo_output : int32_t(2-1 downto 0);

    constant PID_COUNT : natural := 3;

    signal w_pid_en         : std_logic_vector(PID_COUNT-1 downto 0);
    signal w_pid_acc        : int32_t(PID_COUNT-1 downto 0);
    signal w_pid_speed      : int32_t(PID_COUNT-1 downto 0);
    signal w_pid_target     : int32_t(PID_COUNT-1 downto 0);
    signal w_pid_measure    : int32_t(PID_COUNT-1 downto 0);
    signal w_pid_output     : int32_t(PID_COUNT-1 downto 0);
begin
	
    w_reset_n <= not reset;

    --! we return for read the same written data, expect for some bytes (noted masked) where we compute the value internally
    g_reg: for i in 0 to w_regs_data_in_value_mask'length-1 generate
        regs_data_in_value((i+1)*8-1 downto i*8) <= regs_data_out_value((i+1)*8-1 downto i*8) when w_regs_data_in_value_mask(i) = '0' else w_regs_data_in_value((i+1)*8-1 downto i*8);
    end generate;



    b_odometry: block
        signal w_pio_data_in_value   :  std_logic_vector(511 downto 0) := (others=>'0');
        signal w_pio_data_in_read    :  std_logic_vector(15 downto 0);
        signal w_pio_data_out_value  :  std_logic_vector(511 downto 0);
        signal w_pio_data_out_write  :  std_logic_vector(15 downto 0);

        constant REGS_ODO_OUT_OFFSET : natural := 8;
    begin


        w_regs_data_in_value_mask((1+2)*4-1 downto (0+2)*4) <= "0011";
        w_regs_data_in_value_mask((6+9)*4-1 downto (0+9)*4) <= (others=>'1');

        p_async: process(regs_data_out_value,w_pio_data_out_value,qei_value) is
        begin
            w_pio_data_in_value(1*32-1 downto 0*32)      <= X"00000000";
            w_pio_data_in_value((1+13)*32-1 downto 1*32) <= regs_data_out_value((2+13)*32-1 downto 2*32);

            --! we override the values for register 9 & 10 to give QEI inputs
            w_pio_data_in_value((8+2)*32-1 downto 8*32) <= qei_value(3) & qei_value(2) & qei_value(1) & qei_value(0);


            w_regs_data_in_value((8+8)*32-1 downto (0+8)*32) <= w_pio_data_out_value((8+7)*32-1 downto (0+7)*32); 
            
        end process;

        w_odo_output(0) <= w_pio_data_out_value((1+REGS_ODO_OUT_OFFSET)*32-1 downto (0+REGS_ODO_OUT_OFFSET)*32); --! distance
        w_odo_output(1) <= w_pio_data_out_value((2+REGS_ODO_OUT_OFFSET)*32-1 downto (1+REGS_ODO_OUT_OFFSET)*32); --! angle

        sum_m_dist  <= w_pio_data_out_value((1+REGS_ODO_OUT_OFFSET)*32-1 downto (0+REGS_ODO_OUT_OFFSET)*32);
        sum_m_angle <= w_pio_data_out_value((2+REGS_ODO_OUT_OFFSET)*32-1 downto (1+REGS_ODO_OUT_OFFSET)*32);
        sum_c_dist  <= w_pio_data_out_value((3+REGS_ODO_OUT_OFFSET)*32-1 downto (2+REGS_ODO_OUT_OFFSET)*32);
        sum_c_angle <= w_pio_data_out_value((4+REGS_ODO_OUT_OFFSET)*32-1 downto (3+REGS_ODO_OUT_OFFSET)*32);

        pos_valid     <= std_norm_range(w_pio_data_out_value((5+REGS_ODO_OUT_OFFSET)*32-1 downto (4+REGS_ODO_OUT_OFFSET)*32))(0);
        pos_id        <= std_norm_range(w_pio_data_out_value((5+REGS_ODO_OUT_OFFSET)*32-1 downto (4+REGS_ODO_OUT_OFFSET)*32))(16-1 downto 8);
        pos_teta      <= std_norm_range(w_pio_data_out_value((5+REGS_ODO_OUT_OFFSET)*32-1 downto (4+REGS_ODO_OUT_OFFSET)*32))(32-1 downto 16);
        pos_x         <= std_norm_range(w_pio_data_out_value((6+REGS_ODO_OUT_OFFSET)*32-1 downto (5+REGS_ODO_OUT_OFFSET)*32))(16-1 downto 0);
        pos_y         <= std_norm_range(w_pio_data_out_value((6+REGS_ODO_OUT_OFFSET)*32-1 downto (5+REGS_ODO_OUT_OFFSET)*32))(32-1 downto 16);
        pos_sum_dist  <= w_pio_data_out_value((7+REGS_ODO_OUT_OFFSET)*32-1 downto (6+REGS_ODO_OUT_OFFSET)*32);
        pos_sum_angle <= w_pio_data_out_value((8+REGS_ODO_OUT_OFFSET)*32-1 downto (7+REGS_ODO_OUT_OFFSET)*32);



        --! disable warnings
        assert w_pio_data_in_read = w_pio_data_in_read;
        assert w_pio_data_out_write = w_pio_data_out_write;


        inst_odometry_rv : component system
        port map (
            clk_clk                 => clk,
            reset_reset_n           => w_reset_n,
            pio_data_in_value       => w_pio_data_in_value,
            pio_data_in_read        => w_pio_data_in_read,
            pio_data_out_value      => w_pio_data_out_value,
            pio_data_out_write      => w_pio_data_out_write,
			uart_0_external_rxd      => uart_rx(0),
			uart_0_external_txd      => uart_tx(0)
        );

    end block;





    b_carroussel: block
        signal r_position : std_logic_vector(32-1 downto 0);
        signal r_ref      : std_logic_vector(32-1 downto 0);
        signal r_qei_last : std_logic_vector(16-1 downto 0);
        constant REGS_CARROUSSEL_REF_OFFSET : natural := 48;
    begin

        w_regs_data_in_value_mask((REGS_CARROUSSEL_REF_OFFSET+1)*4-1 downto (REGS_CARROUSSEL_REF_OFFSET)*4) <= (others=>'1');
        w_regs_data_in_value((REGS_CARROUSSEL_REF_OFFSET+1)*32-1 downto (REGS_CARROUSSEL_REF_OFFSET)*32) <= r_ref;

        p_sync: process(clk,reset) is
            variable v_diff : integer;
        begin
            if reset = '1' then
                r_position <= (others=>'0');
                r_ref      <= (others=>'0');
                r_qei_last <= (others=>'0');
            elsif rising_edge(clk) then
                r_qei_last <= qei_value(4);
                if qei_value(4) /= r_qei_last then
                    v_diff := to_integer(unsigned(qei_value(4)))-to_integer(unsigned(r_qei_last));
                    if v_diff >= 2**15 then
                        v_diff := v_diff - 2**16;                        
                    end if;
                    if v_diff <= -2**15 then
                        v_diff := v_diff + 2**16;                        
                    end if;
                    r_position <= std_logic_vector(signed(r_position)+to_signed(v_diff,32));
                end if;
                if qei_ref(4) = '1' then
                    r_ref <= r_position;
                end if;
            end if;
        end process;

        w_pid_measure(2) <= r_position;

    end block;

    w_pid_measure(0) <= w_odo_output(0);
    w_pid_measure(1) <= w_odo_output(1);



    w_pid_en(0)         <= dist_en;
    w_pid_acc(0)        <= dist_acc;
    w_pid_speed(0)      <= dist_speed;
    w_pid_target(0)     <= dist_target;


    w_pid_en(1)         <= angle_en;
    w_pid_acc(1)        <= angle_acc;
    w_pid_speed(1)      <= angle_speed;
    w_pid_target(1)     <= angle_target;

    --! not used from Layer 3, override from sw needed to control it
    w_pid_en(2)         <= '0';
    w_pid_acc(2)        <= (others=>'0');
    w_pid_speed(2)      <= (others=>'0');
    w_pid_target(2)     <= (others=>'0');  
    

    b_motor_pid: block
    begin
        g_motor: for i in 0 to PID_COUNT-1 generate
            constant REG_COUNT : natural := 11;
            constant REG_INDEX : natural := 15+REG_COUNT*i;
            constant REG_MEASURE_INDEX : natural := 9;
            constant REG_OUTPUT_INDEX : natural := 11;
            signal w_pio_data_in_value   :  std_logic_vector(511 downto 0) := (others=>'0');
            signal w_pio_data_in_read    :  std_logic_vector(15 downto 0);
            signal w_pio_data_out_value  :  std_logic_vector(511 downto 0);
            signal w_pio_data_out_write  :  std_logic_vector(15 downto 0);

            signal w_pid_override : std_logic;
        begin

            w_regs_data_in_value_mask((1+REG_INDEX)*4-1 downto (REG_INDEX)*4) <= "0011";
            w_regs_data_in_value_mask((REG_COUNT+REG_INDEX)*4-1 downto (1+REG_INDEX)*4) <= (others=>'1');

            w_pid_override <= std_norm_range(regs_data_out_value((REG_INDEX+2)*32-1 downto (REG_INDEX+1)*32))(8);

            p_async: process(regs_data_out_value,w_pio_data_out_value,w_pid_measure,w_pid_override) is
            begin
                --! in the case the measure is uint32_t instead of float, arg[0] = 1
                if i /= 3-1 then
                    w_pio_data_in_value(1*32-1 downto 0*32)      <= X"00000100";
                else
                    w_pio_data_in_value(1*32-1 downto 0*32)      <= X"00010100";
                end if;
                w_pio_data_in_value((1+REG_COUNT)*32-1 downto 1*32) <= regs_data_out_value((REG_INDEX+REG_COUNT)*32-1 downto REG_INDEX*32);

                if w_pid_override = '0' then
                    w_pio_data_in_value(2*32+8-1 downto 2*32)   <= "0000000" & w_pid_en(i);
                    w_pio_data_in_value(7*32-1 downto 6*32)     <=  w_pid_speed(i);
                    w_pio_data_in_value(8*32-1 downto 7*32)     <=  w_pid_acc(i);
                    w_pio_data_in_value(11*32-1 downto 10*32)   <=  w_pid_target(i);
                end if;

                w_pio_data_in_value((REG_MEASURE_INDEX+1)*32-1 downto REG_MEASURE_INDEX*32) <= w_pid_measure(i);
                w_regs_data_in_value((REG_INDEX+REG_MEASURE_INDEX)*32-1 downto (REG_INDEX+REG_MEASURE_INDEX-1)*32) <= w_pid_measure(i);
            end process;


            w_pid_output(i) <= w_pio_data_out_value((REG_OUTPUT_INDEX+1)*32-1 downto REG_OUTPUT_INDEX*32);

            --! disable warnings
            assert w_pio_data_in_read = w_pio_data_in_read;
            assert w_pio_data_out_write = w_pio_data_out_write;

            inst_motor_pid_rv : component system
            port map (
                clk_clk                 => clk,
                reset_reset_n           => w_reset_n,
                pio_data_in_value       => w_pio_data_in_value,
                pio_data_in_read        => w_pio_data_in_read,
                pio_data_out_value      => w_pio_data_out_value,
                pio_data_out_write      => w_pio_data_out_write,
			    uart_0_external_rxd      => uart_rx(i+1),
			    uart_0_external_txd      => uart_tx(i+1)
            );
        end generate;
    end block;

    p_async: process(w_pid_output) is
        variable v_left,v_right : signed(16+1-1 downto 0);
    begin
        v_left  := signed(w_pid_output(0)(31) & w_pid_output(0)(16-1 downto 0)) + signed(w_pid_output(1)(31) & w_pid_output(1)(16-1 downto 0));
        v_right := signed(w_pid_output(0)(31) & w_pid_output(0)(16-1 downto 0)) - signed(w_pid_output(1)(31) & w_pid_output(1)(16-1 downto 0));

        motor_value(0) <= std_logic_vector(v_left(16-1 downto 0));
        motor_value(1) <= std_logic_vector(v_right(16-1 downto 0));

        if v_left > 2**15-1 then
            motor_value(0) <= std_logic_vector(to_signed(2**15-1,16));
        end if;
        if v_left < -2**15 then
            motor_value(0) <= std_logic_vector(to_signed(-2**15-1,16));
        end if;

        if v_right > 2**15-1 then
            motor_value(1) <= std_logic_vector(to_signed(2**15-1,16));
        end if;
        if v_right < -2**15 then
            motor_value(1) <= std_logic_vector(to_signed(-2**15-1,16));
        end if;

    end process;

    motor_value(2) <= w_pid_output(2)(31) & w_pid_output(2)(15-1 downto 0);
    motor_value(3) <= (others=>'0');
    motor_value(4) <= (others=>'0');
    motor_value(5) <= (others=>'0');




end architecture;

