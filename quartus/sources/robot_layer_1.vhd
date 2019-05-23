library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;


library work;
use     work.uart_pkg.all;
use     work.spi_master_pkg.all;
use     work.qei_pkg.all;
use     work.pwm_pkg.all;
use     work.debounce_pkg.all;
use     work.types_pkg.all;
use     work.robot_layer_1_pkg.all;

entity robot_layer_1 is
    generic (
        CLK_FREQUENCY_HZ : positive;
        RegCnt : positive
    );
    port (
        clk                     : in  std_logic;             
        reset                   : in  std_logic;             

        ---------------------------------
        ------ TO/FROM SOFTWARE/OS ------
        ---------------------------------        

        regs_data_in_value      : out  std_logic_vector(RegCnt*32-1 downto 0) := (others => '0'); 
        regs_data_in_read       : in std_logic_vector(RegCnt-1 downto 0);                       
        regs_data_out_value     : in std_logic_vector(RegCnt*32-1 downto 0);                    
        regs_data_out_write     : in std_logic_vector(RegCnt-1 downto 0);

        sw_uart_tx : in  std_logic_vector(SW_UART_L1_COUNT-1 downto 0);
        sw_uart_rx : out std_logic_vector(SW_UART_L1_COUNT-1 downto 0);        
        
        ---------------------------------
        ---------- TO/FROM IOs ----------
        ---------------------------------   
        
        ----------- ADC (//) ---------
        ad0_sclk : out std_logic;
        ad0_miso : in  std_logic;
        ad0_drdy : in  std_logic;
        ad0_sync : out std_logic;
        ad0_clk  : out std_logic;

        --------- ADC (muxed) --------
        ad1_sclk : out std_logic;
        ad1_mosi : out std_logic;
        ad1_miso : in  std_logic;
        ad1_ss   : out std_logic;
        ad1_drdy : in  std_logic;
        ad1_rst  : out std_logic;

        ---------- H BRIDGE ----------
        m0_pwma  : out std_logic;
        m0_pwmb  : out std_logic;
        m01_fault: in  std_logic; --m01_fault
        
        m1_pwma  : out std_logic;
        m1_pwmb  : out std_logic;
        m01_resetn: out  std_logic; --m01_resetn

        m2_pwma  : out std_logic;
        m2_pwmb  : out std_logic;

        m3_pwma  : out std_logic;
        m3_pwmb  : out std_logic;

        m2345_fault: in  std_logic; --m2345_fault

        m4_pwma  : out std_logic;
        m4_pwmb  : out std_logic;

        m5_pwma  : out std_logic;
        m5_pwmb  : out std_logic;

        m2345_resetn: out  std_logic; --m2345_resetn

        ---------- QEI ----------    
        qei0_a   : in  std_logic;
        qei0_b   : in  std_logic;

        qei1_a   : in  std_logic;
        qei1_b   : in  std_logic;

        qei2_a   : in  std_logic;
        qei2_b   : in  std_logic;
        qei2_z   : in  std_logic;

        qei3_a   : in  std_logic;
        qei3_b   : in  std_logic;
        qei3_z   : in  std_logic;

        ---------- ESC ----------    
        esc0_pwm : out std_logic;
        esc0_dir : out std_logic;

        esc1_pwm : out std_logic;
        esc1_dir : out std_logic;

        ------- PWM (Servos) ------
        s : out std_logic_vector(8-1 downto 0);    

        --------- IOs ----------
        io_0 : inout  std_logic;
        io_1 : inout  std_logic;
        io_2 : inout  std_logic;
        io_3 : inout  std_logic;
        io_4 : inout  std_logic;
        io_5 : inout  std_logic;
        io_6 : inout  std_logic;
        io_7 : inout  std_logic;

        --------- UART ----------
        uart0_rx     : in  std_logic;
        uart0_tx     : out std_logic;

        uart1_rx     : in  std_logic;
        uart1_tx     : in  std_logic;

        uart2_rx     : in  std_logic;
        uart2_tx     : out std_logic;
        uart2_custom : out std_logic;

        uart3_rx     : inout std_logic;
        uart3_tx     : in    std_logic;
        uart3_custom : out std_logic;

        --------- I2C ----------
        i2c0_scl     : in std_logic;
        i2c0_sda     : in std_logic;
        i2c0_reset   : in std_logic;

        i2c1_scl     : in std_logic;
        i2c1_sda     : in std_logic;
        i2c1_reset   : in std_logic;

        --------- SPI ----------
        spi0_sclk    : out std_logic;
        spi0_mosi    : in  std_logic;
        spi0_miso    : in  std_logic;
        spi0_ss      : out std_logic;

        spi1_sclk    : out std_logic;
        spi1_mosi    : in  std_logic;
        spi1_miso    : in  std_logic;
        spi1_ss      : out std_logic;

        --! Use SPI1
        imu_ss       : out std_logic;
        imu_drdy     : in  std_logic;
        imu_fsync    : in  std_logic;

        ---------- LED -----------
        led_green : out std_logic;
        led_red   : out std_logic;

        --------- MGMT -----------
        lv_mux    : out std_logic_vector(2-1 downto 0);
        buzzer    : out std_logic;

	    ----------/ NANO SOC LED --------/
	    LED                 : out   std_logic_vector(8-1 downto 0);

	    ----------/ NANO SOC SW --------/
	    SW                  : in    std_logic_vector(4-1 downto 0);


        ---------------------------------
        -------- TO/FROM LAYER 2 --------
        ---------------------------------

        --------- UART ----------
        uart_tx       : in  std_logic_vector(4-1 downto 0);
        uart_rx       : out std_logic_vector(4-1 downto 0);

        motor_value   : in  int16_t(MOTOR_COUNT-1 downto 0);
        motor_current : out int24_t(MOTOR_COUNT-1 downto 0);
        motor_fault   : out std_logic_vector(MOTOR_COUNT-1 downto 0);

        qei_value     : out int16_t(QEI_COUNT-1 downto 0);
        qei_ref       : out  std_logic_vector(QEI_COUNT-1 downto 0)



    );
end entity;

architecture rtl of robot_layer_1 is

	component system_ll is
		port (
			clk_clk                    : in  std_logic                      := 'X';             -- clk
			--i2c_master_serial_0_sda_in : in  std_logic                      := 'X';             -- sda_in
			--i2c_master_serial_0_scl_in : in  std_logic                      := 'X';             -- scl_in
			--i2c_master_serial_0_sda_oe : out std_logic;                                         -- sda_oe
			--i2c_master_serial_0_scl_oe : out std_logic;                                         -- scl_oe
			--i2c_master_serial_1_sda_in : in  std_logic                      := 'X';             -- sda_in
			--i2c_master_serial_1_scl_in : in  std_logic                      := 'X';             -- scl_in
			--i2c_master_serial_1_sda_oe : out std_logic;                                         -- sda_oe
			--i2c_master_serial_1_scl_oe : out std_logic;                                         -- scl_oe
			pio_data_in_value          : in  std_logic_vector(511 downto 0) := (others => 'X'); -- data_in_value
			pio_data_in_read           : out std_logic_vector(15 downto 0);                     -- data_in_read
			pio_data_out_value         : out std_logic_vector(511 downto 0);                    -- data_out_value
			pio_data_out_write         : out std_logic_vector(15 downto 0);                     -- data_out_write
			reset_reset_n              : in  std_logic                      := 'X';             -- reset_n
			uart_0_external_rxd        : in  std_logic                      := 'X';             -- rxd
			uart_0_external_txd        : out std_logic;
			uart_0_external_transmitting: out std_logic 
		);
	end component system_ll;


    signal w_reset_n : std_logic;

    signal w_regs_data_in_value      : std_logic_vector(RegCnt*32-1 downto 0);
    signal w_regs_data_in_value_mask : std_logic_vector(RegCnt*4-1 downto 0) := (others=>'0');


    constant REG_MOTOR_OFFSET           : natural := 12;
    constant REG_MOTOR_CURRENT_OFFSET   : natural := REG_MOTOR_OFFSET+MOTOR_COUNT;

    signal w_motor_value    : int16_t(MOTOR_COUNT-1 downto 0);
    signal w_motor_invert   : std_logic_vector(MOTOR_COUNT-1 downto 0);
    signal w_motor_override : std_logic_vector(MOTOR_COUNT-1 downto 0);
    signal w_motor_current  : int24_t(MOTOR_COUNT-1 downto 0);
    signal w_motor_out      : std_logic_vector(MOTOR_COUNT-1 downto 0);
    signal w_motor_dir      : std_logic_vector(MOTOR_COUNT-1 downto 0);


    constant REG_SERVO_OFFSET : natural := 30;

    constant SERVO_COUNT : natural := 8;

    signal w_servo_value    : int16_t(SERVO_COUNT-1 downto 0);
    signal w_servo_enabled  : std_logic_vector(SERVO_COUNT-1 downto 0);
    signal w_servo_full_range : std_logic_vector(SERVO_COUNT-1 downto 0);
    signal w_servo_override : std_logic_vector(SERVO_COUNT-1 downto 0);
    signal w_servo_current  : int24_t(SERVO_COUNT-1 downto 0);
    signal w_servo_out      : std_logic_vector(SERVO_COUNT-1 downto 0);


    constant REG_ESC_OFFSET : natural := 46;
    constant ESC_COUNT : natural := 2;

    signal w_esc_value    : int16_t(ESC_COUNT-1 downto 0);
    signal w_esc_enabled  : std_logic_vector(ESC_COUNT-1 downto 0);
    signal w_esc_override : std_logic_vector(ESC_COUNT-1 downto 0);
    signal w_esc_current  : int24_t(ESC_COUNT-1 downto 0);
    signal w_esc_out      : std_logic_vector(ESC_COUNT-1 downto 0);
    signal w_esc_dir      : std_logic_vector(ESC_COUNT-1 downto 0);

    constant REG_PWM_CUSTOM_OFFSET : natural := 48;
    constant PWM_CUSTOM_COUNT : natural := 4;

    signal w_pwm_custom_value    : int16_t(PWM_CUSTOM_COUNT-1 downto 0);
    signal w_pwm_custom_out      : std_logic_vector(PWM_CUSTOM_COUNT-1 downto 0);
    signal w_pwm_custom_dir      : std_logic_vector(PWM_CUSTOM_COUNT-1 downto 0);



    constant REG_QEI_OFFSET : natural := 24;

    signal w_qei_a          : std_logic_vector(QEI_COUNT-1 downto 0);
    signal w_qei_b          : std_logic_vector(QEI_COUNT-1 downto 0);
    signal w_qei_z          : std_logic_vector(QEI_COUNT-1 downto 0);
    signal w_qei_override   : std_logic_vector(QEI_COUNT-1 downto 0);
    signal w_qei_value      : int16_t(QEI_COUNT-1 downto 0);
    signal w_qei_ref        : std_logic_vector(QEI_COUNT-1 downto 0);


    signal r_led_red  : std_logic;


    signal w_ad0_rx_data   : std_logic_vector(192-1 downto 0);


    constant VOLTAGE_COUNT : natural := 4;
    constant REG_VOLTAGE_OFFSET : natural := 2;

    signal r_voltage      : int24_t(VOLTAGE_COUNT-1 downto 0); 
    signal r_lv_mux       : std_logic_vector(2-1 downto 0);
    signal r_voltage_cnt  : std_logic_vector(7-1 downto 0); 

    constant REG_BUZZER_OFFSET : natural := 6;

    signal r_buzzer            : std_logic;
    signal w_buzzer_override   : std_logic;
    signal w_buzzer_out        : std_logic;

    constant REG_IO_OFFSET : natural := 7;
    constant IO_COUNT : natural := 4;

    signal w_input_in      : std_logic_vector(IO_COUNT-1 downto 0);
    signal w_input_override: std_logic_vector(IO_COUNT-1 downto 0);
    signal w_input_value   : std_logic_vector(IO_COUNT-1 downto 0);

    signal w_output_value  : std_logic_vector(IO_COUNT-1 downto 0);
    signal w_output_override: std_logic_vector(IO_COUNT-1 downto 0);
    signal w_output_out    : std_logic_vector(IO_COUNT-1 downto 0);

    constant REG_STATE_CONFIG_OFFSET : natural := 1;
    signal w_sim_mode : std_logic;


    signal w_i2c_0_scl     : std_logic;
    signal w_i2c_0_sda     : std_logic;
    signal w_i2c_0_scl_oe  : std_logic;
    signal w_i2c_0_sda_oe  : std_logic;
    signal w_i2c_0_reset   : std_logic;

    signal w_i2c_1_scl     : std_logic;
    signal w_i2c_1_sda     : std_logic;
    signal w_i2c_1_scl_oe  : std_logic;
    signal w_i2c_1_sda_oe  : std_logic;
    signal w_i2c_1_reset   : std_logic;

	 signal r_ll_uart_rs485 : std_logic := '1';
	 signal w_ll_uart_rxd : std_logic;
	 signal w_ll_uart_txd : std_logic;
	 signal w_ll_uart_transmitting : std_logic;
	 

begin
	
    w_reset_n <= not reset;

    imu_ss <= '1';

    --! we return for read the same written data, expect for some bytes (noted masked) where we compute the value internally
    g_reg: for i in 0 to w_regs_data_in_value_mask'length-1 generate
        regs_data_in_value((i+1)*8-1 downto i*8) <= regs_data_out_value((i+1)*8-1 downto i*8) when w_regs_data_in_value_mask(i) = '0' else w_regs_data_in_value((i+1)*8-1 downto i*8);
    end generate;


    b_ads1278: block
        signal r_clk_25mhz : std_logic;
        signal r_clk_12mhz : std_logic;


        signal r_ad_drdy  : std_logic;
        signal r_ad_en    : std_logic;


        signal w_ad_rx_busy   : std_logic;
        signal r_ad_rx_busy   : std_logic;
        signal w_ad_rx_data   : std_logic_vector(192-1 downto 0);
        signal r_ad_rx_valid  : std_logic;
    begin

        p_sync_spi: process(clk) is
        begin
            if rising_edge(clk) then
                r_clk_25mhz <= not r_clk_25mhz;
                r_ad_drdy  <= ad0_drdy;

                if r_clk_25mhz = '1' then
                    r_clk_12mhz <= not r_clk_12mhz;
                end if;
                
                r_ad_en <= '0';

                if ad0_drdy = '0' and r_ad_drdy = '1' then
                    r_ad_en <= '1';
                end if;
            end if;
        end process;

        ad0_clk <= r_clk_12mhz;

        ad0_sync <= '1';

        inst_spi_ad: spi_master
        generic map(
            slaves => 1,
            d_width => w_ad_rx_data'length
        )
        port map (
            clock     => clk,
            reset_n => w_reset_n,                             
            enable  => r_ad_en,                               
            cpol    => '0',
            cpha    => '0',
            cont    => '0',
            clk_div => 2,
            addr    => 0,
            tx_data => (others=>'0'),
            miso    => ad0_miso,
            sclk    => ad0_sclk,
            ss_n    => open,
            mosi    => open,
            ss_n    => open,
            busy    => w_ad_rx_busy,
            rx_data => w_ad_rx_data

        );	 

        p_sync_ad_output: process(clk,reset) is
        begin
            if reset = '1' then
                r_ad_rx_valid <= '0';
                r_ad_rx_busy <= '0';
            elsif rising_edge(clk) then
        
                r_ad_rx_busy <= w_ad_rx_busy;
                r_ad_rx_valid<= '0';

                if w_ad_rx_busy = '0' and r_ad_rx_busy = '1' then
                    r_ad_rx_valid <= '1';
                end if;
            end if;
        end process;

        p_sync_voltage_mux: process(clk,reset) is
        begin
            if reset = '1' then
                r_voltage_cnt <= (others=>'0');
                r_lv_mux      <= (others=>'0');
                r_buzzer      <= '0';
            elsif rising_edge(clk) then
                if r_ad_rx_valid = '1' then
                    r_voltage_cnt <= std_logic_vector(unsigned(r_voltage_cnt)+1);
                    if r_voltage_cnt = (r_voltage_cnt'range=>'1') then
                        r_lv_mux <= std_logic_vector(unsigned(r_lv_mux)+1);
                        r_voltage(to_integer(unsigned(r_lv_mux))) <= w_ad_rx_data(8*24-1 downto 7*24);
                    end if;
                end if;
                
                r_buzzer <= '0';
                for i in 0 to VOLTAGE_COUNT-1 loop
                    if unsigned(w_regs_data_in_value(((i+1)+REG_VOLTAGE_OFFSET)*32-1 downto (i+REG_VOLTAGE_OFFSET)*32)) < unsigned(regs_data_out_value(((i+1)+REG_VOLTAGE_OFFSET)*32-1 downto (i+REG_VOLTAGE_OFFSET)*32)) then
                        r_buzzer <= '1';
                    end if;
                end loop;
            end if;
        end process;

        b_buzzer: block
            signal w_reg     : std_logic_vector(32-1 downto 0);
        begin
            w_reg <= regs_data_out_value((REG_BUZZER_OFFSET+1)*32-1 downto REG_BUZZER_OFFSET*32);

            w_buzzer_override <= w_reg(8);

            w_buzzer_out <= r_buzzer when w_buzzer_override = '0' else w_reg(0);

            w_regs_data_in_value((REG_BUZZER_OFFSET+1)*32-1 downto REG_BUZZER_OFFSET*32) <= X"0000" & X"00" & "0000000" & w_buzzer_out;  
            w_regs_data_in_value_mask((REG_BUZZER_OFFSET+1)*4-1 downto REG_BUZZER_OFFSET*4) <= "0001";

            g_voltage_mux: for i in 0 to VOLTAGE_COUNT-1 generate
            begin    
                w_regs_data_in_value(((i+1)+REG_VOLTAGE_OFFSET)*32-1 downto (i+REG_VOLTAGE_OFFSET)*32) <= (8-1 downto 0=>r_voltage(i)(r_voltage(i)'high)) & r_voltage(i);
                w_regs_data_in_value_mask(((i+1)+REG_VOLTAGE_OFFSET)*4-1 downto (i+REG_VOLTAGE_OFFSET)*4) <= (others=>'1');
            end generate;
        end block;

        buzzer <= w_buzzer_out;

        w_ad0_rx_data <= w_ad_rx_data;
    end block;




--		    spi_master_0_MISO          => ad1_miso,          --        spi_master_0.MISO
--		    spi_master_0_MOSI          => ad1_mosi,          --                    .MOSI
--		    spi_master_0_SCLK          => ad1_sclk ,          --                    .SCLK
--		    spi_master_0_SS_n          => ad1_ss           --  

--        ad1_sclk : out std_logic;
--        ad1_mosi : out std_logic;
--        ad1_miso : in  std_logic;
--        ad1_ss   : out std_logic;
--        ad1_drdy : in  std_logic;
--        ad1_rst  : out std_logic;

    b_ads1258: block

        signal r_ad_drdy  : std_logic;
        signal r_ad_en    : std_logic;


        signal w_ad_rx_busy   : std_logic;
        signal r_ad_rx_busy   : std_logic;
        signal r_ad_tx_data   : std_logic_vector(8*4-1 downto 0);
        signal w_ad_rx_data   : std_logic_vector(8*4-1 downto 0);
        signal r_ad_rx_valid  : std_logic;

        constant ADC_COUNT : natural := 8;

        signal r_adc_values   : int24_t(ADC_COUNT-1 downto 0); 

        constant REGS_ADC_SERVO_OFFSET : natural := 38;

        signal r_state : std_logic_vector(2-1 downto 0);
        signal r_cnt : natural;
    begin

    --uint8_t spi_buf1[2] = {0x60, 0x18};
    --uint8_t spi_buf2[2] = {0x61, 0x70};
    
        p_sync_spi: process(clk,reset) is
        begin
            if (reset = '1') then
                ad1_rst     <= '0';
                r_ad_drdy   <= '0';
                r_ad_en     <= '0';
                r_ad_tx_data <= (others=>'0');
                r_state     <= "00";
                r_cnt <= 0;
            elsif rising_edge(clk) then
                r_ad_en <= '0'; 

                case r_state is
                    when "00" =>
                        ad1_rst     <= '1';
                        r_state     <= "01";
                    when "01" =>
                        r_cnt <= r_cnt + 1;
                        if w_ad_rx_busy = '0' and r_cnt = 250000000 then
                            r_ad_tx_data <= X"60" & X"1A" & X"60" & X"1A";
                            r_ad_en <= '1';
                            r_state <= "10";
                        end if;
                    when "10" =>
                        if r_ad_rx_valid = '1' then
                            r_ad_tx_data <= X"61" & X"70" & X"61" & X"70";
                            r_ad_en <= '1';
                            r_state <= "11";
                        end if;
                    when "11" =>
                        r_ad_drdy   <= ad1_drdy;

                        if ad1_drdy = '0' and r_ad_drdy = '1' and w_ad_rx_busy = '0' then
                            r_ad_tx_data <= (others=>'0');
                            r_ad_en <= '1';
                        end if;
                end case;

            end if;
        end process;


        inst_spi_ad: spi_master
        generic map(
            slaves => 1,
            d_width => w_ad_rx_data'length
        )
        port map (
            clock     => clk,
            reset_n => w_reset_n,                             
            enable  => r_ad_en,                               
            cpol    => '0',
            cpha    => '0',
            cont    => '0',
            clk_div => 8,
            addr    => 0,
            tx_data => r_ad_tx_data,
            miso    => ad1_miso,
            mosi    => ad1_mosi,
            sclk    => ad1_sclk,
            ss_n(0) => ad1_ss,
            busy    => w_ad_rx_busy,
            rx_data => w_ad_rx_data

        );	 
        --spi1_ss <= '0';
        --ad1_ss <= '0';

        p_sync_ad_output: process(clk,reset) is
        begin
            if reset = '1' then
                r_ad_rx_valid <= '0';
                r_ad_rx_busy <= '0';
            elsif rising_edge(clk) then
        
                r_ad_rx_busy <= w_ad_rx_busy;
                r_ad_rx_valid<= '0';

                if w_ad_rx_busy = '0' and r_ad_rx_busy = '1' then
                    r_ad_rx_valid <= '1';
                end if;
            end if;
        end process;

        p_ad_read: process(clk,reset) is
        begin
            if reset = '1' then
                r_adc_values <= (others=>(others=>'0'));
            elsif rising_edge(clk) then
                if r_ad_rx_valid = '1' then
                    if unsigned(w_ad_rx_data(32-3-1 downto 32-8)) >= 16 and unsigned(w_ad_rx_data(32-3-1 downto 32-8)) <= 23 then
                        r_adc_values(to_integer(unsigned(w_ad_rx_data(32-3-1 downto 32-8)))-16) <= w_ad_rx_data(3*8-1 downto 0);
                    end if;
                end if;  
            end if;
        end process;

        b_adc_regs: block
        begin
            g_mux: for i in 0 to ADC_COUNT-1 generate
            begin    
                w_regs_data_in_value(((i+1)+REGS_ADC_SERVO_OFFSET)*32-1 downto (i+REGS_ADC_SERVO_OFFSET)*32) <= (8-1 downto 0=>r_adc_values(i)(r_adc_values(i)'high)) & r_adc_values(i);
                w_regs_data_in_value_mask(((i+1)+REGS_ADC_SERVO_OFFSET)*4-1 downto (i+REGS_ADC_SERVO_OFFSET)*4) <= (others=>'1');
            end generate;
        end block;

    end block;





    w_input_in(0) <= io_0 when w_sim_mode = '0' else SW(0);
    w_input_in(1) <= io_2 when w_sim_mode = '0' else SW(1);
    w_input_in(2) <= io_3 when w_sim_mode = '0' else SW(2);
    w_input_in(3) <= io_4 when w_sim_mode = '0' else SW(3);

    io_0 <= 'Z';
    io_2 <= 'Z';
    io_3 <= 'Z';
    io_4 <= 'Z';



    b_io: block
        signal w_input_in_filtered   : std_logic_vector(IO_COUNT-1 downto 0);

        signal w_input_value_reg     : std_logic_vector(32-1 downto 0);
        signal w_input_override_reg  : std_logic_vector(32-1 downto 0);

        signal w_output_value_reg     : std_logic_vector(32-1 downto 0);
        signal w_output_override_reg  : std_logic_vector(32-1 downto 0);
    begin
     

        w_input_value_reg    <= regs_data_out_value(((0+1)+REG_IO_OFFSET)*32-1 downto (0+REG_IO_OFFSET)*32);
        w_input_override_reg <= regs_data_out_value(((1+1)+REG_IO_OFFSET)*32-1 downto (1+REG_IO_OFFSET)*32);

        w_output_value_reg    <= regs_data_out_value(((2+1)+REG_IO_OFFSET)*32-1 downto (2+REG_IO_OFFSET)*32);
        w_output_override_reg <= regs_data_out_value(((3+1)+REG_IO_OFFSET)*32-1 downto (3+REG_IO_OFFSET)*32);


        w_regs_data_in_value_mask(((0+1)+REG_IO_OFFSET)*4-1 downto (0+REG_IO_OFFSET)*4) <= (others=>'1');
        w_regs_data_in_value_mask(((2+1)+REG_IO_OFFSET)*4-1 downto (2+REG_IO_OFFSET)*4) <= (others=>'1');

        g_io: for i in 0 to IO_COUNT-1 generate
        begin
            inst_in_filter: debounce
            generic map(
                counter_size  => 19 --counter size (19 bits gives 10.5ms with 50MHz clock)
            ) 
            port map(
                clk     => clk,
                button  => w_input_in(i),
                result  => w_input_in_filtered(i)
            );

            w_input_override(i) <= w_input_override_reg(i*8);
            w_input_value(i) <= w_input_in_filtered(i) when w_input_override(i) = '0' else w_input_value_reg(i*8);

            --! to be completed by Layer 2 signals
            w_output_value(i) <= '0';

            w_output_override(i) <= w_output_override_reg(i*8);
            w_output_out(i)      <= w_output_value(i) when w_output_override(i) = '0' else w_output_value_reg(i*8);

            w_regs_data_in_value((0+REG_IO_OFFSET)*32+(i+1)*8-1 downto (0+REG_IO_OFFSET)*32+i*8) <= "0000000" & w_input_value(i);
            w_regs_data_in_value((2+REG_IO_OFFSET)*32+(i+1)*8-1 downto (2+REG_IO_OFFSET)*32+i*8) <= "0000000" & w_output_value(i);

        end generate;

    end block;



    io_1 <= w_output_out(0);
    io_6 <= w_output_out(1);
    io_7 <= w_output_out(2);
    io_5 <= w_output_out(3);



    b_state_config: block
        signal w_reg   : std_logic_vector(32-1 downto 0);
    begin
        w_reg    <= regs_data_out_value(((0+1)+REG_STATE_CONFIG_OFFSET)*32-1 downto (0+REG_STATE_CONFIG_OFFSET)*32);
        
        w_sim_mode <= w_reg(8);
    end block;




    w_motor_current(0) <= w_ad0_rx_data(1*24-1 downto 0*24);
    w_motor_current(1) <= w_ad0_rx_data(2*24-1 downto 1*24);
    w_motor_current(2) <= w_ad0_rx_data(3*24-1 downto 2*24);
    w_motor_current(3) <= w_ad0_rx_data(4*24-1 downto 3*24);
    w_motor_current(4) <= w_ad0_rx_data(5*24-1 downto 4*24);
    w_motor_current(5) <= w_ad0_rx_data(6*24-1 downto 5*24);

    motor_current <= w_motor_current;

    motor_fault(0) <= m01_fault;
    motor_fault(1) <= m01_fault;
    motor_fault(2) <= m2345_fault;
    motor_fault(3) <= m2345_fault;
    motor_fault(4) <= m2345_fault;
    motor_fault(5) <= m2345_fault;

    w_regs_data_in_value(((0+1)+REG_MOTOR_OFFSET-1)*32-1 downto (0+REG_MOTOR_OFFSET-1)*32) <= X"0000" & "0000000" & m2345_fault & "0000000" & m01_fault;
    w_regs_data_in_value_mask(((0+1)+REG_MOTOR_OFFSET-1)*4-1 downto (0+REG_MOTOR_OFFSET-1)*4) <= (others=>'1');



    g_motor: for i in 0 to MOTOR_COUNT-1 generate
        signal w_duty : std_logic_vector(15-1 downto 0);
        signal w_reg  : std_logic_vector(32-1 downto 0);
    begin
        
        w_regs_data_in_value(((i+1)+REG_MOTOR_CURRENT_OFFSET)*32-1 downto (i+REG_MOTOR_CURRENT_OFFSET)*32) <= (8-1 downto 0=>w_motor_current(i)(w_motor_current(i)'high)) & w_motor_current(i);
        w_regs_data_in_value_mask(((i+1)+REG_MOTOR_CURRENT_OFFSET)*4-1 downto (i+REG_MOTOR_CURRENT_OFFSET)*4) <= (others=>'1');

        w_reg <= regs_data_out_value(((i+1)+REG_MOTOR_OFFSET)*32-1 downto (i+REG_MOTOR_OFFSET)*32);

        w_motor_invert(i) <= w_reg(16);
        w_motor_override(i) <= w_reg(24);

        w_motor_value(i) <= w_reg(16-1 downto 0) when w_motor_override(i) = '1' else motor_value(i);



        w_duty <= std_logic_vector(abs(signed(w_motor_value(i))))(w_duty'range);

        inst_pwm_motor: pwm 
        generic map(
            sys_clk         => CLK_FREQUENCY_HZ, --system clock frequency in Hz
            pwm_freq        => 20_000,     --PWM switching frequency in Hz
            bits_resolution => w_duty'length,         --bits of resolution setting the duty cycle
            phases          => 1           --number of out : pwms and phases
        )
        port map (
            clk       => clk,
            reset_n   => w_reset_n,                             
            ena       => '1',                               
            duty      => w_duty,
            pwm_out(0)=> w_motor_out(i)
        );
        
        w_motor_dir(i) <= w_motor_value(i)(w_motor_value(i)'high) when w_motor_invert(i) = '0' else not w_motor_value(i)(w_motor_value(i)'high);

    end generate;

    LED(0) <= w_motor_out(0) when w_motor_dir(0) = '0' else not w_motor_out(0);
    LED(1) <= w_motor_out(1) when w_motor_dir(1) = '0' else not w_motor_out(1);
    LED(2) <= w_motor_out(2) when w_motor_dir(2) = '0' else not w_motor_out(2);
    LED(3) <= w_motor_out(3) when w_motor_dir(3) = '0' else not w_motor_out(3);


    LED(4) <= w_output_out(0);
    LED(5) <= w_output_out(1);
    LED(6) <= w_output_out(2);
    LED(7) <= w_output_out(3);


    m0_pwma  <= w_motor_out(0) and not w_motor_dir(0);
    m0_pwmb  <= w_motor_out(0) and w_motor_dir(0);

    m1_pwma  <= w_motor_out(1) and not w_motor_dir(1);
    m1_pwmb  <= w_motor_out(1) and w_motor_dir(1);

    m2_pwma  <= w_motor_out(2) and not w_motor_dir(2);
    m2_pwmb  <= w_motor_out(2) and w_motor_dir(2);

    m3_pwma  <= w_motor_out(3) and not w_motor_dir(3);
    m3_pwmb  <= w_motor_out(3) and w_motor_dir(3);

    m4_pwma  <= w_motor_out(4) and not w_motor_dir(4);
    m4_pwmb  <= w_motor_out(4) and w_motor_dir(4);

    m5_pwma  <= w_motor_out(5) and not w_motor_dir(5);
    m5_pwmb  <= w_motor_out(5) and w_motor_dir(5);


    --uart3_custom <= w_motor_out(5);


    b_blk_motor: block
        signal w_reg : std_logic_vector(32-1 downto 0);
    begin
        w_reg <= regs_data_out_value(((0+1)+REG_MOTOR_OFFSET-1)*32-1 downto (0+REG_MOTOR_OFFSET-1)*32);

        p_sync_reset: process(clk, w_reset_n) is
        begin
            if (w_reset_n = '0') then
                m01_resetn <= '0';
                m2345_resetn <= '0';
            elsif rising_edge(clk) then
                m01_resetn <= '1' and not w_reg(0);
                m2345_resetn <= '1' and not w_reg(8);
            end if;
        end process;
    end block;

    g_servo: for i in 0 to SERVO_COUNT-1 generate
        signal w_duty : std_logic_vector(16-1 downto 0);
        signal w_reg  : std_logic_vector(32-1 downto 0);
    begin

        w_reg <= regs_data_out_value(((i+1)+REG_SERVO_OFFSET)*32-1 downto (i+REG_SERVO_OFFSET)*32);

        w_servo_enabled(i)    <= w_reg(16);
        w_servo_full_range(i) <= w_reg(17);
        w_servo_override(i) <= w_reg(24);

        w_servo_value(i) <= w_reg(16-1 downto 0) when w_servo_override(i) = '1' else (others=>'0');

        w_duty <= std_logic_vector(to_unsigned(2**16/5,w_duty'length)+unsigned(w_servo_value(i))*52)(w_duty'range) when w_servo_enabled(i) = '1' and w_servo_full_range(i) = '0' 
             else w_servo_value(i) when w_servo_enabled(i) = '1' and w_servo_full_range(i) = '1'
             else (others=>'0'); 

        inst_pwm_servo: pwm 
        generic map(
            sys_clk         => CLK_FREQUENCY_HZ, --system clock frequency in Hz
            pwm_freq        => 200,     --PWM switching frequency in Hz
            bits_resolution => w_duty'length,         --bits of resolution setting the duty cycle
            phases          => 1           --number of out : pwms and phases
        )
        port map (
            clk       => clk,
            reset_n   => w_reset_n,                             
            ena       => '1',         
            duty      => w_duty,
            pwm_out(0)=> w_servo_out(i)
        );
    
    end generate;


    s <= w_servo_out;


    g_esc: for i in 0 to ESC_COUNT-1 generate
        signal w_duty : std_logic_vector(12-1 downto 0);
        signal w_reg  : std_logic_vector(32-1 downto 0);
    begin

        w_reg <= regs_data_out_value(((i+1)+REG_ESC_OFFSET)*32-1 downto (i+REG_ESC_OFFSET)*32);

        w_esc_enabled(i) <= w_reg(16);
        w_esc_override(i) <= w_reg(24);

        w_esc_value(i) <= w_reg(16-1 downto 0) when w_esc_override(i) = '1' else (others=>'0');

        w_duty <= std_logic_vector(to_unsigned(256,w_duty'length)+unsigned(abs(signed(w_esc_value(i)))))(w_duty'range) when w_esc_enabled(i) = '1' else (others=>'0');

        inst_pwm_esc: pwm 
        generic map(
            sys_clk         => CLK_FREQUENCY_HZ*2,   --system clock frequency in Hz
            pwm_freq        => 625*2/10,             --PWM switching frequency in Hz
            bits_resolution => w_duty'length,      --bits of resolution setting the duty cycle
            phases          => 1                   --number of out : pwms and phases
        )
        port map (
            clk       => clk,
            reset_n   => w_reset_n,                             
            ena       => '1',                               
            duty      => w_duty,
            pwm_out(0)=> w_esc_out(i)
        );

        w_esc_dir(i) <= w_esc_value(i)(w_esc_value(i)'high);
    
    end generate;



    g_pwm_custom: for i in 0 to PWM_CUSTOM_COUNT-1 generate
        signal w_duty : std_logic_vector(15-1 downto 0);
        signal w_reg  : std_logic_vector(16-1 downto 0);
    begin
        
        w_reg <= regs_data_out_value(REG_PWM_CUSTOM_OFFSET*32+16*(i+1)-1 downto REG_PWM_CUSTOM_OFFSET*32+16*i);


        w_pwm_custom_value(i) <= w_reg;

        w_duty <= std_logic_vector(abs(signed(w_pwm_custom_value(i))))(w_duty'range);

        inst_pwm_custom: pwm 
        generic map(
            sys_clk         => CLK_FREQUENCY_HZ, --system clock frequency in Hz
            pwm_freq        => 25_000,     --PWM switching frequency in Hz
            bits_resolution => w_duty'length,         --bits of resolution setting the duty cycle
            phases          => 1           --number of out : pwms and phases
        )
        port map (
            clk       => clk,
            reset_n   => w_reset_n,                             
            ena       => '1',                               
            duty      => w_duty,
            pwm_out(0)=> w_pwm_custom_out(i)
        );
        
        w_pwm_custom_dir(i) <= w_pwm_custom_value(i)(w_pwm_custom_value(i)'high);
       
    end generate;



    lv_mux <= r_lv_mux;
    
    esc0_pwm <= w_esc_out(0);
    esc1_pwm <= w_esc_out(1);

    
    esc0_dir <= w_esc_dir(0);
    esc1_dir <= w_esc_dir(1);

	 
    w_qei_a(0) <= qei0_a;
    w_qei_b(0) <= qei0_b;
    w_qei_z(0) <= '0';

    w_qei_a(1) <= qei1_a;
    w_qei_b(1) <= qei1_b;
    w_qei_z(1) <= '0';

    w_qei_a(2) <= qei2_a;
    w_qei_b(2) <= qei2_b;
    w_qei_z(2) <= qei2_z;

    w_qei_a(3) <= qei3_a;
    w_qei_b(3) <= qei3_b;
    w_qei_z(3) <= qei3_z;

    w_qei_a(4) <= i2c0_sda;--spi0_sclk;
    w_qei_b(4) <= i2c0_reset;--spi0_ss;
    w_qei_z(4) <= i2c0_scl;--not spi0_mosi;

    w_qei_a(5) <= i2c1_sda;--spi1_sclk;
    w_qei_b(5) <= i2c1_reset;--spi1_ss;
    w_qei_z(5) <= i2c1_scl;--not spi1_mosi;

    qei_value <= w_qei_value;
    qei_ref   <= w_qei_ref;

    g_qei: for i in 0 to QEI_COUNT-1 generate
        signal w_reg    : std_logic_vector(32-1 downto 0);
        signal w_cnt    : std_logic_vector(16-1 downto 0);
        signal r_qei_z  : std_logic;
        signal r2_qei_z : std_logic;
        signal r_ref    : std_logic;

        signal r_qei_value_simu : std_logic_vector(16-1 downto 0);

        function get_id_motor(qei_id : natural) return natural is
        begin
            if qei_id = 0 or qei_id = 2 then
                return 0;
            end if;

            if qei_id = 1 or qei_id = 3 then
                return 1;
            end if;

            if qei_id = 4 then
                return 2;
            end if;

            return 3;        
        end function;

        constant SIMU_ID_MOTOR : natural := get_id_motor(i);
        constant SIMU_REF_COUNT : natural := 8192;
        signal r_ref_cnt : integer range -(SIMU_REF_COUNT+1) to SIMU_REF_COUNT+1;
        signal w_qei_z_local : std_logic;
        signal r_qei_z_simu : std_logic;
    begin



        w_regs_data_in_value(((i+1)+REG_QEI_OFFSET)*32-1 downto (i+REG_QEI_OFFSET)*32) <= "0000000" & w_qei_override(i) & "0000000" & w_qei_ref(i) & w_qei_value(i);

        w_regs_data_in_value_mask(((i+1)+REG_QEI_OFFSET)*4-1 downto (i+REG_QEI_OFFSET)*4) <= (others=>'1');

        w_reg <= regs_data_out_value(((i+1)+REG_QEI_OFFSET)*32-1 downto (i+REG_QEI_OFFSET)*32);

        w_qei_override(i) <= w_reg(24);


        inst_qei: QuadratureCounterPorts 
        port map (
            clock     => clk,
            QuadA     => w_qei_a(i),                             
            QuadB     => w_qei_b(i),                               
            CounterValue => w_cnt
        );	 	
        
        p_async: process(w_sim_mode,r_qei_value_simu,w_cnt,w_qei_override,w_reg) is
        begin
            if w_sim_mode = '0' then
                if w_qei_override(i) = '0' then
                    w_qei_value(i) <= w_cnt;
                else
                    w_qei_value(i) <= w_reg(16-1 downto 0);
                end if;
            else
                if w_qei_override(i) = '0' then
                    w_qei_value(i) <= r_qei_value_simu;
                else
                    w_qei_value(i) <= w_reg(16-1 downto 0);
                end if;
            end if;
        end process;


        p_sync_simu: process(clk,reset) is
        begin
            if reset = '1' then
                r_qei_value_simu <= (others=>'0');
                r_ref_cnt <= 0;
            elsif rising_edge(clk) then
                if unsigned(abs(signed(w_motor_value(SIMU_ID_MOTOR)))) >= 10000 then
                    if w_motor_dir(SIMU_ID_MOTOR) = '0' then
                        r_qei_value_simu <= std_logic_vector(unsigned(r_qei_value_simu)+1);
                        r_ref_cnt <= r_ref_cnt+1;
                    else
                        r_qei_value_simu <= std_logic_vector(unsigned(r_qei_value_simu)-1);
                        r_ref_cnt <= r_ref_cnt-1;
                    end if;
                end if;
                r_qei_z_simu <= '0';
                if r_ref_cnt = SIMU_REF_COUNT or r_ref_cnt = -SIMU_REF_COUNT then
                    r_ref_cnt <= 0;
                    r_qei_z_simu <= '1';
                end if;
            
                if w_sim_mode = '0' then
                    r_qei_value_simu <= (others=>'0');
                    r_ref_cnt <= 0;
                end if;
                
            end if;
        end process;

        w_qei_z_local <= w_qei_z(i) when w_sim_mode = '0' else r_qei_z_simu;
    
        p_sync: process(clk,reset) is
        begin
            if reset = '1' then
                r_ref <= '0';
                r_qei_z <= '0';
            elsif rising_edge(clk) then
                r_qei_z  <= w_qei_z_local;
                r2_qei_z <= r_qei_z;

                r_ref <= '0';
                if r_qei_z = '1' and r2_qei_z = '0' then
                    r_ref <= '1';
                end if; 
                --if r_qei_z = '0' and r2_qei_z = '1' then
                --    r_ref <= '0';
                --end if; 
            end if;
        end process;

        w_qei_ref(i) <= r_ref when w_qei_override(i) = '0' else w_reg(16);

    end generate;

    
    --------------------------------------------------------
    --! PHYSICAL UART CABLING

    --! I2C0SDA(RX)+I2C0RESET(TX) = NEXTION + <==> SW_UART
    --! I2C1SCL(RX) = PROX SENSOR 1 ARM LEFT  <==> SW_UART
    --! I2C1SDA(RX)+I2C1RESET(TX)+I2C1SCL(PWM) = RPLIDARA2_1 + PWM <==> LAYER2


    --! UART0 = BLUETOOTH                  <==> SW_UART
    --! UART1 RX = SMART SERVO BUS         <==> ORCA LOW LEVEL
    --! UART1 TX (but real RX) = PROX SENSOR 2 ARM RIGHT <==> SW_UART
    --! UART2 = RPLIDARA2_2+PWM            <==> LAYER2
    --! UART3 = RPLIDARA2_3+PWM            <==> LAYER2

         
    --------------------------------------------------------


    --------------------------------------------------------
    ------------ UART 0 <==> SW_UART_L1_ID_SCREEN -------------
    uart0_tx     <= sw_uart_tx(SW_UART_L1_ID_SCREEN);--uart_tx(3);
    sw_uart_rx(SW_UART_L1_ID_SCREEN)   <= uart0_rx;
    

    --------------------------------------------------------
    --------------------------------------------------------
    
    --------------------------------------------------------
    ------------ UART 1 <==> ORCA LOW LEVEL -------------
	 --uart1_rx <= w_ll_uart_txd when r_ll_uart_rs485 = '1' and w_ll_uart_transmitting = '1'
     --       else 'Z';
					  		  
	 --w_ll_uart_rxd <= uart1_rx when r_ll_uart_rs485 = '0' or w_ll_uart_transmitting = '0' 
     --           else '1';
               	    

    sw_uart_rx(SW_UART_L1_ID_PROXIMITY_2) <= uart1_tx;
    sw_uart_rx(SW_UART_L1_ID_BLUETOOTH) <= uart1_rx;
 


    --! WARNING: HERE TX is an input
    --sw_uart_rx(SW_UART_L1_ID_PROXIMITY_2)<= uart1_tx;

    --------------------------------------------------------        
    --------------------------------------------------------
    
    --------------------------------------------------------
    ---------------- UART 2 <==> LAYER2 --------------------
    uart2_tx     <= uart_tx(2);
    uart_rx(2)   <= uart2_rx;
    
    --! PWM for RPLIDAR A2 motocontrol PIN (TOP LIDAR)
    uart2_custom <= w_pwm_custom_out(2);
    --------------------------------------------------------
    --------------------------------------------------------

    --------------------------------------------------------
    ---------------- UART 3 <==> LAYER2 --------------------

	uart3_rx <= w_ll_uart_txd when r_ll_uart_rs485 = '1' and w_ll_uart_transmitting = '1'
           else 'Z';
					  		  
	w_ll_uart_rxd <= uart3_rx when r_ll_uart_rs485 = '0' or w_ll_uart_transmitting = '0' 
           else '1';
               	    

    sw_uart_rx(SW_UART_L1_ID_PROXIMITY_1)<= uart3_tx;


    uart3_custom <= sw_uart_tx(SW_UART_L1_ID_BLUETOOTH);
    --------------------------------------------------------
    --------------------------------------------------------
    
    --------------------------------------------------------
    ------------ SPI 0 <==> LAYER2 (UART0) -----------------

    spi0_ss      <= uart_tx(0);
    uart_rx(0)   <= spi0_mosi;
    spi0_sclk     <= w_pwm_custom_out(0);

    --------------------------------------------------------
    --------------------------------------------------------
    
    --------------------------------------------------------
    -------------- SPI 1 <==> LAYER2 (UART1) ---------------

    spi1_ss      <= uart_tx(1);
    uart_rx(1)   <= spi1_mosi;
    spi1_sclk     <= w_pwm_custom_out(1);

    --------------------------------------------------------
    --------------------------------------------------------
    


    --i2c0_scl <= 'Z' when w_i2c_0_scl_oe = '0' else '0';
    --i2c0_sda <= 'Z' when w_i2c_0_sda_oe = '0' else '0';
    --i2c0_reset <= '0';

    --i2c1_scl <= 'Z' when w_i2c_1_scl_oe = '0' else '0';
    --i2c1_sda <= 'Z' when w_i2c_1_sda_oe = '0' else '0';




    b_orca_low_level: block
    --uint8_t reset;
    --uint8_t pgm_id;
    --uint8_t arg[2];
    --// ADC 1278 Muxed
    --uint8_t  adc_drdy;  // in // 1
    --uint8_t  adc_reset; // out
    --uint8_t  adc_valid; // 
    --uint8_t  adc_id;
    --uint32_t adc_value; // 2
    --// Color Sensor
    --uint8_t  color_valid; // 3
    --uint8_t  reserved2[3];
    --uint16_t R; //4
    --uint16_t G;
    --uint16_t B; //5
    --uint16_t C;
    --// Distance sensors
    --uint8_t  dist_valid; // 6
    --uint8_t  dist_id;
    --uint8_t  reserved3[2];
    --uint32_t dist_value; // 7   

        signal w_pio_data_in_value   :  std_logic_vector(511 downto 0) := (others=>'0');
        signal w_pio_data_out_value  :  std_logic_vector(511 downto 0);

        constant REGS_ADC_SERVO_OFFSET : natural := 38;

        constant REGS_COLOR_VALID_OFFSET : natural := 50;
        constant REGS_COLOR_RG_OFFSET    : natural := 51;
        constant REGS_COLOR_BC_OFFSET    : natural := 52;
        constant REGS_DISTANCE_OFFSET    : natural := 53;

        constant REGS_CMD_IN_OFFSET       : natural := 61;
        constant REGS_CMD_PARAMS_OFFSET   : natural := 62;
		  constant REGS_CMD_OUT_OFFSET      : natural := 63;
		  
		  
		  
        constant ORCA_REGS_ADC_CFG_OFFSET    : natural := 1;
        constant ORCA_REGS_ADC_VALUE_OFFSET  : natural := 2;
        constant ORCA_REGS_COLOR_CFG_OFFSET  : natural := 3;
        constant ORCA_REGS_COLOR_RG_OFFSET   : natural := 4;
        constant ORCA_REGS_COLOR_BC_OFFSET   : natural := 5;
        constant ORCA_REGS_DIST_CFG_OFFSET   : natural := 6;
        constant ORCA_REGS_DIST_VALUE_OFFSET : natural := 7;
        constant ORCA_REGS_CMD_IN_OFFSET 	   : natural := 8;
        constant ORCA_REGS_CMD_PARAMS_OFFSET : natural := 9;
        constant ORCA_REGS_CMD_OUT_OFFSET 	: natural := 10;

		  
		  
        signal w_adc_muxed_id       : std_logic_vector(8-1 downto 0);
        signal w_adc_muxed_valid    : std_logic;
        signal w_adc_muxed_data     : std_logic_vector(32-1 downto 0);
        signal r_adc_muxed_data     : int32_t(8-1 downto 0);

        signal w_dist_id       : std_logic_vector(8-1 downto 0);
        signal w_dist_valid    : std_logic;
        signal w_dist_data     : std_logic_vector(32-1 downto 0);

        signal r_distance : int32_t(8-1 downto 0);


    begin
        --w_regs_data_in_value_mask((1+8+REGS_ADC_SERVO_OFFSET)*4-1    downto (0+REGS_ADC_SERVO_OFFSET)*4)   <= (others=>'1');

        w_regs_data_in_value_mask((1+REGS_COLOR_VALID_OFFSET)*4-1   downto (0+REGS_COLOR_VALID_OFFSET)*4)   <= "1111";
        w_regs_data_in_value_mask((1+REGS_COLOR_RG_OFFSET)*4-1      downto (0+REGS_COLOR_RG_OFFSET)*4)      <= "1111";
        w_regs_data_in_value_mask((1+REGS_COLOR_BC_OFFSET)*4-1      downto (0+REGS_COLOR_BC_OFFSET)*4)      <= "1111";
        w_regs_data_in_value_mask((1+8+REGS_DISTANCE_OFFSET)*4-1    downto (0+REGS_DISTANCE_OFFSET)*4)   <= (others=>'1');

        w_regs_data_in_value_mask((1+REGS_CMD_IN_OFFSET)*4-1      downto (0+REGS_CMD_IN_OFFSET)*4)      <= "1111";
        w_regs_data_in_value_mask((1+REGS_CMD_PARAMS_OFFSET)*4-1  downto (0+REGS_CMD_PARAMS_OFFSET)*4)  <= "1111";
        w_regs_data_in_value_mask((1+REGS_CMD_OUT_OFFSET)*4-1     downto (0+REGS_CMD_OUT_OFFSET)*4)     <= "1111";

		  
        p_async: process(regs_data_out_value,w_pio_data_out_value,r_distance) is
        begin
            w_pio_data_in_value(1*32-1 downto 0*32)      <= X"00000000";


            w_pio_data_in_value((1+ORCA_REGS_ADC_CFG_OFFSET)*32-1 downto (0+ORCA_REGS_ADC_CFG_OFFSET)*32) <= X"0000000" & "000" & regs_data_out_value(8);
            w_pio_data_in_value((1+ORCA_REGS_ADC_VALUE_OFFSET)*32-1 downto (0+ORCA_REGS_ADC_VALUE_OFFSET)*32) <= X"0000000" & "000" & regs_data_out_value(8);
            --w_pio_data_in_value((1+ORCA_REGS_ADC_CFG_OFFSET)*32-1 downto (0+ORCA_REGS_ADC_CFG_OFFSET)*32)(0) <= regs_data_out_value(8);


            
            for i in 0 to r_distance'length-1 loop
                w_regs_data_in_value((1+i+REGS_DISTANCE_OFFSET)*32-1    downto (0+i+REGS_DISTANCE_OFFSET)*32) <= r_distance(i);
            end loop;

            w_pio_data_in_value((ORCA_REGS_CMD_OUT_OFFSET+1)*32-1 downto (ORCA_REGS_CMD_IN_OFFSET)*32) <= regs_data_out_value((REGS_CMD_OUT_OFFSET+1)*32-1 downto REGS_CMD_IN_OFFSET*32);


        end process;

        w_regs_data_in_value((1+REGS_COLOR_VALID_OFFSET)*32-1 downto (0+REGS_COLOR_VALID_OFFSET)*32) <= X"0000000" & "000" & std_norm_range(w_pio_data_out_value((1+ORCA_REGS_COLOR_CFG_OFFSET)*32-1   downto (0+ORCA_REGS_COLOR_CFG_OFFSET)*32))(0);

        w_regs_data_in_value((1+REGS_COLOR_RG_OFFSET)*32-1 downto (0+REGS_COLOR_RG_OFFSET)*32) <= w_pio_data_out_value((1+ORCA_REGS_COLOR_RG_OFFSET)*32-1   downto (0+ORCA_REGS_COLOR_RG_OFFSET)*32);
        w_regs_data_in_value((1+REGS_COLOR_BC_OFFSET)*32-1 downto (0+REGS_COLOR_BC_OFFSET)*32) <= w_pio_data_out_value((1+ORCA_REGS_COLOR_BC_OFFSET)*32-1   downto (0+ORCA_REGS_COLOR_BC_OFFSET)*32);

        w_regs_data_in_value((1+REGS_CMD_IN_OFFSET)*32-1 downto (0+REGS_CMD_IN_OFFSET)*32)         <= w_pio_data_out_value((1+ORCA_REGS_CMD_IN_OFFSET)*32-1   downto (0+ORCA_REGS_CMD_IN_OFFSET)*32);
        w_regs_data_in_value((1+REGS_CMD_PARAMS_OFFSET)*32-1 downto (0+REGS_CMD_PARAMS_OFFSET)*32) <= w_pio_data_out_value((1+ORCA_REGS_CMD_PARAMS_OFFSET)*32-1   downto (0+ORCA_REGS_CMD_PARAMS_OFFSET)*32);
        w_regs_data_in_value((1+REGS_CMD_OUT_OFFSET)*32-1 downto (0+REGS_CMD_OUT_OFFSET)*32)       <= w_pio_data_out_value((1+ORCA_REGS_CMD_OUT_OFFSET)*32-1   downto (0+ORCA_REGS_CMD_OUT_OFFSET)*32);

		  
        --i2c1_reset <= std_norm_range(w_pio_data_out_value((1+ORCA_REGS_ADC_CFG_OFFSET)*32-1 downto (0+ORCA_REGS_ADC_CFG_OFFSET)*32))(8);

        w_adc_muxed_id      <= std_norm_range(w_pio_data_out_value((1+ORCA_REGS_ADC_CFG_OFFSET)*32-1   downto (0+ORCA_REGS_ADC_CFG_OFFSET)*32))(32-1 downto 24);
        w_adc_muxed_valid   <= std_norm_range(w_pio_data_out_value((1+ORCA_REGS_ADC_CFG_OFFSET)*32-1   downto (0+ORCA_REGS_ADC_CFG_OFFSET)*32))(16);
        w_adc_muxed_data    <= w_pio_data_out_value((1+ORCA_REGS_ADC_VALUE_OFFSET)*32-1 downto (0+ORCA_REGS_ADC_VALUE_OFFSET)*32);
 
        w_dist_id      <= std_norm_range(w_pio_data_out_value((1+ORCA_REGS_DIST_CFG_OFFSET)*32-1   downto (0+ORCA_REGS_DIST_CFG_OFFSET)*32))(16-1 downto 8);
        w_dist_valid   <= std_norm_range(w_pio_data_out_value((1+ORCA_REGS_DIST_CFG_OFFSET)*32-1   downto (0+ORCA_REGS_DIST_CFG_OFFSET)*32))(0);
        w_dist_data    <= w_pio_data_out_value((1+ORCA_REGS_DIST_VALUE_OFFSET)*32-1 downto (0+ORCA_REGS_DIST_VALUE_OFFSET)*32);


        p_sync: process(clk,reset) is
        begin
            if reset = '1' then
                r_adc_muxed_data    <= (others=>(others=>'0'));
                r_distance          <= (others=>(others=>'0'));
            elsif rising_edge(clk) then
                if w_adc_muxed_valid = '1' then
                    for i in 0 to r_adc_muxed_data'length-1 loop
                        if i = unsigned(w_adc_muxed_id) then
                            r_adc_muxed_data(i) <= w_adc_muxed_data;
                        end if;
                    end loop;
                end if;

                if w_dist_valid = '1' then
                    for i in 0 to r_distance'length-1 loop
                        if i = unsigned(w_dist_id) then
                            r_distance(i) <= w_dist_data;
                        end if;
                    end loop;
                end if;
            end if;
        end process;


        -- loopback for now
        sw_uart_rx(SW_UART_L1_ID_LOW_LEVEL) <= sw_uart_tx(SW_UART_L1_ID_LOW_LEVEL);
        
        
	    inst_orca_low_level : component system_ll
	    port map (
		    clk_clk                    => clk,                    --                 clk.clk
		    --i2c_master_serial_0_sda_in => i2c0_sda, -- i2c_master_serial_0.sda_in
		    --i2c_master_serial_0_scl_in => i2c0_scl, --                    .scl_in
		    --i2c_master_serial_0_sda_oe => w_i2c_0_sda_oe, --                    .sda_oe
		    --i2c_master_serial_0_scl_oe => w_i2c_0_scl_oe, --                    .scl_oe
		    --i2c_master_serial_1_sda_in => i2c1_sda, -- i2c_master_serial_1.sda_in
		    --i2c_master_serial_1_scl_in => i2c1_scl, --                    .scl_in
		    --i2c_master_serial_1_sda_oe => w_i2c_1_sda_oe, --                    .sda_oe
		    --i2c_master_serial_1_scl_oe => w_i2c_1_scl_oe, --                    .scl_oe
		    pio_data_in_value          => w_pio_data_in_value,          --                 pio.data_in_value
		    pio_data_in_read           => open,           --                    .data_in_read
		    pio_data_out_value         => w_pio_data_out_value,         --                    .data_out_value
		    pio_data_out_write         => open,         --                    .data_out_write
		    reset_reset_n              => not reset,              --               reset.reset_n
		    uart_0_external_rxd        => w_ll_uart_rxd,             
		    uart_0_external_txd        => w_ll_uart_txd,     
		    uart_0_external_transmitting=> w_ll_uart_transmitting

		    --uart_0_external_rxd        => CONNECTED_TO_uart_0_external_rxd,        --     uart_0_external.rxd
		    --uart_0_external_txd        => CONNECTED_TO_uart_0_external_txd,        --                    .txd
	    );
    end block;
	 	 
end architecture;

