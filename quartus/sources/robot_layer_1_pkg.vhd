library IEEE;
use     IEEE.STD_LOGIC_1164.ALL;
use     IEEE.NUMERIC_STD.ALL;

library work;
use     work.types_pkg.all;

package robot_layer_1_pkg is

    constant MOTOR_COUNT : natural := 6;
    constant QEI_COUNT   : natural := 4+1+1;
    constant SW_UART_L1_COUNT : natural := 6; 
    constant SW_UART_L1_ID_SCREEN      : natural := 0; 
    constant SW_UART_L1_ID_LOW_LEVEL   : natural := 1; 
    constant SW_UART_L1_ID_PROXIMITY_1 : natural := 2; 
    constant SW_UART_L1_ID_PROXIMITY_2 : natural := 3; 
    constant SW_UART_L1_ID_BLUETOOTH   : natural := 4; 
    constant SW_UART_L1_ID_IMU         : natural := 5; 
	 
    component robot_layer_1 is
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
        spi1_mosi    : out std_logic;
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
    end component;


end package;
