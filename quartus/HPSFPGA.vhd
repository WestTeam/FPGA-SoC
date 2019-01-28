--=================================
-- Hello world
-- Author: J.Savonet
--=================================

--`define ENABLE_HPS
----`define ENABLE_CLK

library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;

library work;
use     work.types_pkg.all;
use     work.robot_layer_1_pkg.all;
use     work.robot_layer_2_pkg.all;
use     work.robot_layer_3_pkg.all;

entity hpsfpga is
    port (
	----------/ ADC --------/
	ADC_CONVST  : out std_logic;
	ADC_SCK     : out std_logic;
	ADC_SDI     : out std_logic;
	ADC_SDO     : in  std_logic;

	----------/ ARDUINO --------/
	--ARDUINO_IO      : inout std_logic_vector(16-1 downto 0);
	--ARDUINO_RESET_N : inout std_logic;

--`ifdef ENABLE_CLK
--	--------/ CLK --------/
--	out :             CLK_I2C_SCL,
--	inout :              CLK_I2C_SDA,
--`endif /*ENABLE_CLK*/

	----------/ FPGA --------/
	FPGA_CLK1_50 : in std_logic;
	FPGA_CLK2_50 : in std_logic;
	FPGA_CLK3_50 : in std_logic;

	----------/ GPIO --------/
	--GPIO_0  : inout   std_logic_vector(36-1 downto 0);
	--GPIO_1  : inout   std_logic_vector(36-1 downto 0);


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
    uart1_tx     : out std_logic;

    uart2_rx     : in  std_logic;
    uart2_tx     : out std_logic;
    uart2_custom : out std_logic;

    uart3_rx     : in  std_logic;
    uart3_tx     : out std_logic;
    uart3_custom : out std_logic;

    --------- I2C ----------
    i2c0_scl     : inout std_logic;
    i2c0_sda     : inout std_logic;
    i2c0_reset   : out   std_logic;

    i2c1_scl     : inout std_logic;
    i2c1_sda     : inout std_logic;
    i2c1_reset   : out   std_logic;

    --------- SPI ----------
    spi0_sclk    : in  std_logic;
    spi0_mosi    : in  std_logic;
    spi0_miso    : in  std_logic;
    spi0_ss      : in  std_logic;

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

--`ifdef ENABLE_HPS
	----------/ HPS --------/
	HPS_CONV_USB_N      : inout std_logic;
	HPS_DDR3_ADDR       : out   std_logic_vector(15-1 downto 0); 
	HPS_DDR3_BA         : out   std_logic_vector(3-1 downto 0);
	HPS_DDR3_CAS_N      : out   std_logic;
	HPS_DDR3_CKE        : out   std_logic;
	HPS_DDR3_CK_N       : out   std_logic;
	HPS_DDR3_CK_P       : out   std_logic;
	HPS_DDR3_CS_N       : out   std_logic;
	HPS_DDR3_DM         : out   std_logic_vector(4-1 downto 0);
	HPS_DDR3_DQ         : inout std_logic_vector(32-1 downto 0);
	HPS_DDR3_DQS_N      : inout std_logic_vector(4-1 downto 0); 
	HPS_DDR3_DQS_P      : inout std_logic_vector(4-1 downto 0);
	HPS_DDR3_ODT        : out   std_logic;
	HPS_DDR3_RAS_N      : out   std_logic;
	HPS_DDR3_RESET_N    : out   std_logic;
	HPS_DDR3_RZQ        : in    std_logic;
	HPS_DDR3_WE_N       : out   std_logic;
	HPS_ENET_GTX_CLK    : out   std_logic;
	HPS_ENET_INT_N      : inout std_logic;
	HPS_ENET_MDC        : out   std_logic;
	HPS_ENET_MDIO       : inout std_logic;
	HPS_ENET_RX_CLK     : in    std_logic;
	HPS_ENET_RX_DATA    : in    std_logic_vector(4-1 downto 0);
	HPS_ENET_RX_DV      : in    std_logic;
	HPS_ENET_TX_DATA    : out   std_logic_vector(4-1 downto 0);
	HPS_ENET_TX_EN      : out   std_logic;
	HPS_GSENSOR_INT     : inout std_logic;
	HPS_I2C0_SCLK       : inout std_logic;
	HPS_I2C0_SDAT       : inout std_logic;
	HPS_I2C1_SCLK       : inout std_logic;
	HPS_I2C1_SDAT       : inout std_logic;
	HPS_KEY             : inout std_logic;
	HPS_LED             : inout std_logic;
	HPS_LTC_GPIO        : inout std_logic;
	HPS_SD_CLK          : out   std_logic;
	HPS_SD_CMD          : inout std_logic;
	HPS_SD_DATA         : inout std_logic_vector(4-1 downto 0);
	HPS_SPIM_CLK        : out   std_logic;
	HPS_SPIM_MISO       : in    std_logic;
	HPS_SPIM_MOSI       : out   std_logic;
	HPS_SPIM_SS         : inout std_logic;
	HPS_UART_RX         : in    std_logic;
	HPS_UART_TX         : out   std_logic;
	HPS_USB_CLKOUT      : in    std_logic;
	HPS_USB_DATA        : inout std_logic_vector(8-1 downto 0);
	HPS_USB_DIR         : in    std_logic;
	HPS_USB_NXT         : in    std_logic;
	HPS_USB_STP         : out   std_logic;
--`endif /*ENABLE_HPS*/

	----------/ KEY --------/
	KEY                 : in    std_logic_vector(2-1 downto 0);

	----------/ LED --------/
	LED                 : out   std_logic_vector(8-1 downto 0);

	----------/ SW --------/
	SW                  : in    std_logic_vector(4-1 downto 0)
);
end entity;

architecture hpsfpga_arch of hpsfpga is



	component hps_fpga is
		port (
            clk_clk                               : in    std_logic                       := 'X';             -- clk
            hps_arm_h2f_reset_reset_n               : out   std_logic;                                          -- reset_n
            hps_arm_hps_io_hps_io_emac1_inst_TX_CLK : out   std_logic;                                          -- hps_io_emac1_inst_TX_CLK
            hps_arm_hps_io_hps_io_emac1_inst_TXD0   : out   std_logic;                                          -- hps_io_emac1_inst_TXD0
            hps_arm_hps_io_hps_io_emac1_inst_TXD1   : out   std_logic;                                          -- hps_io_emac1_inst_TXD1
            hps_arm_hps_io_hps_io_emac1_inst_TXD2   : out   std_logic;                                          -- hps_io_emac1_inst_TXD2
            hps_arm_hps_io_hps_io_emac1_inst_TXD3   : out   std_logic;                                          -- hps_io_emac1_inst_TXD3
            hps_arm_hps_io_hps_io_emac1_inst_RXD0   : in    std_logic                       := 'X';             -- hps_io_emac1_inst_RXD0
            hps_arm_hps_io_hps_io_emac1_inst_MDIO   : inout std_logic                       := 'X';             -- hps_io_emac1_inst_MDIO
            hps_arm_hps_io_hps_io_emac1_inst_MDC    : out   std_logic;                                          -- hps_io_emac1_inst_MDC
            hps_arm_hps_io_hps_io_emac1_inst_RX_CTL : in    std_logic                       := 'X';             -- hps_io_emac1_inst_RX_CTL
            hps_arm_hps_io_hps_io_emac1_inst_TX_CTL : out   std_logic;                                          -- hps_io_emac1_inst_TX_CTL
            hps_arm_hps_io_hps_io_emac1_inst_RX_CLK : in    std_logic                       := 'X';             -- hps_io_emac1_inst_RX_CLK
            hps_arm_hps_io_hps_io_emac1_inst_RXD1   : in    std_logic                       := 'X';             -- hps_io_emac1_inst_RXD1
            hps_arm_hps_io_hps_io_emac1_inst_RXD2   : in    std_logic                       := 'X';             -- hps_io_emac1_inst_RXD2
            hps_arm_hps_io_hps_io_emac1_inst_RXD3   : in    std_logic                       := 'X';             -- hps_io_emac1_inst_RXD3
            hps_arm_hps_io_hps_io_sdio_inst_CMD     : inout std_logic                       := 'X';             -- hps_io_sdio_inst_CMD
            hps_arm_hps_io_hps_io_sdio_inst_D0      : inout std_logic                       := 'X';             -- hps_io_sdio_inst_D0
            hps_arm_hps_io_hps_io_sdio_inst_D1      : inout std_logic                       := 'X';             -- hps_io_sdio_inst_D1
            hps_arm_hps_io_hps_io_sdio_inst_CLK     : out   std_logic;                                          -- hps_io_sdio_inst_CLK
            hps_arm_hps_io_hps_io_sdio_inst_D2      : inout std_logic                       := 'X';             -- hps_io_sdio_inst_D2
            hps_arm_hps_io_hps_io_sdio_inst_D3      : inout std_logic                       := 'X';             -- hps_io_sdio_inst_D3
            hps_arm_hps_io_hps_io_usb1_inst_D0      : inout std_logic                       := 'X';             -- hps_io_usb1_inst_D0
            hps_arm_hps_io_hps_io_usb1_inst_D1      : inout std_logic                       := 'X';             -- hps_io_usb1_inst_D1
            hps_arm_hps_io_hps_io_usb1_inst_D2      : inout std_logic                       := 'X';             -- hps_io_usb1_inst_D2
            hps_arm_hps_io_hps_io_usb1_inst_D3      : inout std_logic                       := 'X';             -- hps_io_usb1_inst_D3
            hps_arm_hps_io_hps_io_usb1_inst_D4      : inout std_logic                       := 'X';             -- hps_io_usb1_inst_D4
            hps_arm_hps_io_hps_io_usb1_inst_D5      : inout std_logic                       := 'X';             -- hps_io_usb1_inst_D5
            hps_arm_hps_io_hps_io_usb1_inst_D6      : inout std_logic                       := 'X';             -- hps_io_usb1_inst_D6
            hps_arm_hps_io_hps_io_usb1_inst_D7      : inout std_logic                       := 'X';             -- hps_io_usb1_inst_D7
            hps_arm_hps_io_hps_io_usb1_inst_CLK     : in    std_logic                       := 'X';             -- hps_io_usb1_inst_CLK
            hps_arm_hps_io_hps_io_usb1_inst_STP     : out   std_logic;                                          -- hps_io_usb1_inst_STP
            hps_arm_hps_io_hps_io_usb1_inst_DIR     : in    std_logic                       := 'X';             -- hps_io_usb1_inst_DIR
            hps_arm_hps_io_hps_io_usb1_inst_NXT     : in    std_logic                       := 'X';             -- hps_io_usb1_inst_NXT
            hps_arm_hps_io_hps_io_spim1_inst_CLK    : out   std_logic;                                          -- hps_io_spim1_inst_CLK
            hps_arm_hps_io_hps_io_spim1_inst_MOSI   : out   std_logic;                                          -- hps_io_spim1_inst_MOSI
            hps_arm_hps_io_hps_io_spim1_inst_MISO   : in    std_logic                       := 'X';             -- hps_io_spim1_inst_MISO
            hps_arm_hps_io_hps_io_spim1_inst_SS0    : out   std_logic;                                          -- hps_io_spim1_inst_SS0
            hps_arm_hps_io_hps_io_uart0_inst_RX     : in    std_logic                       := 'X';             -- hps_io_uart0_inst_RX
            hps_arm_hps_io_hps_io_uart0_inst_TX     : out   std_logic;                                          -- hps_io_uart0_inst_TX
            hps_arm_hps_io_hps_io_i2c0_inst_SDA     : inout std_logic                       := 'X';             -- hps_io_i2c0_inst_SDA
            hps_arm_hps_io_hps_io_i2c0_inst_SCL     : inout std_logic                       := 'X';             -- hps_io_i2c0_inst_SCL
            hps_arm_hps_io_hps_io_i2c1_inst_SDA     : inout std_logic                       := 'X';             -- hps_io_i2c1_inst_SDA
            hps_arm_hps_io_hps_io_i2c1_inst_SCL     : inout std_logic                       := 'X';             -- hps_io_i2c1_inst_SCL
            memory_mem_a                          : out   std_logic_vector(14 downto 0);                      -- mem_a
            memory_mem_ba                         : out   std_logic_vector(2 downto 0);                       -- mem_ba
            memory_mem_ck                         : out   std_logic;                                          -- mem_ck
            memory_mem_ck_n                       : out   std_logic;                                          -- mem_ck_n
            memory_mem_cke                        : out   std_logic;                                          -- mem_cke
            memory_mem_cs_n                       : out   std_logic;                                          -- mem_cs_n
            memory_mem_ras_n                      : out   std_logic;                                          -- mem_ras_n
            memory_mem_cas_n                      : out   std_logic;                                          -- mem_cas_n
            memory_mem_we_n                       : out   std_logic;                                          -- mem_we_n
            memory_mem_reset_n                    : out   std_logic;                                          -- mem_reset_n
            memory_mem_dq                         : inout std_logic_vector(31 downto 0)   := (others => 'X'); -- mem_dq
            memory_mem_dqs                        : inout std_logic_vector(3 downto 0)    := (others => 'X'); -- mem_dqs
            memory_mem_dqs_n                      : inout std_logic_vector(3 downto 0)    := (others => 'X'); -- mem_dqs_n
            memory_mem_odt                        : out   std_logic;                                          -- mem_odt
            memory_mem_dm                         : out   std_logic_vector(3 downto 0);                       -- mem_dm
            memory_oct_rzqin                      : in    std_logic                       := 'X';             -- oct_rzqin
            reset_reset_n                         : in    std_logic                       := 'X';             -- reset_n
            pio_n_layer1_data_in_value            : in    std_logic_vector(2048-1 downto 0) := (others => 'X'); -- data_in_value
            pio_n_layer1_data_in_read             : out   std_logic_vector(64-1 downto 0);                     -- data_in_read
            pio_n_layer1_data_out_value           : out   std_logic_vector(2048-1 downto 0);                    -- data_out_value
            pio_n_layer1_data_out_write           : out   std_logic_vector(64-1 downto 0);                     -- data_out_write
            pio_n_layer2_data_in_value            : in    std_logic_vector(2048-1 downto 0) := (others => 'X'); -- data_in_value
            pio_n_layer2_data_in_read             : out   std_logic_vector(64-1 downto 0);                     -- data_in_read
            pio_n_layer2_data_out_value           : out   std_logic_vector(2048-1 downto 0);                    -- data_out_value
            pio_n_layer2_data_out_write           : out   std_logic_vector(64-1 downto 0);                     -- data_out_write
            pio_n_layer3_data_in_value            : in    std_logic_vector(2048-1 downto 0) := (others => 'X'); -- data_in_value
            pio_n_layer3_data_in_read             : out   std_logic_vector(64-1 downto 0);                     -- data_in_read
            pio_n_layer3_data_out_value           : out   std_logic_vector(2048-1 downto 0);                    -- data_out_value
            pio_n_layer3_data_out_write           : out   std_logic_vector(64-1 downto 0);                     -- data_out_write
			   uart_0_rxd                            : in    std_logic                       := 'X';             -- rxd
			   uart_0_txd                            : out   std_logic;                                           -- txd
			   uart_1_rxd                            : in    std_logic                       := 'X';             -- rxd
			   uart_1_txd                            : out   std_logic;                                           -- txd
			   uart_2_rxd                            : in    std_logic                       := 'X';             -- rxd
			   uart_2_txd                            : out   std_logic;                                           -- txd
			   uart_3_rxd                            : in    std_logic                       := 'X';             -- rxd
			   uart_3_txd                            : out   std_logic;                                           -- txd
			   uart_4_rxd                            : in    std_logic                       := 'X';             -- rxd
			   uart_4_txd                            : out   std_logic;                                           -- txd
			   uart_5_rxd                            : in    std_logic                       := 'X';             -- rxd
			   uart_5_txd                            : out   std_logic                                           -- txd
			   --uart_6_rxd                            : in    std_logic                       := 'X';             -- rxd
			   --uart_6_txd                            : out   std_logic;                                           -- txd
			   --uart_7_rxd                            : in    std_logic                       := 'X';             -- rxd
			   --uart_7_txd                            : out   std_logic                                           -- txd
			
        );
	end component hps_fpga;



----=======================================================
----  REG/WIRE declarations
----=======================================================
    signal hps_fpga_reset_n : std_logic;




    signal w_pio_n_layer1_data_in_value            : std_logic_vector(2048-1 downto 0) := (others => 'X'); -- data_in_value
    signal w_pio_n_layer1_data_in_read             : std_logic_vector(64-1 downto 0);                     -- data_in_read
    signal w_pio_n_layer1_data_out_value           : std_logic_vector(2048-1 downto 0);                    -- data_out_value
    signal w_pio_n_layer1_data_out_write           : std_logic_vector(64-1 downto 0);                     -- data_out_write
    signal w_pio_n_layer2_data_in_value            : std_logic_vector(2048-1 downto 0) := (others => 'X'); -- data_in_value
    signal w_pio_n_layer2_data_in_read             : std_logic_vector(64-1 downto 0);                     -- data_in_read
    signal w_pio_n_layer2_data_out_value           : std_logic_vector(2048-1 downto 0);                    -- data_out_value
    signal w_pio_n_layer2_data_out_write           : std_logic_vector(64-1 downto 0);                     -- data_out_write
    signal w_pio_n_layer3_data_in_value            : std_logic_vector(2048-1 downto 0) := (others => 'X'); -- data_in_value
    signal w_pio_n_layer3_data_in_read             : std_logic_vector(64-1 downto 0);                     -- data_in_read
    signal w_pio_n_layer3_data_out_value           : std_logic_vector(2048-1 downto 0);                    -- data_out_value
    signal w_pio_n_layer3_data_out_write           : std_logic_vector(64-1 downto 0);                      -- data_out_write


        --------- UART ----------
    signal w_uart_tx       : std_logic_vector(4-1 downto 0);
    signal w_uart_rx       : std_logic_vector(4-1 downto 0);


    signal w_motor_value    : int16_t(6-1 downto 0);
    signal w_motor_current  : int24_t(6-1 downto 0);
    signal w_motor_fault    : std_logic_vector(6-1 downto 0);
    signal w_qei_value      : int16_t(5-1 downto 0);
    signal w_qei_ref        : std_logic_vector(5-1 downto 0);
 
    signal w_sum_m_dist    : std_logic_vector(32-1 downto 0);
    signal w_sum_m_angle   : std_logic_vector(32-1 downto 0);

    signal w_sum_c_dist    : std_logic_vector(32-1 downto 0);
    signal w_sum_c_angle   : std_logic_vector(32-1 downto 0);


    signal w_pos_valid     : std_logic;
    signal w_pos_id        : std_logic_vector(8-1 downto 0);
    signal w_pos_teta      : std_logic_vector(16-1 downto 0);
    signal w_pos_x         : std_logic_vector(16-1 downto 0);
    signal w_pos_y         : std_logic_vector(16-1 downto 0);
    signal w_pos_sum_dist  : std_logic_vector(32-1 downto 0);
    signal w_pos_sum_angle : std_logic_vector(32-1 downto 0);

    signal w_dist_en       : std_logic;
    signal w_dist_acc      : std_logic_vector(32-1 downto 0);
    signal w_dist_speed    : std_logic_vector(32-1 downto 0);
    signal w_dist_target   : std_logic_vector(32-1 downto 0);

    signal w_angle_en      : std_logic;
    signal w_angle_acc     : std_logic_vector(32-1 downto 0);
    signal w_angle_speed   : std_logic_vector(32-1 downto 0);
    signal w_angle_target  : std_logic_vector(32-1 downto 0);


    signal r_reset : std_logic;
    --signal r_cnt : natural := 0;

	 signal w_uart_loop : std_logic;
begin

   
    p_sync: process(FPGA_CLK1_50)
    begin
        if rising_edge(FPGA_CLK1_50) then
            r_reset <= not hps_fpga_reset_n or w_pio_n_layer1_data_out_value(0);
        end if;
    
    end process;



    --led(8-1 downto 4) <= w_ledg_out(4-1 downto 0);
	buzzer <= '0';

    


    inst_layer_1: robot_layer_1
    generic map (
        CLK_FREQUENCY_HZ => 50_000_000,
        RegCnt => 64
    )
    port map (

        clk     => FPGA_CLK1_50,
        reset   => r_reset,

        regs_data_in_value      => w_pio_n_layer1_data_in_value,
        regs_data_in_read       => w_pio_n_layer1_data_in_read,              
        regs_data_out_value     => w_pio_n_layer1_data_out_value,
        regs_data_out_write     => w_pio_n_layer1_data_out_write,

        ----------- ADC (//) ---------
        ad0_sclk => ad0_sclk,
        ad0_miso => ad0_miso,
        ad0_drdy => ad0_drdy,
        ad0_sync => ad0_sync,
        ad0_clk  => ad0_clk ,

        --------- ADC (muxed) --------
        ad1_sclk => ad1_sclk,
        ad1_mosi => ad1_mosi,
        ad1_miso => ad1_miso,
        ad1_ss   => ad1_ss,
        ad1_drdy => ad1_drdy,
        ad1_rst  => ad1_rst,

        ---------- H BRIDGE ----------
        m0_pwma  => m0_pwma,
        m0_pwmb  => m0_pwmb,
        m01_fault=> m01_fault,
        
        m1_pwma  => m1_pwma,
        m1_pwmb  => m1_pwmb,
        m01_resetn=> m01_resetn,

        m2_pwma  => m2_pwma,
        m2_pwmb  => m2_pwmb,

        m3_pwma  => m3_pwma,
        m3_pwmb  => m3_pwmb,

        m2345_fault=> m2345_fault,

        m4_pwma  => m4_pwma,
        m4_pwmb  => m4_pwmb,

        m5_pwma  => m5_pwma,
        m5_pwmb  => m5_pwmb,

        m2345_resetn=> m2345_resetn,

        ---------- QEI ----------    
        qei0_a   => qei0_a,
        qei0_b   => qei0_b,

        qei1_a   => qei1_a,
        qei1_b   => qei1_b,

        qei2_a   => qei2_a,
        qei2_b   => qei2_b,
        qei2_z   => qei2_z,

        qei3_a   => qei3_a,
        qei3_b   => qei3_b,
        qei3_z   => qei3_z,

        ---------- ESC ----------    
        esc0_pwm => esc0_pwm,
        esc0_dir => esc0_dir,

        esc1_pwm => esc1_pwm,
        esc1_dir => esc1_dir,

        ------- PWM (Servos) ------
        s => s,

        --------- IOs ----------
        io_0 => io_0,
        io_1 => io_1,
        io_2 => io_2,
        io_3 => io_3,
        io_4 => io_4,
        io_5 => io_5,
        io_6 => io_6,
        io_7 => io_7,

        --------- UART ----------
        uart0_rx     => uart0_rx,
        uart0_tx     => uart0_tx,

        uart1_rx     => uart1_rx,
        uart1_tx     => uart1_tx,

        uart2_rx     => uart2_rx,
        uart2_tx     => uart2_tx,
        uart2_custom => uart2_custom,

        uart3_rx     => uart3_rx,
        uart3_tx     => uart3_tx,
        uart3_custom => uart3_custom,

        --------- I2C ----------
        i2c0_scl     => i2c0_scl,
        i2c0_sda     => i2c0_sda,
        i2c0_reset   => i2c0_reset,

        i2c1_scl     => i2c1_scl,
        i2c1_sda     => i2c1_sda,
        i2c1_reset   => i2c1_reset,

        --------- SPI ----------
        spi0_sclk    => spi0_sclk,
        spi0_mosi    => spi0_mosi,
        spi0_miso    => spi0_miso,
        spi0_ss      => spi0_ss,

        spi1_sclk    => spi1_sclk,
        spi1_mosi    => spi1_mosi,
        spi1_miso    => spi1_miso,
        spi1_ss      => spi1_ss,

        --! Use SPI1
        imu_ss       => imu_ss,
        imu_drdy     => imu_drdy,
        imu_fsync    => imu_fsync,

        ---------- LED -----------
        led_green => led_green,
        led_red   => led_red,

        --------- MGMT -----------
        lv_mux    => lv_mux,
        buzzer    => open,

	    ----------/ NANO SOC LED --------/
	    LED                 => LED,

	    ----------/ NANO SOC SW --------/
	    SW                  => SW,


        ---------------------------------
        -------- TO/FROM LAYER 2 --------
        ---------------------------------


        --------- UART ----------
        uart_tx       => w_uart_tx,
        uart_rx       => w_uart_rx,

        motor_value   => w_motor_value,
        motor_current => w_motor_current,
        motor_fault   => w_motor_fault,

        qei_value     => w_qei_value,
        qei_ref       => w_qei_ref

    );

    inst_layer_2: robot_layer_2
    generic map (
        CLK_FREQUENCY_HZ => 50_000_000,
        RegCnt => 64
    )
    port map (
        clk     => FPGA_CLK1_50,
        reset   => r_reset,

        regs_data_in_value      => w_pio_n_layer2_data_in_value,
        regs_data_in_read       => w_pio_n_layer2_data_in_read,              
        regs_data_out_value     => w_pio_n_layer2_data_out_value,
        regs_data_out_write     => w_pio_n_layer2_data_out_write,

        ---------------------------------
        -------- TO/FROM LAYER 1 --------
        ---------------------------------

        --------- UART ----------
        uart_tx       => w_uart_tx,
        uart_rx       => w_uart_rx,



        motor_value   => w_motor_value,
        motor_current => w_motor_current,
        motor_fault   => w_motor_fault,

        qei_value     => w_qei_value,
        qei_ref       => w_qei_ref,

        ---------------------------------
        -------- TO/FROM LAYER 3 --------
        ---------------------------------
        sum_m_dist    => w_sum_m_dist,
        sum_m_angle    => w_sum_m_angle,
        sum_c_dist    => w_sum_c_dist,
        sum_c_angle    => w_sum_c_angle,

        pos_valid    => w_pos_valid,
        pos_id    => w_pos_id,
        pos_teta    => w_pos_teta,
        pos_x    => w_pos_x,
        pos_y    => w_pos_y,
        pos_sum_dist    => w_pos_sum_dist,
        pos_sum_angle    => w_pos_sum_angle,

        dist_en    => w_dist_en,
        dist_acc    => w_dist_acc,
        dist_speed    => w_dist_speed,
        dist_target    => w_dist_target,

        angle_en    => w_angle_en,
        angle_acc    => w_angle_acc,
        angle_speed    => w_angle_speed,
        angle_target    => w_angle_target

    );


    inst_layer_3: robot_layer_3
    generic map (
        CLK_FREQUENCY_HZ => 50_000_000,
        RegCnt => 64
    )
    port map (
        clk     => FPGA_CLK1_50,
        reset   => r_reset,

        regs_data_in_value      => w_pio_n_layer3_data_in_value,
        regs_data_in_read       => w_pio_n_layer3_data_in_read,              
        regs_data_out_value     => w_pio_n_layer3_data_out_value,
        regs_data_out_write     => w_pio_n_layer3_data_out_write,

        ---------------------------------
        -------- TO/FROM LAYER 2 --------
        ---------------------------------

        sum_m_dist    => w_sum_m_dist,
        sum_m_angle    => w_sum_m_angle,
        sum_c_dist    => w_sum_c_dist,
        sum_c_angle    => w_sum_c_angle,

        pos_valid    => w_pos_valid,
        pos_id    => w_pos_id,
        pos_teta    => w_pos_teta,
        pos_x    => w_pos_x,
        pos_y    => w_pos_y,
        pos_sum_dist    => w_pos_sum_dist,
        pos_sum_angle    => w_pos_sum_angle,

        dist_en    => w_dist_en,
        dist_acc    => w_dist_acc,
        dist_speed    => w_dist_speed,
        dist_target    => w_dist_target,

        angle_en    => w_angle_en,
        angle_acc    => w_angle_acc,
        angle_speed    => w_angle_speed,
        angle_target    => w_angle_target

    );




----=======================================================
----  Structural coding
----=======================================================
    inst_hps: hps_fpga 
    port map(
	    -- CLK & RESET
	    clk_clk                               =>  FPGA_CLK1_50 ,                               --                            clkclk
	    reset_reset_n                         =>  '1' ,                         --                          resetreset_n

        hps_arm_h2f_reset_reset_n => hps_fpga_reset_n,
		 
        pio_n_layer1_data_in_value            => w_pio_n_layer1_data_in_value,            --    pio_n_layer1.data_in_value
        pio_n_layer1_data_in_read             => w_pio_n_layer1_data_in_read,             --                .data_in_read
        pio_n_layer1_data_out_value           => w_pio_n_layer1_data_out_value,           --                .data_out_value
        pio_n_layer1_data_out_write           => w_pio_n_layer1_data_out_write,           --                .data_out_write
        pio_n_layer2_data_in_value            => w_pio_n_layer2_data_in_value,            --    pio_n_layer2.data_in_value
        pio_n_layer2_data_in_read             => w_pio_n_layer2_data_in_read,             --                .data_in_read
        pio_n_layer2_data_out_value           => w_pio_n_layer2_data_out_value,           --                .data_out_value
        pio_n_layer2_data_out_write           => w_pio_n_layer2_data_out_write,           --                .data_out_write
        pio_n_layer3_data_in_value            => w_pio_n_layer3_data_in_value,            --    pio_n_layer3.data_in_value
        pio_n_layer3_data_in_read             => w_pio_n_layer3_data_in_read,             --                .data_in_read
        pio_n_layer3_data_out_value           => w_pio_n_layer3_data_out_value,           --                .data_out_value
        pio_n_layer3_data_out_write           => w_pio_n_layer3_data_out_write,            --                .data_out_write



	    -- HPS ETHERNET
	    hps_arm_hps_io_hps_io_emac1_inst_TX_CLK =>  HPS_ENET_GTX_CLK    ,   --                             hps_arm_hps_iohps_io_emac1_inst_TX_CLK
	    hps_arm_hps_io_hps_io_emac1_inst_TXD0   =>  HPS_ENET_TX_DATA(0) ,   --                             hps_io_emac1_inst_TXD0
	    hps_arm_hps_io_hps_io_emac1_inst_TXD1   =>  HPS_ENET_TX_DATA(1) ,   --                             hps_io_emac1_inst_TXD1
	    hps_arm_hps_io_hps_io_emac1_inst_TXD2   =>  HPS_ENET_TX_DATA(2),   --                             hps_io_emac1_inst_TXD2
	    hps_arm_hps_io_hps_io_emac1_inst_TXD3   =>  HPS_ENET_TX_DATA(3),   --                             hps_io_emac1_inst_TXD3
	    hps_arm_hps_io_hps_io_emac1_inst_RXD0   =>  HPS_ENET_RX_DATA(0),   --                             hps_io_emac1_inst_RXD0
	    hps_arm_hps_io_hps_io_emac1_inst_MDIO   =>  HPS_ENET_MDIO       ,   --                             hps_io_emac1_inst_MDIO
	    hps_arm_hps_io_hps_io_emac1_inst_MDC    =>  HPS_ENET_MDC        ,   --                             hps_io_emac1_inst_MDC
	    hps_arm_hps_io_hps_io_emac1_inst_RX_CTL =>  HPS_ENET_RX_DV      ,   --                             hps_io_emac1_inst_RX_CTL
	    hps_arm_hps_io_hps_io_emac1_inst_TX_CTL =>  HPS_ENET_TX_EN      ,   --                             hps_io_emac1_inst_TX_CTL
	    hps_arm_hps_io_hps_io_emac1_inst_RX_CLK =>  HPS_ENET_RX_CLK     ,   --                             hps_io_emac1_inst_RX_CLK
	    hps_arm_hps_io_hps_io_emac1_inst_RXD1   =>  HPS_ENET_RX_DATA(1),   --                             hps_io_emac1_inst_RXD1
	    hps_arm_hps_io_hps_io_emac1_inst_RXD2   =>  HPS_ENET_RX_DATA(2),   --                             hps_io_emac1_inst_RXD2
	    hps_arm_hps_io_hps_io_emac1_inst_RXD3   =>  HPS_ENET_RX_DATA(3),   --                             hps_io_emac1_inst_RXD3		  
	
	    -- HPS SD CARD
	    hps_arm_hps_io_hps_io_sdio_inst_CMD     =>  HPS_SD_CMD     ,        --                               hps_io_sdio_inst_CMD
	    hps_arm_hps_io_hps_io_sdio_inst_D0      =>  HPS_SD_DATA(0),        --                               hps_io_sdio_inst_D0
	    hps_arm_hps_io_hps_io_sdio_inst_D1      =>  HPS_SD_DATA(1),        --                               hps_io_sdio_inst_D1
	    hps_arm_hps_io_hps_io_sdio_inst_CLK     =>  HPS_SD_CLK     ,        --                               hps_io_sdio_inst_CLK
	    hps_arm_hps_io_hps_io_sdio_inst_D2      =>  HPS_SD_DATA(2),        --                               hps_io_sdio_inst_D2
	    hps_arm_hps_io_hps_io_sdio_inst_D3      =>  HPS_SD_DATA(3),        --                               hps_io_sdio_inst_D3
	
	    -- HPS USB
	    hps_arm_hps_io_hps_io_usb1_inst_D0      =>  HPS_USB_DATA(0),      --                               hps_io_usb1_inst_D0
	    hps_arm_hps_io_hps_io_usb1_inst_D1      =>  HPS_USB_DATA(1),      --                               hps_io_usb1_inst_D1
	    hps_arm_hps_io_hps_io_usb1_inst_D2      =>  HPS_USB_DATA(2),      --                               hps_io_usb1_inst_D2
	    hps_arm_hps_io_hps_io_usb1_inst_D3      =>  HPS_USB_DATA(3),      --                               hps_io_usb1_inst_D3
	    hps_arm_hps_io_hps_io_usb1_inst_D4      =>  HPS_USB_DATA(4),      --                               hps_io_usb1_inst_D4
	    hps_arm_hps_io_hps_io_usb1_inst_D5      =>  HPS_USB_DATA(5),      --                               hps_io_usb1_inst_D5
	    hps_arm_hps_io_hps_io_usb1_inst_D6      =>  HPS_USB_DATA(6),      --                               hps_io_usb1_inst_D6
	    hps_arm_hps_io_hps_io_usb1_inst_D7      =>  HPS_USB_DATA(7),      --                               hps_io_usb1_inst_D7
	    hps_arm_hps_io_hps_io_usb1_inst_CLK     =>  HPS_USB_CLKOUT  ,      --                               hps_io_usb1_inst_CLK
	    hps_arm_hps_io_hps_io_usb1_inst_STP     =>  HPS_USB_STP     ,      --                               hps_io_usb1_inst_STP
	    hps_arm_hps_io_hps_io_usb1_inst_DIR     =>  HPS_USB_DIR     ,      --                               hps_io_usb1_inst_DIR
	    hps_arm_hps_io_hps_io_usb1_inst_NXT     =>  HPS_USB_NXT     ,      --                               hps_io_usb1_inst_NXT
	      
	    -- HPS SPI
	    hps_arm_hps_io_hps_io_spim1_inst_CLK    =>  HPS_SPIM_CLK  ,        --                               hps_io_spim1_inst_CLK
	    hps_arm_hps_io_hps_io_spim1_inst_MOSI   =>  HPS_SPIM_MOSI ,        --                               hps_io_spim1_inst_MOSI
	    hps_arm_hps_io_hps_io_spim1_inst_MISO   =>  HPS_SPIM_MISO ,        --                               hps_io_spim1_inst_MISO
	    hps_arm_hps_io_hps_io_spim1_inst_SS0    =>  HPS_SPIM_SS   ,        --                               hps_io_spim1_inst_SS0
	
	    -- HPS UART
	    hps_arm_hps_io_hps_io_uart0_inst_RX     =>  HPS_UART_RX ,     --                               hps_io_uart0_inst_RX
	    hps_arm_hps_io_hps_io_uart0_inst_TX     =>  HPS_UART_TX ,     --                               hps_io_uart0_inst_TX
	
	    hps_arm_hps_io_hps_io_i2c0_inst_SDA     =>  HPS_I2C0_SDAT ,     --                               hps_io_i2c0_inst_SDA
	    hps_arm_hps_io_hps_io_i2c0_inst_SCL     =>  HPS_I2C0_SCLK ,     --                               hps_io_i2c0_inst_SCL
	
	    -- HPS I2C1
	    hps_arm_hps_io_hps_io_i2c1_inst_SDA     =>  HPS_I2C1_SDAT ,     --                               hps_io_i2c1_inst_SDA
	    hps_arm_hps_io_hps_io_i2c1_inst_SCL     =>  HPS_I2C1_SCLK ,     --                               hps_io_i2c1_inst_SCL
	
		 uart_0_rxd => w_uart_tx(0),
		 uart_0_txd => open,

		 uart_1_rxd => w_uart_tx(1),
		 uart_1_txd => open,
		 
		 uart_2_rxd => w_uart_tx(2),
		 uart_2_txd => open,

		 uart_3_rxd => w_uart_tx(3),
		 uart_3_txd => open,

		 uart_4_rxd => w_uart_tx(3),
		 uart_4_txd => open,

		 uart_5_rxd => w_uart_loop,
		 uart_5_txd => w_uart_loop,

		 --uart_6_rxd => '0',
		 --uart_6_txd => open,

		 --uart_7_rxd => '0',
		 --uart_7_txd => open,		 
		 
	
	    -- HPS DDR3
	    memory_mem_a                          =>  HPS_DDR3_ADDR ,                       --                memorymem_a
	    memory_mem_ba                         =>  HPS_DDR3_BA ,                         --                mem_ba
	    memory_mem_ck                         =>  HPS_DDR3_CK_P ,                       --                mem_ck
	    memory_mem_ck_n                       =>  HPS_DDR3_CK_N ,                       --                mem_ck_n
	    memory_mem_cke                        =>  HPS_DDR3_CKE ,                        --                mem_cke
	    memory_mem_cs_n                       =>  HPS_DDR3_CS_N ,                       --                mem_cs_n
	    memory_mem_ras_n                      =>  HPS_DDR3_RAS_N ,                      --                mem_ras_n
	    memory_mem_cas_n                      =>  HPS_DDR3_CAS_N ,                      --                mem_cas_n
	    memory_mem_we_n                       =>  HPS_DDR3_WE_N ,                       --                mem_we_n
	    memory_mem_reset_n                    =>  HPS_DDR3_RESET_N ,                    --                mem_reset_n
	    memory_mem_dq                         =>  HPS_DDR3_DQ ,                         --                mem_dq
	    memory_mem_dqs                        =>  HPS_DDR3_DQS_P ,                      --                mem_dqs
	    memory_mem_dqs_n                      =>  HPS_DDR3_DQS_N ,                      --                mem_dqs_n
	    memory_mem_odt                        =>  HPS_DDR3_ODT ,                        --                mem_odt
	    memory_mem_dm                         =>  HPS_DDR3_DM ,                         --                mem_dm
	    memory_oct_rzqin                      =>  HPS_DDR3_RZQ                          --                .oct_rzqin      

    );


end architecture;
