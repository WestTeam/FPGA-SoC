library IEEE;
use     IEEE.STD_LOGIC_1164.ALL;
use     IEEE.NUMERIC_STD.ALL;

library work;
use     work.types_pkg.all;

package robot_layer_3_pkg is

    component robot_layer_3 is
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
    end component;


end package;
