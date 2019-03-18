library IEEE;
use     IEEE.STD_LOGIC_1164.ALL;
use     IEEE.NUMERIC_STD.ALL;

package system_generic_pkg is

    component system_generic is
        generic (
            INIT_FILE           : string := "system.hex";
            MEMORY_SIZE_BYTES   : positive := 65536
        );
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
    end component;

end package;
