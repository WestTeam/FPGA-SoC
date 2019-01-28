library IEEE;
use     IEEE.STD_LOGIC_1164.ALL;
use     IEEE.NUMERIC_STD.ALL;

package uart_pkg is

    component UART is
    generic (
        CLK_FREQ    : integer := 50e6;   -- set system clock frequency in Hz
        BAUD_RATE   : integer := 115200; -- baud rate value
        PARITY_BIT  : string  := "none"  -- legal values: "none", "even", "odd", "mark", "space"
    );
    port (
        CLK         : in  std_logic; -- system clock
        RST         : in  std_logic; -- high active synchronous reset
        -- UART INTERFACE
        UART_TXD    : out std_logic;
        UART_RXD    : in  std_logic;
        -- USER DATA INPUT INTERFACE
        DATA_IN     : in  std_logic_vector(7 downto 0);
        DATA_SEND   : in  std_logic; -- when DATA_SEND = 1, data on DATA_IN will be transmit, DATA_SEND can set to 1 only when BUSY = 0
        BUSY        : out std_logic; -- when BUSY = 1 transiever is busy, you must not set DATA_SEND to 1
        -- USER DATA OUTPUT INTERFACE
        DATA_OUT    : out std_logic_vector(7 downto 0);
        DATA_VLD    : out std_logic; -- when DATA_VLD = 1, data on DATA_OUT are valid
        FRAME_ERROR : out std_logic  -- when FRAME_ERROR = 1, stop bit was invalid, current and next data may be invalid
    );
    end component;

end package;
