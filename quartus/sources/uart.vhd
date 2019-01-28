--------------------------------------------------------------------------------
-- PROJECT: SIMPLE UART FOR FPGA
--------------------------------------------------------------------------------
-- MODULE:  UART PARITY BIT GENERATOR
-- AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
-- lICENSE: The MIT License (MIT)
-- WEBSITE: https://github.com/jakubcabal/uart_for_fpga
--------------------------------------------------------------------------------
 
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
 
entity UART_PARITY is
    Generic (
        DATA_WIDTH  : integer := 8;
        PARITY_TYPE : string  := "none" -- legal values: "none", "even", "odd", "mark", "space"
    );
    Port (
        DATA_IN     : in  std_logic_vector(DATA_WIDTH-1 downto 0);
        PARITY_OUT  : out std_logic
    );
end UART_PARITY;
 
architecture FULL of UART_PARITY is
 
begin
 
    -- -------------------------------------------------------------------------
    -- PARITY BIT GENERATOR
    -- -------------------------------------------------------------------------
 
    even_parity_g : if (PARITY_TYPE = "even") generate
 
        process (DATA_IN)
        	variable parity_temp : std_logic;
        begin
            parity_temp := '0';
            for i in DATA_IN'range loop
                parity_temp := parity_temp XOR DATA_IN(i);
            end loop;
            PARITY_OUT <= parity_temp;
        end process;
 
    end generate;
 
    odd_parity_g : if (PARITY_TYPE = "odd") generate
 
        process (DATA_IN)
        	variable parity_temp : std_logic;
        begin
            parity_temp := '1';
            for i in DATA_IN'range loop
                parity_temp := parity_temp XOR DATA_IN(i);
            end loop;
            PARITY_OUT <= parity_temp;
        end process;
 
    end generate;
 
    mark_parity_g : if (PARITY_TYPE = "mark") generate
 
        PARITY_OUT <= '1';
 
    end generate;
 
    space_parity_g : if (PARITY_TYPE = "space") generate
 
        PARITY_OUT <= '0';
 
    end generate;
 
end FULL;


--------------------------------------------------------------------------------
-- PROJECT: SIMPLE UART FOR FPGA
--------------------------------------------------------------------------------
-- MODULE:  UART RECEIVER
-- AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
-- lICENSE: The MIT License (MIT)
-- WEBSITE: https://github.com/jakubcabal/uart_for_fpga
--------------------------------------------------------------------------------
 
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
 
entity UART_RX is
    Generic (
        PARITY_BIT  : string := "none" -- legal values: "none", "even", "odd", "mark", "space"
    );
    Port (
        CLK         : in  std_logic; -- system clock
        RST         : in  std_logic; -- high active synchronous reset
        -- UART INTERFACE
        UART_CLK_EN : in  std_logic; -- oversampling (16x) UART clock enable
        UART_RXD    : in  std_logic;
        -- USER DATA OUTPUT INTERFACE
        DATA_OUT    : out std_logic_vector(7 downto 0);
        DATA_VLD    : out std_logic; -- when DATA_VLD = 1, data on DATA_OUT are valid
        FRAME_ERROR : out std_logic  -- when FRAME_ERROR = 1, stop bit was invalid, current and next data may be invalid
    );
end UART_RX;
 
architecture FULL of UART_RX is
 
    signal rx_clk_en          : std_logic;
    signal rx_ticks           : unsigned(3 downto 0);
    signal rx_clk_divider_en  : std_logic;
    signal rx_data            : std_logic_vector(7 downto 0);
    signal rx_bit_count       : unsigned(2 downto 0);
    signal rx_bit_count_en    : std_logic;
    signal rx_data_shreg_en   : std_logic;
    signal rx_parity_bit      : std_logic;
    signal rx_parity_error    : std_logic;
    signal rx_parity_check_en : std_logic;
    signal rx_output_reg_en   : std_logic;
 
    type state is (idle, startbit, databits, paritybit, stopbit);
    signal rx_pstate : state;
    signal rx_nstate : state;
 
begin
 
    -- -------------------------------------------------------------------------
    -- UART RECEIVER CLOCK DIVIDER
    -- -------------------------------------------------------------------------
 
    uart_rx_clk_divider : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (rx_clk_divider_en = '1') then
                if (uart_clk_en = '1') then
                    if (rx_ticks = "1111") then
                        rx_ticks <= (others => '0');
                        rx_clk_en <= '0';
                    elsif (rx_ticks = "0111") then
                        rx_ticks <= rx_ticks + 1;
                        rx_clk_en <= '1';
                    else
                        rx_ticks <= rx_ticks + 1;
                        rx_clk_en <= '0';
                    end if;
                else
                    rx_ticks <= rx_ticks;
                    rx_clk_en <= '0';
                end if;
            else
                rx_ticks <= (others => '0');
                rx_clk_en <= '0';
            end if;
        end if;
    end process;
 
    -- -------------------------------------------------------------------------
    -- UART RECEIVER BIT COUNTER
    -- -------------------------------------------------------------------------
 
    uart_rx_bit_counter : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                rx_bit_count <= (others => '0');
            elsif (rx_bit_count_en = '1' AND rx_clk_en = '1') then
                if (rx_bit_count = "111") then
                    rx_bit_count <= (others => '0');
                else
                    rx_bit_count <= rx_bit_count + 1;
                end if;
            end if;
        end if;
    end process;
 
    -- -------------------------------------------------------------------------
    -- UART RECEIVER DATA SHIFT REGISTER
    -- -------------------------------------------------------------------------
 
    uart_rx_data_shift_reg : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                rx_data <= (others => '0');
            elsif (rx_clk_en = '1' AND rx_data_shreg_en = '1') then
                rx_data <= UART_RXD & rx_data(7 downto 1);
            end if;
        end if;
    end process;
 
    DATA_OUT <= rx_data;
 
    -- -------------------------------------------------------------------------
    -- UART RECEIVER PARITY GENERATOR AND CHECK
    -- -------------------------------------------------------------------------
 
    uart_rx_parity_g : if (PARITY_BIT /= "none") generate
        uart_rx_parity_gen_i: entity work.UART_PARITY
        generic map (
            DATA_WIDTH  => 8,
            PARITY_TYPE => PARITY_BIT
        )
        port map (
            DATA_IN     => rx_data,
            PARITY_OUT  => rx_parity_bit
        );
 
        uart_rx_parity_check_reg : process (CLK)
        begin
            if (rising_edge(CLK)) then
                if (RST = '1') then
                    rx_parity_error <= '0';
                elsif (rx_parity_check_en = '1') then
                    rx_parity_error <= rx_parity_bit XOR UART_RXD;
                end if;
            end if;
        end process;
    end generate;
 
    uart_rx_noparity_g : if (PARITY_BIT = "none") generate
        rx_parity_error <= '0';
    end generate;
 
    -- -------------------------------------------------------------------------
    -- UART RECEIVER OUTPUT REGISTER
    -- -------------------------------------------------------------------------
 
    uart_rx_output_reg : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                DATA_VLD <= '0';
                FRAME_ERROR <= '0';
            else
                if (rx_output_reg_en = '1') then
                    DATA_VLD <= NOT rx_parity_error AND UART_RXD;
                    FRAME_ERROR <= NOT UART_RXD;
                else
                    DATA_VLD <= '0';
                    FRAME_ERROR <= '0';
                end if;
            end if;
        end if;
    end process;
 
    -- -------------------------------------------------------------------------
    -- UART RECEIVER FSM
    -- -------------------------------------------------------------------------
 
    -- PRESENT STATE REGISTER
    process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                rx_pstate <= idle;
            else
                rx_pstate <= rx_nstate;
            end if;
        end if;
    end process;
 
    -- NEXT STATE AND OUTPUTS LOGIC
    process (rx_pstate, UART_RXD, rx_clk_en, rx_bit_count)
    begin
        case rx_pstate is
 
            when idle =>
                rx_output_reg_en <= '0';
                rx_bit_count_en <= '0';
                rx_data_shreg_en <= '0';
                rx_clk_divider_en <= '0';
                rx_parity_check_en <= '0';
 
                if (UART_RXD = '0') then
                    rx_nstate <= startbit;
                else
                    rx_nstate <= idle;
                end if;
 
            when startbit =>
                rx_output_reg_en <= '0';
                rx_bit_count_en <= '0';
                rx_data_shreg_en <= '0';
                rx_clk_divider_en <= '1';
                rx_parity_check_en <= '0';
 
                if (rx_clk_en = '1') then
                    rx_nstate <= databits;
                else
                    rx_nstate <= startbit;
                end if;
 
            when databits =>
                rx_output_reg_en <= '0';
                rx_bit_count_en <= '1';
                rx_data_shreg_en <= '1';
                rx_clk_divider_en <= '1';
                rx_parity_check_en <= '0';
 
                if ((rx_clk_en = '1') AND (rx_bit_count = "111")) then
                    if (PARITY_BIT = "none") then
                        rx_nstate <= stopbit;
                    else
                        rx_nstate <= paritybit;
                    end if ;
                else
                    rx_nstate <= databits;
                end if;
 
            when paritybit =>
                rx_output_reg_en <= '0';
                rx_bit_count_en <= '0';
                rx_data_shreg_en <= '0';
                rx_clk_divider_en <= '1';
                rx_parity_check_en <= '1';
 
                if (rx_clk_en = '1') then
                    rx_nstate <= stopbit;
                else
                    rx_nstate <= paritybit;
                end if;
 
            when stopbit =>
                rx_bit_count_en <= '0';
                rx_data_shreg_en <= '0';
                rx_clk_divider_en <= '1';
                rx_parity_check_en <= '0';
 
                if (rx_clk_en = '1') then
                    rx_nstate <= idle;
                    rx_output_reg_en <= '1';
                else
                    rx_nstate <= stopbit;
                    rx_output_reg_en <= '0';
                end if;
 
            when others =>
                rx_output_reg_en <= '0';
                rx_bit_count_en <= '0';
                rx_data_shreg_en <= '0';
                rx_clk_divider_en <= '0';
                rx_parity_check_en <= '0';
                rx_nstate <= idle;
 
        end case;
    end process;
 
end FULL;


--------------------------------------------------------------------------------
-- PROJECT: SIMPLE UART FOR FPGA
--------------------------------------------------------------------------------
-- MODULE:  UART TRANSMITTER
-- AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
-- lICENSE: The MIT License (MIT)
-- WEBSITE: https://github.com/jakubcabal/uart_for_fpga
--------------------------------------------------------------------------------
 
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
 
entity UART_TX is
    Generic (
        PARITY_BIT  : string := "none" -- legal values: "none", "even", "odd", "mark", "space"
    );
    Port (
        CLK         : in  std_logic; -- system clock
        RST         : in  std_logic; -- high active synchronous reset
        -- UART INTERFACE
        UART_CLK_EN : in  std_logic; -- oversampling (16x) UART clock enable
        UART_TXD    : out std_logic;
        -- USER DATA INPUT INTERFACE
        DATA_IN     : in  std_logic_vector(7 downto 0);
        DATA_SEND   : in  std_logic; -- when DATA_SEND = 1, data on DATA_IN will be transmit, DATA_SEND can set to 1 only when BUSY = 0
        BUSY        : out std_logic  -- when BUSY = 1 transiever is busy, you must not set DATA_SEND to 1
    );
end UART_TX;
 
architecture FULL of UART_TX is
 
    signal tx_clk_en         : std_logic;
    signal tx_clk_divider_en : std_logic;
    signal tx_ticks          : unsigned(3 downto 0);
    signal tx_data           : std_logic_vector(7 downto 0);
    signal tx_bit_count      : unsigned(2 downto 0);
    signal tx_bit_count_en   : std_logic;
    signal tx_busy           : std_logic;
    signal tx_parity_bit     : std_logic;
    signal tx_data_out_sel   : std_logic_vector(1 downto 0);
 
    type state is (idle, txsync, startbit, databits, paritybit, stopbit);
    signal tx_pstate : state;
    signal tx_nstate : state;
 
begin
 
    BUSY <= tx_busy;
 
    -- -------------------------------------------------------------------------
    -- UART TRANSMITTER CLOCK DIVIDER
    -- -------------------------------------------------------------------------
 
    uart_tx_clk_divider : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (tx_clk_divider_en = '1') then
                if (uart_clk_en = '1') then
                    if (tx_ticks = "1111") then
                        tx_ticks <= (others => '0');
                        tx_clk_en <= '0';
                    elsif (tx_ticks = "0001") then
                        tx_ticks <= tx_ticks + 1;
                        tx_clk_en <= '1';
                    else
                        tx_ticks <= tx_ticks + 1;
                        tx_clk_en <= '0';
                    end if;
                else
                    tx_ticks <= tx_ticks;
                    tx_clk_en <= '0';
                end if;
            else
                tx_ticks <= (others => '0');
                tx_clk_en <= '0';
            end if;
        end if;
    end process;
 
    -- -------------------------------------------------------------------------
    -- UART TRANSMITTER INPUT DATA REGISTER
    -- -------------------------------------------------------------------------
 
    uart_tx_input_data_reg : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                tx_data <= (others => '0');
            elsif (DATA_SEND = '1' AND tx_busy = '0') then
                tx_data <= DATA_IN;
            end if;
        end if;
    end process;
 
    -- -------------------------------------------------------------------------
    -- UART TRANSMITTER BIT COUNTER
    -- -------------------------------------------------------------------------
 
    uart_tx_bit_counter : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                tx_bit_count <= (others => '0');
            elsif (tx_bit_count_en = '1' AND tx_clk_en = '1') then
                if (tx_bit_count = "111") then
                    tx_bit_count <= (others => '0');
                else
                    tx_bit_count <= tx_bit_count + 1;
                end if;
            end if;
        end if;
    end process;
 
    -- -------------------------------------------------------------------------
    -- UART TRANSMITTER PARITY GENERATOR
    -- -------------------------------------------------------------------------
 
    uart_tx_parity_g : if (PARITY_BIT /= "none") generate
        uart_tx_parity_gen_i: entity work.UART_PARITY
        generic map (
            DATA_WIDTH  => 8,
            PARITY_TYPE => PARITY_BIT
        )
        port map (
            DATA_IN     => tx_data,
            PARITY_OUT  => tx_parity_bit
        );
    end generate;
 
    uart_tx_noparity_g : if (PARITY_BIT = "none") generate
        tx_parity_bit <= 'Z';
    end generate;
 
    -- -------------------------------------------------------------------------
    -- UART TRANSMITTER OUTPUT DATA REGISTER
    -- -------------------------------------------------------------------------
 
    uart_tx_output_data_reg : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                UART_TXD <= '1';
            else
                case tx_data_out_sel is
                    when "01" => -- START BIT
                        UART_TXD <= '0';
                    when "10" => -- DATA BITS
                        UART_TXD <= tx_data(to_integer(tx_bit_count));
                    when "11" => -- PARITY BIT
                        UART_TXD <= tx_parity_bit;
                    when others => -- STOP BIT OR IDLE
                        UART_TXD <= '1';
                end case;
            end if;
        end if;
    end process;
 
    -- -------------------------------------------------------------------------
    -- UART TRANSMITTER FSM
    -- -------------------------------------------------------------------------
 
    -- PRESENT STATE REGISTER
    process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                tx_pstate <= idle;
            else
                tx_pstate <= tx_nstate;
            end if;
        end if;
    end process;
 
    -- NEXT STATE AND OUTPUTS LOGIC
    process (tx_pstate, DATA_SEND, tx_clk_en, tx_bit_count)
    begin
 
        case tx_pstate is
 
            when idle =>
                tx_busy <= '0';
                tx_data_out_sel <= "00";
                tx_bit_count_en <= '0';
                tx_clk_divider_en <= '0';
 
                if (DATA_SEND = '1') then
                    tx_nstate <= txsync;
                else
                    tx_nstate <= idle;
                end if;
 
            when txsync =>
                tx_busy <= '1';
                tx_data_out_sel <= "00";
                tx_bit_count_en <= '0';
                tx_clk_divider_en <= '1';
 
                if (tx_clk_en = '1') then
                    tx_nstate <= startbit;
                else
                    tx_nstate <= txsync;
                end if;
 
            when startbit =>
                tx_busy <= '1';
                tx_data_out_sel <= "01";
                tx_bit_count_en <= '0';
                tx_clk_divider_en <= '1';
 
                if (tx_clk_en = '1') then
                    tx_nstate <= databits;
                else
                    tx_nstate <= startbit;
                end if;
 
            when databits =>
                tx_busy <= '1';
                tx_data_out_sel <= "10";
                tx_bit_count_en <= '1';
                tx_clk_divider_en <= '1';
 
                if ((tx_clk_en = '1') AND (tx_bit_count = "111")) then
                    if (PARITY_BIT = "none") then
                        tx_nstate <= stopbit;
                    else
                        tx_nstate <= paritybit;
                    end if ;
                else
                    tx_nstate <= databits;
                end if;
 
            when paritybit =>
                tx_busy <= '1';
                tx_data_out_sel <= "11";
                tx_bit_count_en <= '0';
                tx_clk_divider_en <= '1';
 
                if (tx_clk_en = '1') then
                    tx_nstate <= stopbit;
                else
                    tx_nstate <= paritybit;
                end if;
 
            when stopbit =>
                tx_busy <= '0';
                tx_data_out_sel <= "00";
                tx_bit_count_en <= '0';
                tx_clk_divider_en <= '1';
 
                if (DATA_SEND = '1') then
                    tx_nstate <= txsync;
                elsif (tx_clk_en = '1') then
                    tx_nstate <= idle;
                else
                    tx_nstate <= stopbit;
                end if;
 
            when others =>
                tx_busy <= '1';
                tx_data_out_sel <= "00";
                tx_bit_count_en <= '0';
                tx_clk_divider_en <= '0';
                tx_nstate <= idle;
 
        end case;
    end process;
 
end FULL;



--------------------------------------------------------------------------------
-- PROJECT: SIMPLE UART FOR FPGA
--------------------------------------------------------------------------------
-- MODULE:  UART TOP MODULE
-- AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
-- lICENSE: The MIT License (MIT)
-- WEBSITE: https://github.com/jakubcabal/uart_for_fpga
--------------------------------------------------------------------------------
 
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
 
-- UART FOR FPGA REQUIRES: 1 START BIT, 8 DATA BITS, 1 STOP BIT!!!
-- OTHER PARAMETERS CAN BE SET USING GENERICS.
 
entity UART is
    Generic (
        CLK_FREQ    : integer := 50e6;   -- set system clock frequency in Hz
        BAUD_RATE   : integer := 115200; -- baud rate value
        PARITY_BIT  : string  := "none"  -- legal values: "none", "even", "odd", "mark", "space"
    );
    Port (
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
end UART;
 
architecture FULL of UART is
 
    constant divider_value    : integer := CLK_FREQ/(16*BAUD_RATE);
 
    signal uart_ticks         : integer range 0 to divider_value-1;
    signal uart_clk_en        : std_logic;
    signal uart_rxd_shreg     : std_logic_vector(3 downto 0);
    signal uart_rxd_debounced : std_logic;
 
begin
 
    -- -------------------------------------------------------------------------
    -- UART OVERSAMPLING CLOCK DIVIDER
    -- -------------------------------------------------------------------------
 
    uart_oversampling_clk_divider : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                uart_ticks <= 0;
                uart_clk_en <= '0';
            elsif (uart_ticks = divider_value-1) then
                uart_ticks <= 0;
                uart_clk_en <= '1';
            else
                uart_ticks <= uart_ticks + 1;
                uart_clk_en <= '0';
            end if;
        end if;
    end process;
 
    -- -------------------------------------------------------------------------
    -- UART RXD DEBAUNCER
    -- -------------------------------------------------------------------------
 
    uart_rxd_debouncer : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                uart_rxd_shreg <= (others => '1');
                uart_rxd_debounced <= '1';
            else
                uart_rxd_shreg <= UART_RXD & uart_rxd_shreg(3 downto 1);
                uart_rxd_debounced <= uart_rxd_shreg(0) OR
                                      uart_rxd_shreg(1) OR
                                      uart_rxd_shreg(2) OR
                                      uart_rxd_shreg(3);
            end if;
        end if;
    end process;
 
    -- -------------------------------------------------------------------------
    -- UART TRANSMITTER
    -- -------------------------------------------------------------------------
 
    uart_tx_i: entity work.UART_TX
    generic map (
        PARITY_BIT => PARITY_BIT
    )
    port map (
        CLK         => CLK,
        RST         => RST,
        -- UART INTERFACE
        UART_CLK_EN => uart_clk_en,
        UART_TXD    => UART_TXD,
        -- USER DATA INPUT INTERFACE
        DATA_IN     => DATA_IN,
        DATA_SEND   => DATA_SEND,
        BUSY        => BUSY
    );
 
    -- -------------------------------------------------------------------------
    -- UART RECEIVER
    -- -------------------------------------------------------------------------
 
    uart_rx_i: entity work.UART_RX
    generic map (
        PARITY_BIT => PARITY_BIT
    )
    port map (
        CLK         => CLK,
        RST         => RST,
        -- UART INTERFACE
        UART_CLK_EN => uart_clk_en,
        UART_RXD    => uart_rxd_debounced,
        -- USER DATA OUTPUT INTERFACE
        DATA_OUT    => DATA_OUT,
        DATA_VLD    => DATA_VLD,
        FRAME_ERROR => FRAME_ERROR
    );
 
end FULL;
