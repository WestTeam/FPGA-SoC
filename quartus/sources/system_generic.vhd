library IEEE;
use     IEEE.STD_LOGIC_1164.ALL;
use     IEEE.NUMERIC_STD.ALL;
use     IEEE.math_real.all;

library work;
use     work.system_generic_memory_pkg.all;

entity system_generic is
    generic (
        INIT_FILE           : string := "system.hex";
        MEMORY_SIZE_BYTES   : positive := 65536
    );
    port (
        clk                     : in  std_logic                      := 'X';             -- clk
        reset_n                 : in  std_logic                      := 'X';              -- reset_n
        pio_data_in_value       : in  std_logic_vector(2048-1 downto 0) := (others => 'X'); -- data_in_value
        pio_data_in_read        : out std_logic_vector(64-1 downto 0);                     -- data_in_read
        pio_data_out_value      : out std_logic_vector(2048-1 downto 0);                    -- data_out_value
        pio_data_out_write      : out std_logic_vector(64-1 downto 0);                     -- data_out_write
		uart_0_rxd              : in  std_logic                      := '1';             -- rxd
		uart_0_txd              : out std_logic;                                         -- txd
		uart_1_rxd              : in  std_logic                      := '1';             -- rxd
		uart_1_txd              : out std_logic                                          -- txd
    );
end entity;
 
architecture rtl of system_generic is

	component system is
		port (
			clk_clk                              : in  std_logic                      := 'X';             -- clk
			memory_data_address              : out std_logic_vector(23 downto 0);                     -- address
			memory_data_burstcount           : out std_logic_vector(0 downto 0);                      -- burstcount
			memory_data_read                 : out std_logic;                                         -- read
			memory_data_write                : out std_logic;                                         -- write
			memory_data_waitrequest          : in  std_logic                      := 'X';             -- waitrequest
			memory_data_readdatavalid        : in  std_logic                      := 'X';             -- readdatavalid
			memory_data_byteenable           : out std_logic_vector(3 downto 0);                      -- byteenable
			memory_data_readdata             : in  std_logic_vector(31 downto 0)  := (others => 'X'); -- readdata
			memory_data_writedata            : out std_logic_vector(31 downto 0);                     -- writedata
			memory_data_lock                 : out std_logic;                                         -- lock
			memory_data_debugaccess          : out std_logic;                                         -- debugaccess
			memory_data_clken                : out std_logic;                                         -- clken
			memory_instruction_address       : out std_logic_vector(23 downto 0);                     -- address
			memory_instruction_burstcount    : out std_logic_vector(0 downto 0);                      -- burstcount
			memory_instruction_read          : out std_logic;                                         -- read
			memory_instruction_write         : out std_logic;                                         -- write
			memory_instruction_waitrequest   : in  std_logic                      := 'X';             -- waitrequest
			memory_instruction_readdatavalid : in  std_logic                      := 'X';             -- readdatavalid
			memory_instruction_byteenable    : out std_logic_vector(3 downto 0);                      -- byteenable
			memory_instruction_readdata      : in  std_logic_vector(31 downto 0)  := (others => 'X'); -- readdata
			memory_instruction_writedata     : out std_logic_vector(31 downto 0);                     -- writedata
			memory_instruction_lock          : out std_logic;                                         -- lock
			memory_instruction_debugaccess   : out std_logic;                                         -- debugaccess
			memory_instruction_clken         : out std_logic;                                         -- clken
			pio_data_in_value                : in  std_logic_vector(2048-1 downto 0) := (others => 'X'); -- data_in_value
			pio_data_in_read                 : out std_logic_vector(64-1 downto 0);                     -- data_in_read
			pio_data_out_value               : out std_logic_vector(2048-1 downto 0);                    -- data_out_value
			pio_data_out_write               : out std_logic_vector(64-1 downto 0);                     -- data_out_write
			reset_reset_n                          : in  std_logic                      := 'X';             -- reset_n
			uart_0_rxd                       : in  std_logic                      := 'X';             -- rxd
			uart_0_txd                       : out std_logic;                                          -- txd
			uart_1_rxd                       : in  std_logic                      := 'X';             -- rxd
			uart_1_txd                       : out std_logic             
        );
	end component system;

    signal w_memory_data_address              : std_logic_vector(23 downto 0);                     -- address
    signal w_memory_data_burstcount           : std_logic_vector(0 downto 0);                      -- burstcount
    signal w_memory_data_read                 : std_logic;                                         -- read
    signal w_memory_data_write                : std_logic;                                         -- write
    signal w_memory_data_waitrequest          : std_logic                      ;             -- waitrequest
    signal w_memory_data_readdatavalid        : std_logic                      := '0';             -- readdatavalid
    signal w_memory_data_byteenable           : std_logic_vector(3 downto 0);                      -- byteenable
    signal w_memory_data_readdata             : std_logic_vector(31 downto 0)  ; -- readdata
    signal w_memory_data_writedata            : std_logic_vector(31 downto 0);                     -- writedata
    signal w_memory_data_lock                 : std_logic;                                         -- lock
    signal w_memory_data_debugaccess          : std_logic;                                         -- debugaccess
    signal w_memory_data_clken                : std_logic;                                         -- clken

    signal w_memory_instruction_address       : std_logic_vector(23 downto 0);                     -- address
    signal w_memory_instruction_burstcount    : std_logic_vector(0 downto 0);                      -- burstcount
    signal w_memory_instruction_read          : std_logic;                                         -- read
    signal w_memory_instruction_write         : std_logic;                                         -- write
    signal w_memory_instruction_waitrequest   : std_logic                      ;             -- waitrequest
    signal w_memory_instruction_readdatavalid : std_logic                       := '0';             -- readdatavalid
    signal w_memory_instruction_byteenable    : std_logic_vector(3 downto 0);                      -- byteenable
    signal w_memory_instruction_readdata      : std_logic_vector(31 downto 0)  ; -- readdata
    signal w_memory_instruction_writedata     : std_logic_vector(31 downto 0);                     -- writedata
    signal w_memory_instruction_lock          : std_logic;                                         -- lock
    signal w_memory_instruction_debugaccess   : std_logic;                                         -- debugaccess
    signal w_memory_instruction_clken         : std_logic;   

    signal w_reset : std_logic;

    constant ADDR_WIDTH : natural := integer(ceil(log2(real(MEMORY_SIZE_BYTES/4))));

begin

    w_reset <= not reset_n;

    w_memory_data_waitrequest <= '0';

    w_memory_instruction_waitrequest <= '0';

    process(clk) is
    begin
        if rising_edge(clk) then
            w_memory_data_readdatavalid <= w_memory_data_readdatavalid;
            if w_memory_data_clken = '1' then
                w_memory_data_readdatavalid <= w_memory_data_read;
            end if;
            w_memory_instruction_readdatavalid <= w_memory_instruction_readdatavalid;

            if w_memory_instruction_clken = '1' then
                w_memory_instruction_readdatavalid <= w_memory_instruction_read;
            end if;

        end if;
    end process;

	inst_memory : system_generic_memory
    generic map (
        INIT_FILE => INIT_FILE,
        MEMORY_SIZE_BYTES => MEMORY_SIZE_BYTES
    )
	port map (
		clk         => clk,                                          --   clk1.clk
		address     => w_memory_data_address(ADDR_WIDTH+2-1 downto 2),    --     s1.address
		clken       => w_memory_data_clken,      --       .clken
		chipselect  => '1', --       .chipselect
		write       => w_memory_data_write,      --       .write
		readdata    => w_memory_data_readdata,   --       .readdata
		writedata   => w_memory_data_writedata,  --       .writedata
		byteenable  => w_memory_data_byteenable, --       .byteenable
		reset       => w_reset,                   -- reset1.reset
		reset_req   => w_reset,               --       .reset_req
		address2    => w_memory_instruction_address(ADDR_WIDTH+2-1 downto 2),    --     s2.address
		chipselect2 => '1', --       .chipselect
		clken2      => w_memory_instruction_clken,      --       .clken
		write2      => w_memory_instruction_write,      --       .write
		readdata2   => w_memory_instruction_readdata,   --       .readdata
		writedata2  => w_memory_instruction_writedata,  --       .writedata
		byteenable2 => w_memory_instruction_byteenable, --       .byteenable
		clk2        => clk,                                          --   clk2.clk
		reset2      => w_reset,                   -- reset2.reset
		reset_req2  => w_reset,               --       .reset_req
		freeze      => '0'                                               -- (terminated)
	);



    inst_system : component system
    port map (
        clk_clk                     => clk,
        reset_reset_n                 => reset_n,

		memory_data_address              => w_memory_data_address,
		memory_data_burstcount           => w_memory_data_burstcount,
		memory_data_read                 => w_memory_data_read,
		memory_data_write                => w_memory_data_write,
		memory_data_waitrequest          => w_memory_data_waitrequest,
		memory_data_readdatavalid        => w_memory_data_readdatavalid,
		memory_data_byteenable           => w_memory_data_byteenable,
		memory_data_readdata             => w_memory_data_readdata,
		memory_data_writedata            => w_memory_data_writedata,
		memory_data_lock                 => w_memory_data_lock,
		memory_data_debugaccess          => w_memory_data_debugaccess,
		memory_data_clken                => w_memory_data_clken,

		memory_instruction_address       => w_memory_instruction_address,
		memory_instruction_burstcount    => w_memory_instruction_burstcount,
		memory_instruction_read          => w_memory_instruction_read,
		memory_instruction_write         => w_memory_instruction_write,
		memory_instruction_waitrequest   => w_memory_instruction_waitrequest,
		memory_instruction_readdatavalid => w_memory_instruction_readdatavalid,
		memory_instruction_byteenable    => w_memory_instruction_byteenable,
		memory_instruction_readdata      => w_memory_instruction_readdata,
		memory_instruction_writedata     => w_memory_instruction_writedata,
		memory_instruction_lock          => w_memory_instruction_lock,
		memory_instruction_debugaccess   => w_memory_instruction_debugaccess,
		memory_instruction_clken         => w_memory_instruction_clken,

        pio_data_in_value       => pio_data_in_value,
        pio_data_in_read        => pio_data_in_read,
        pio_data_out_value      => pio_data_out_value,
        pio_data_out_write      => pio_data_out_write,
	    uart_0_rxd              => uart_0_rxd,
	    uart_0_txd              => uart_0_txd,
	    uart_1_rxd              => uart_1_rxd,
	    uart_1_txd              => uart_1_txd
    );

 
end architecture;

