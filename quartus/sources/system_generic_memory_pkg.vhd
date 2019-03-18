library IEEE;
use     IEEE.STD_LOGIC_1164.ALL;
use     IEEE.NUMERIC_STD.ALL;
use     IEEE.math_real.all;


package system_generic_memory_pkg is

    component system_generic_memory is
        generic (
            INIT_FILE : string := "system.hex";
            MEMORY_SIZE_BYTES : positive := 65540
        );
	    port (
		    clk         : in  std_logic                     := 'X';             -- clk
		    address     : in  std_logic_vector(integer(ceil(log2(real(MEMORY_SIZE_BYTES/4))))-1 downto 0) := (others => 'X'); -- address
		    clken       : in  std_logic                     := 'X';             -- clken
		    chipselect  : in  std_logic                     := 'X';             -- chipselect
		    write       : in  std_logic                     := 'X';             -- write
		    readdata    : out std_logic_vector(31 downto 0);                    -- readdata
		    writedata   : in  std_logic_vector(31 downto 0) := (others => 'X'); -- writedata
		    byteenable  : in  std_logic_vector(3 downto 0)  := (others => 'X'); -- byteenable
		    reset       : in  std_logic                     := 'X';             -- reset
		    reset_req   : in  std_logic                     := 'X';             -- reset_req
		    address2    : in  std_logic_vector(integer(ceil(log2(real(MEMORY_SIZE_BYTES/4))))-1 downto 0) := (others => 'X'); -- address
		    chipselect2 : in  std_logic                     := 'X';             -- chipselect
		    clken2      : in  std_logic                     := 'X';             -- clken
		    write2      : in  std_logic                     := 'X';             -- write
		    readdata2   : out std_logic_vector(31 downto 0);                    -- readdata
		    writedata2  : in  std_logic_vector(31 downto 0) := (others => 'X'); -- writedata
		    byteenable2 : in  std_logic_vector(3 downto 0)  := (others => 'X'); -- byteenable
		    clk2        : in  std_logic                     := 'X';             -- clk
		    reset2      : in  std_logic                     := 'X';             -- reset
		    reset_req2  : in  std_logic                     := 'X';             -- reset_req
		    freeze      : in  std_logic                     := 'X'              -- freeze
	    );
    end component;

end package;
