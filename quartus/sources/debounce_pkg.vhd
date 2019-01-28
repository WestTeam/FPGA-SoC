library IEEE;
use     IEEE.STD_LOGIC_1164.ALL;
use     IEEE.NUMERIC_STD.ALL;

package debounce_pkg is

    component debounce is
    generic(
        counter_size  :  INTEGER := 19 --counter size (19 bits gives 10.5ms with 50MHz clock)
    ); 
    port(
        clk     : IN  STD_LOGIC;  --input clock
        button  : IN  STD_LOGIC;  --input signal to be debounced
        result  : OUT STD_LOGIC   --debounced signal  
    ); 
    end component;

end package;
