library IEEE;
use     IEEE.STD_LOGIC_1164.ALL;
use     IEEE.NUMERIC_STD.ALL;

package qei_pkg is

    component QuadratureCounterPorts is
    port (
         clock : in std_logic;	--system clock, i.e. 10MHz oscillator
		 QuadA : in std_logic;	--first input from quadrature device  (i.e. optical disk encoder)
		 QuadB : in std_logic;	--second input from quadrature device (i.e. optical disk encoder)
		 CounterValue : out std_logic_vector(15 downto 0) --just an example debuggin output
    );
    end component;

end package;
