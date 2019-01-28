library IEEE;
use     IEEE.STD_LOGIC_1164.ALL;
use     IEEE.NUMERIC_STD.ALL;

package pwm_pkg is

    component pwm is
    generic(
        sys_clk         : INTEGER := 50_000_000; --system clock frequency in Hz
        pwm_freq        : INTEGER := 100_000;    --PWM switching frequency in Hz
        bits_resolution : INTEGER := 8;          --bits of resolution setting the duty cycle
        phases          : INTEGER := 1);         --number of output pwms and phases
    port(
        clk       : IN  STD_LOGIC;                                    --system clock
        reset_n   : IN  STD_LOGIC;                                    --asynchronous reset
        ena       : IN  STD_LOGIC;                                    --latches in new duty cycle
        duty      : IN  STD_LOGIC_VECTOR(bits_resolution-1 DOWNTO 0); --duty cycle
        pwm_out   : OUT STD_LOGIC_VECTOR(phases-1 DOWNTO 0);          --pwm outputs
        pwm_n_out : OUT STD_LOGIC_VECTOR(phases-1 DOWNTO 0)           --pwm inverse outputs
    );         
    end component;

end package;
