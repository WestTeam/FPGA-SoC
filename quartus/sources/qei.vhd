library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_SIGNED.ALL;
 
-- c2003 Franks Development, LLC
-- http://www.franks-development.com
-- !This source is distributed under the terms & conditions specified at opencores.org
 
--resource or companion to this code: 
	-- Xilinx Application note 12 - "Quadrature Phase Decoder" - xapp012.pdf
	-- no longer appears on xilinx website (to best of my knowledge), perhaps it has been superceeded?
 
--this code was origonally intended for use on Xilinx XPLA3 'coolrunner' CPLD devices
--origonally compiled/synthesized with Xilinx 'Webpack' 5.2 software
 
--How we 'talk' to the outside world:
entity QuadratureCounterPorts is
    Port ( clock : in std_logic;	--system clock, i.e. 10MHz oscillator
		 QuadA : in std_logic;	--first input from quadrature device  (i.e. optical disk encoder)
		 QuadB : in std_logic;	--second input from quadrature device (i.e. optical disk encoder)
		 CounterValue : out std_logic_vector(15 downto 0) --just an example debuggin output
		);
end entity;
 
--What we 'do':
architecture QuadratureCounter of QuadratureCounterPorts is
 
	-- local 'variables' or 'registers'
 
	--This is the counter for how many quadrature ticks have gone past.
	--the size of this counter is dependant on how far you need to count
	--it was origonally used with a circular disk encoder having 2048 ticks/revolution
	--thus this 16-bit count could hold 2^15 ticks in either direction, or a total
	--of 32768/2048 = 16 revolutions in either direction.  if the disk
	--was turned more than 16 times in a given direction, the counter overflows
	--and the origonal location is lost.  If you had a linear instead of 
	--circular encoder that physically could not move more than 2048 ticks,
	--then Count would only need to be 11 downto 0, and you could count
	--2048 ticks in either direction, regardless of the position of the 
	--encoder at system bootup.
	signal Count : std_logic_vector(15 downto 0);
 
	--this is the signal from the quadrature logic that it is time to change
	--the value of the counter on this clock signal (either + or -)
	signal CountEnable : std_logic;
 
	--should we increment or decrement count?
	signal CountDirection : std_logic;
 
	--where all the 'work' is done: quadraturedecoder.vhd
	component QuadratureDecoderPorts
    		Port (
        			clock     : in    std_logic;
        			QuadA     : in    std_logic;
        			QuadB     : in    std_logic;
        			Direction : out std_logic;
	   			CountEnable : out std_logic
    			);
	end component;
 
	begin --architecture QuadratureCounter		 
 
	--instanciate the decoder
	iQuadratureDecoder: QuadratureDecoderPorts 
	port map	( 
 				clock => clock,
	      		QuadA => QuadA,
 	   			QuadB => QuadB,
    				Direction => CountDirection,
	       		CountEnable => CountEnable
			);
 
 
	-- do our actual work every clock cycle
	process(clock)
	begin
 
		--keep track of the counter
		if ( (clock'event) and (clock = '1') ) then
 
			if (CountEnable = '1') then
 
				if (CountDirection = '1') then Count <= Count + "0000000000000001"; end if;
				if (CountDirection = '0') then Count <= Count - "0000000000000001"; end if;
 
			end if;
 
		end if; --clock'event
 
		--!!!!!!!!!!!INSERT SOMETHING USEFULL HERE!!!!!!!!!!!
		--This is where you do actual work based on the value of the counter
		--for instance, I will just output the value of the counter
		--led's on an output like this are very useful - you can see the top
		--bits light when moved backwards from initial position (count goes negative)
		CounterValue <= Count;
 
	end process; --(clock)
 
end architecture;



library ieee;
use ieee.std_logic_1164.all;
 
-- c2003 Franks Development, LLC
-- http://www.franks-development.com
-- !This source is distributed under the terms & conditions specified at opencores.org
 
--How we 'talk' to the outside world:
entity QuadratureDecoderPorts is
    port (
       	clock : 		in std_logic;
       	QuadA : 		in std_logic;
       	QuadB : 		in std_logic;
       	Direction : 	out std_logic;
	   	CountEnable :	out std_logic
    );
end entity;
 
--What we 'do':
architecture QuadratureDecoder of QuadratureDecoderPorts is
 
	--local 'variables' or 'registers'
 
	--this runs our state machine: where are we in the decoding process?
	--the following constants describe each state
	--note that every possible state is not listed.  the unused states
	--are physically unreachable in a functioning quadratre device, given that the 
	--clock is fast enough to 'catch' each transition on the quadrature inputs
	--LR means left-right, RL = left-right.  Of course the two are reversed
	--if the two quadratre inputs are switched.
	signal state : std_logic_vector(3 downto 0);
	constant Wait0 : std_logic_vector(3 downto 0) := "0000";
	constant Wait1 : std_logic_vector(3 downto 0) := "0001";
	constant Count0 : std_logic_vector(3 downto 0) := "0010";
	constant Count1 : std_logic_vector(3 downto 0) := "0011";
	constant LR1 : std_logic_vector(3 downto 0) := "1001";
	constant LR2 : std_logic_vector(3 downto 0) := "1101";
	constant LR3 : std_logic_vector(3 downto 0) := "0101";
	constant RL1 : std_logic_vector(3 downto 0) := "0100";
	constant RL2 : std_logic_vector(3 downto 0) := "1100";
	constant RL3 : std_logic_vector(3 downto 0) := "1000";
 
	--this is a temp where the two quadrature inputs are stored
	signal Quad : std_logic_vector(1 downto 0);
 
	--as a single quadrature count is made up of several states, and the decoder
	--can remain in a given state indefinately (if the quadrature input
	--device is not 'moving'), so we need these 'gate-ing' variables
	--to keep us from counting on every clock when we sit idle in the 
	--'count' state; thusly, we just count on the first clock 
	--upon entering a 'count' state.
	signal counted : std_logic;
	signal counting : std_logic;
 
begin   --architecture QuadratureDecoder
 
process (clock)
 
	begin --(clock)
 
	if ( (clock'event) and (clock = '1') ) 	then --every rising edge
 
		--convert inputs from asynch to synch by assigning once on each rising edge of clock
		Quad(0) <= QuadA;
		Quad(1) <= QuadB;	
 
		--we are not going to be counting on this clock by default
		CountEnable <= '0';
 
		--we are not in a 'count' state
		if (Counting = '0') then
 
			Counted <= '0';   --haven't counted when not in count state
			CountEnable <= '0';	 --are not outputing a count either
 
		end if;
 
		--we are in a count state
		if (Counting = '1') then
 
			if (Counted = '1') then	  --note that this is covered by default, but is included for clarity.
				CountEnable <= '0';	  --already counted this one, don't output a count
			end if;
 
			if (Counted = '0') then	   --we haven't counted it already
				Counted <= '1';	   --make sure we dont count it again on next clock
				CountEnable <= '1';	   --output a count!
			end if;
 
		end if;
 
		-- run our state machine
		-- the state transitions are governed by the nature of reality -
		-- vis-a-vis this is what quadratre is.
		-- the '--?' are the physically un-reachable states.
		-- note that it is imperative that the clock be at least (4 I recal)
		-- times faster than the maximum transition rate on each quadratre
		-- input, or else transitions will occur in between clocks, corrupting
		-- the state of the decoder.  Put differently, the quadratre device must
		-- physically remain in each state for at least a single clock
		-- or state changes will not be 'captured' and decoder output will be bogus.
		-- which is substancially the case with any clock-based logic.
		-- the difference is that a normal glitch is any change in input which
		-- has duration less than a single clock, but in quadrature, as single
		-- transition of the actual device cases 4 transitions in the state,
		-- by design of the quadrature encoding process.
		case state is
 
			when Wait0 =>
				if (Quad = "00") then state <= Wait0; end if;
				if (Quad = "01") then state <= RL1; end if;
				if (Quad = "10") then state <= LR1; end if;
				if (Quad = "11") then state <= Wait0; end if; --?
				Counting <= '0';
 
			when Wait1 =>
				if (Quad = "00") then state <= Wait0; end if;
				if (Quad = "01") then state <= RL1; end if;
				if (Quad = "10") then state <= LR1; end if;
				if (Quad = "11") then state <= Wait0; end if; --?
				Counting <= '0';
 
			when Count0 =>
				if (Quad = "00") then state <= Wait0; end if;
				if (Quad = "01") then state <= RL1; end if;
				if (Quad = "10") then state <= LR1; end if;
				if (Quad = "11") then state <= Count0; end if; --?
				Counting <= '1';
 
			when Count1 =>
				if (Quad = "00") then state <= Wait0; end if;
				if (Quad = "01") then state <= RL1; end if;
				if (Quad = "10") then state <= LR1; end if;
				if (Quad = "11") then state <= Count0; end if; --?
				Counting <= '1';
 
			when LR1 =>
				if (Quad = "00") then state <= Wait0; end if;
				if (Quad = "01") then state <= LR1; end if; --?
				if (Quad = "10") then state <= LR1; end if;
				if (Quad = "11") then state <= LR2; end if;
				Direction <= '0';
				Counting <= '0';
 
			when LR2 =>
				if (Quad = "00") then state <= LR2; end if; --?
				if (Quad = "01") then state <= LR3; end if;
				if (Quad = "10") then state <= LR1; end if;
				if (Quad = "11") then state <= LR2; end if; --?
				Direction <= '0';
				Counting <= '0';
 
			when LR3 =>
				if (Quad = "00") then state <= Count0; end if;
				if (Quad = "01") then state <= LR3; end if;
				if (Quad = "10") then state <= LR3; end if; --?
				if (Quad = "11") then state <= LR2; end if;
				Direction <= '0';
				Counting <= '0';
 
			when RL1 =>
				if (Quad = "00") then state <= Wait0; end if;
				if (Quad = "01") then state <= RL1; end if;
				if (Quad = "10") then state <= RL1; end if; --?
				if (Quad = "11") then state <= RL2; end if;
				Direction <= '1';
				Counting <= '0';
 
			when RL2 =>
				if (Quad = "00") then state <= RL2; end if; --?
				if (Quad = "01") then state <= RL1; end if;
				if (Quad = "10") then state <= RL3; end if;
				if (Quad = "11") then state <= RL2; end if; --?
				Direction <= '1';
				Counting <= '0';
 
			when RL3 =>
				if (Quad = "00") then state <= Count0; end if;
				if (Quad = "01") then state <= RL3; end if; --?
				if (Quad = "10") then state <= RL3; end if;
				if (Quad = "11") then state <= RL2; end if;
				Direction <= '1';
				Counting <= '0';
 
			when others => state <= Wait0; -- undefined state; just go back to wait so we don't get stuck here...
 
		end case; --state
 
	end if; --clock'event
 
	end process; --(clock)
 
end architecture;
