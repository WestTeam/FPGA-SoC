library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity pio_n is
    generic (
        RegCnt : positive := 256;
        ByteEnable: boolean := false
    );
    port (
        clk                     : in  std_logic                       := '0';             --    clk.clk
        reset                   : in  std_logic                       := '0';             --  reset.reset
        data_in_value           : in  std_logic_vector(RegCnt*32-1 downto 0) := (others => '0'); --   data.data_in_value
        data_in_read            : out std_logic_vector(RegCnt-1 downto 0);                       --       .data_in_read
        data_out_value          : out std_logic_vector(RegCnt*32-1 downto 0);                    --       .data_out_value
        data_out_write          : out std_logic_vector(RegCnt-1 downto 0);                       --       .data_out_write
        avl_mm_address    : in  std_logic_vector(7 downto 0)    := (others => '0'); -- avl_mm.address
        avl_mm_writebyteenable  : in  std_logic_vector(3 downto 0)   := (others => '0'); --  .writebyteenable
        avl_mm_writedata  : in  std_logic_vector(31 downto 0)   := (others => '0'); --       .writedata
        avl_mm_chipselect : in  std_logic                       := '0';             --       .chipselect
        avl_mm_readdata   : out std_logic_vector(31 downto 0);                      --       .readdata
        avl_mm_read       : in  std_logic                       := '0'              --       .read
    );
end entity pio_n;

architecture rtl of pio_n is
begin

    p_sync_rd: process(clk,reset) is
    begin
        if reset = '1' then
            avl_mm_readdata <= (others=>'0');
            data_in_read    <= (others=>'0');
        elsif rising_edge(clk) then
            data_in_read <= (others=>'0');
            for i in 0 to RegCnt-1 loop
                if i = unsigned(avl_mm_address) then
                    avl_mm_readdata <= data_in_value((i+1)*32-1 downto i*32);

                    if avl_mm_chipselect = '1' and avl_mm_read = '1' then 
                        data_in_read(i) <= '1';
                    end if;
                end if;
            end loop;
        end if;
    end process;

    p_sync_wr: process(clk,reset) is
    begin
        if reset = '1' then
            data_out_value <= (others=>'0');
            data_out_write <= (others=>'0');
        elsif rising_edge(clk) then
            data_out_write <= (others=>'0');
            for i in 0 to RegCnt-1 loop
                if i = unsigned(avl_mm_address) and avl_mm_chipselect = '1' then--and avl_mm_writebyteenable /= "0000" then
                    for b in 0 to 4-1 loop
                        if avl_mm_writebyteenable(b) = '1' then
                            data_out_value((i)*32+(b+1)*8-1 downto b*8+i*32) <= avl_mm_writedata((b+1)*8-1 downto b*8);
                            data_out_write(i) <= '1';
                        end if;
                    end loop;
                end if;
            end loop;
        end if;
    end process;

end architecture rtl; -- of new_component

