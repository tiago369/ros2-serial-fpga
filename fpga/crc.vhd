library ieee;
use ieee.std_logic_1164.all;

entity CRC is
    generic (
        WIDTH     : integer := 8;         -- Width of input vector
        POLY      : std_logic_vector := x"07";  -- CRC polynomial
        INIT_CRC  : std_logic_vector := x"00"   -- Initial CRC value
    );
    port (
        clk       : in  std_logic;          -- Clock signal
        reset     : in  std_logic;          -- Reset signal
        data_in   : in  std_logic_vector(WIDTH-1 downto 0);  -- Input data vector
        crc_out   : out std_logic_vector(WIDTH-1 downto 0)   -- Output CRC value
    );
end CRC;

architecture rtl of CRC is
    signal reg       : std_logic_vector(WIDTH-1 downto 0);
    signal crc       : std_logic_vector(WIDTH-1 downto 0);
begin
    process (clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                reg <= INIT_CRC;
            else
                reg <= data_in xor crc;
            end if;
        end if;
    end process;

    process (clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                crc <= INIT_CRC;
            else
                crc <= reg;
            end if;
        end if;
    end process;

    crc_out <= crc;
end rtl;