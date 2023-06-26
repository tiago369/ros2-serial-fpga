library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity crc16 is
    port (
        clk      : in  std_logic;
        reset    : in  std_logic;
        data_in  : in  std_logic_vector(7 downto 0);
        crc_in   : in  std_logic_vector(15 downto 0);
        crc_out  : out std_logic_vector(15 downto 0)
    );
end entity crc16;

architecture rtl of crc16 is
    signal crc_reg : std_logic_vector(15 downto 0);

    constant crc_poly : std_logic_vector(15 downto 0) := "10001000000100001";  -- CRC-16 polynomial (0x8005)
begin
    process (clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                crc_reg <= (others => '0');
            else
                crc_reg <= crc_reg xor ("00" & data_in);
                for i in 0 to 7 loop
                    if crc_reg(15) = '1' then
                        crc_reg <= crc_reg(14 downto 0) xor crc_poly;
                    else
                        crc_reg <= crc_reg(14 downto 0) & '0';
                    end if;
                end loop;
            end if;
        end if;
    end process;

    crc_out <= crc_reg;
end architecture rtl;
