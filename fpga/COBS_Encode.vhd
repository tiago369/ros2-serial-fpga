library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity COBS_Encode is
    generic (
        MAX_PAYLOAD_LENGTH : integer := 255  -- Maximum payload length (excluding topic ID and CRC)
    );
    port (
        clk          : in  std_logic;                                    -- Clock signal
        reset        : in  std_logic;                                    -- Reset signal
        topic_id     : in  std_logic_vector(7 downto 0);                 -- Topic ID
        payload_in   : in  std_logic_vector(0 to MAX_PAYLOAD_LENGTH-1);  -- Input payload vector
        valid_in     : in  std_logic;                                    -- Input data valid signal
        encoded_data : out std_logic_vector(0 to MAX_PAYLOAD_LENGTH+9);  -- Output encoded data vector
        valid_out    : out std_logic                                     -- Output encoded data valid signal
    );
end COBS_Encode;

architecture rtl of COBS_Encode is
    signal encoded_output : std_logic_vector(0 to MAX_PAYLOAD_LENGTH+9);
    signal output_valid : std_logic := '0';

    constant ZERO_BYTE : std_logic_vector(7 downto 0) := (others => '0');
	 
    signal encoded_index : integer range 0 to MAX_PAYLOAD_LENGTH+9 := 0;
	 
	 function crc16_calc(data_in : std_logic_vector(7 downto 0); crc_in : std_logic_vector(15 downto 0)) return std_logic_vector is
    variable crc_value : std_logic_vector(15 downto 0) := crc_in;
begin
    for i in data_in'range loop
        crc_value(i) := crc_value(i) xor data_in(i);
        
        for j in 0 to 7 loop
            if crc_value(15) = '1' then
                crc_value := (crc_value(14 downto 0) & '0') xor "1000100000100001";
            else
                crc_value := crc_value(14 downto 0) & '0';
            end if;
        end loop;
    end loop;

    return crc_value;
end function;

	 
begin
    process (clk)
        variable payload_start : integer range 0 to MAX_PAYLOAD_LENGTH := 0;
        variable data_index    : integer range 0 to MAX_PAYLOAD_LENGTH-1 := 0;
        variable crc_value     : std_logic_vector(15 downto 0) := (others => '0');  -- Initial CRC value
        
        variable crc_high      : std_logic_vector(7 downto 0) := (others => '0');  -- Placeholder for CRC high byte
        variable crc_low       : std_logic_vector(7 downto 0) := (others => '0');  -- Placeholder for CRC low byte
    begin
        if rising_edge(clk) then
            if reset = '1' then
                payload_start := 0;
                data_index := 0;
                encoded_index <= 0;
                output_valid <= '0';
            elsif valid_in = '1' then
                -- Encode Topic ID
                encoded_output(encoded_index to encoded_index+7) <= topic_id;
                encoded_index <= encoded_index + 8;

                -- Encode Length
                encoded_output(encoded_index to encoded_index+7) <= std_logic_vector(to_unsigned(MAX_PAYLOAD_LENGTH, 8));
                encoded_index <= encoded_index + 8;

                -- Encode Payload
                for i in 0 to MAX_PAYLOAD_LENGTH-1 loop
                    payload_start := payload_start + 1;
                    encoded_output(encoded_index to encoded_index+7) <= payload_in(data_index to data_index+7);
                    encoded_index <= encoded_index + 8;
                    data_index := data_index + 8;
						  
						  -- Update CRC value
						  crc_value := crc16_calc(data_in => payload_in(data_index-8 to data_index-1), crc_in => crc_value);


                    if (data_index = MAX_PAYLOAD_LENGTH) or (payload_start = MAX_PAYLOAD_LENGTH) then
                        payload_start := 0;
                        encoded_output(encoded_index to encoded_index+7) <= ZERO_BYTE;
                        encoded_index <= encoded_index + 8;
                        output_valid <= '1';
                        exit;
                    end if;
                end loop;

                -- Encode CRC
                crc_high := crc_value(15 downto 8);
                crc_low := crc_value(7 downto 0);

                encoded_output(encoded_index to encoded_index+7) <= crc_high;
                encoded_index <= encoded_index + 8;
                encoded_output(encoded_index to encoded_index+7) <= crc_low;
                encoded_index <= encoded_index + 8;

                -- Encode End of Packet marker
                encoded_output(encoded_index to encoded_index+7) <= ZERO_BYTE;
                encoded_index <= encoded_index + 1;
					 encoded_output(encoded_index to MAX_PAYLOAD_LENGTH+9)  <= (others => '0');
                output_valid <= '1';
            end if;
        end if;
    end process;

    -- Assign the remaining bytes to zero
	 
    encoded_data <= encoded_output;

    -- Assign output signals
    valid_out <= output_valid;
end rtl;
