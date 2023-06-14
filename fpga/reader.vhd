---------------------------------------------------------------------
-- File Downloaded from http://www.nandland.com
----------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
 
entity uart_protocol is
generic (
    c_CLKS_PER_BIT : integer := 115     -- Needs to be set correctly
    );
  port(reset  : in std_logic;
  r_CLOCK     : in std_logic;
  r_TX_DV     : std_logic;
  r_TX_BYTE   : std_logic_vector(7 downto 0);
  w_TX_SERIAL : out std_logic;
  w_TX_DONE   : out std_logic;
  w_RX_DV     : out std_logic;
  w_RX_BYTE   : out std_logic_vector(7 downto 0);
  r_RX_SERIAL : std_logic
  );
end uart_protocol;
 
architecture behave of uart_protocol is
    type fsm_type is (zero, one, two, three, four, five); -- 2 states are required for Mealy
    signal fsm_reg, fsm_next : fsm_type;

  ------------------------
  -- serial communication
  ------------------------
  component uart_tx is
    generic (
    -- Want to interface to 115200 baud UART
    -- 50000000 / 115200 = 87 Clocks Per Bit.
      c_CLKS_PER_BIT : integer := 434   -- Needs to be set correctly
      );
    port (
	   i_clk       : in  std_logic;
      i_tx_dv     : in  std_logic;
      i_tx_byte   : in  std_logic_vector(7 downto 0);
      o_tx_active : out std_logic;
      o_tx_serial : out std_logic;
      o_tx_done   : out std_logic
	 );
  end component uart_tx;
 
  component uart_rx is
    generic (
      g_CLKS_PER_BIT : integer := 115   -- Needs to be set correctly
      );
	 port(
      i_clk       : in  std_logic;
      i_rx_serial : in  std_logic;
      o_rx_dv     : out std_logic;
      o_rx_byte   : out std_logic_vector(7 downto 0)
      );
  end component uart_rx;
 
  constant c_BIT_PERIOD : time := 8680 ns;
  
  -- Low-level byte-write
  procedure UART_WRITE_BYTE (
    i_data_in       : in  std_logic_vector(7 downto 0);
    signal o_serial : out std_logic) is
  begin
 
    -- Send Start Bit
    o_serial <= '0';
    wait for c_BIT_PERIOD;
 
    -- Send Data Byte
    for ii in 0 to 7 loop
      o_serial <= i_data_in(ii);
      wait for c_BIT_PERIOD;
    end loop;  -- ii
 
    -- Send Stop Bit
    o_serial <= '1'; wait for c_BIT_PERIOD; end UART_WRITE_BYTE; begin -- Instantiate UART transmitter 
	 UART_TX_INST : uart_tx 
	 --generic map (g_CLKS_PER_BIT => c_CLKS_PER_BIT)
    port map (
      i_clk       => r_CLOCK,
      i_tx_dv     => r_TX_DV,
      i_tx_byte   => r_TX_BYTE,
      o_tx_active => open,
      o_tx_serial => w_TX_SERIAL,
      o_tx_done   => w_TX_DONE
      );
 
  -- Instantiate UART Receiver
  UART_RX_INST : uart_rx
    generic map (
      g_CLKS_PER_BIT => c_CLKS_PER_BIT
      )
    port map (
      i_clk       => r_CLOCK,
      i_rx_serial => r_RX_SERIAL,
      o_rx_dv     => w_RX_DV,
      o_rx_byte   => w_RX_BYTE
      );
 
  -----------------------------------------------------
  -- Add Finite State machine to Broke the protocol
  -----------------------------------------------------

  -- Add state change based on clock and reset option
  process(r_CLOCK, reset)
  begin
    if (reset = '1') then -- go to state zero if reset
      fsm_reg <= zero;
    elsif (rising_edge(r_CLOCK)) then -- otherwise update the states
      fsm_reg <= fsm_next;
    else
      null;
    end if; 
  end process;

  process(fsm_reg)
  variable aux_len : integer := 0;
  variable aux_crc : integer := 0;
  
  --signal len : std_logic_vector(16 downto 0) := (others => '0');
  --signal crc : std_logic_vector(16 downto 0) := (others => '0');
  --signal data_full : std_logic := '0';
  --signal payload : std_logic_vector(63 downto 0);

  begin
    fsm_next <= fsm_reg;

    case fsm_reg is
      -- check if header is recieved (topic ID)
      when zero =>
        if w_RX_BYTE = X"0" then -- Discover the topic id so this shit can work
          fsm_next <= one;
        end if;

      -- get len of the buffer
      when one =>
        -- cria contador para pegar os 2 bit e passar pro proximo
        if aux_len < 1 then
          len <= w_RX_BYTE;
          aux_len := aux_len + 1;
        else
          len <= w_RX_BYTE;
          fsm_next <= two;
        end if;

      -- get crc
      when two =>
        -- mesma vibe do anterior
        if aux_crc < 1 then
          crc <= w_RX_BYTE;
          aux_crc := aux_crc + 1;
        else
          crc <= w_RX_BYTE;
          fsm_next <= three;
        end if;
      -- add msgs to the buffer
      when three =>
        -- vai adicionando valores na variavel de payload (ainda tenho que descobirr oq fazer com ela)
        -- vai somando os bits tbm pra pegar o crc (deve ser facil)
        if (w_RX_BYTE /= X"00") then
          payload <= payload & w_RX_BYTE;
        else
          fsm_next <= five;
			 data_full <= '1';
        end if;
      -- check src
      --when four =>
          -- achar uma forma de esvaziar a variavel (c++ seria mais facil)

      -- clear variables to start again
      when five =>
       len <= (others => '0');
       crc <= (others => '0');
		 data_full <= '0';
       fsm_next <= zero;
    end case;
  end process;
  
  
  SEND_DATA : process(r_CLOCK)
  -- signal data : std_logic_vector(63 downto 0);
  variable next_proc : std_logic := '0';
  variable const : integer := 0;
  
  begin
  if(data_full='1') then
    data <= payload;
	 next_proc := '1';
  elsif(rising_edge(r_CLOCK) and next_proc) then
    UART_WRITE_BYTE(data(63-const downto 48-const), r_RX_SERIAL);
	 const := const - 15;
	 if const > 50 then
	   const := 0;
	 end if;
  end if;
  
  end process;
  
   
end behave;