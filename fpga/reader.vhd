---------------------------------------------------------------------
-- File Downloaded from http://www.nandland.com
----------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.all;
 
entity uart_protocol is
  port(reset : in std_logic);
end uart_protocol;
 
architecture behave of uart_protocol is
    type fsm_type is (zero, one, two); -- 2 states are required for Mealy
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
    port ();
  end component uart_tx;
 
  component uart_rx is
    generic (
      g_CLKS_PER_BIT : integer := 115   -- Needs to be set correctly
      );
    port (
      i_clk       : in  std_logic;
      i_rx_serial : in  std_logic;
      o_rx_dv     : out std_logic;
      o_rx_byte   : out std_logic_vector(7 downto 0)
      );
  end component uart_rx;
 
  constant c_BIT_PERIOD : time := 8680 ns;
   
  signal r_CLOCK     : std_logic                    := '0';
  signal r_TX_DV     : std_logic                    := '0';
  signal r_TX_BYTE   : std_logic_vector(7 downto 0) := (others => '0');
  signal w_TX_SERIAL : std_logic;
  signal w_TX_DONE   : std_logic;
  signal w_RX_DV     : std_logic;
  signal w_RX_BYTE   : std_logic_vector(7 downto 0);
  signal r_RX_SERIAL : std_logic := '1';
 
   
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
    o_serial <= '1'; wait for c_BIT_PERIOD; end UART_WRITE_BYTE; begin -- Instantiate UART transmitter UART_TX_INST : uart_tx generic map ( g_CLKS_PER_BIT => c_CLKS_PER_BIT)
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
 
  r_CLOCK <= not r_CLOCK after 50 ns;
  -----------------------------------
  
begin
   
  process is
  begin
 
    -- Tell the UART to send a command.
    wait until rising_edge(r_CLOCK);
    wait until rising_edge(r_CLOCK);
    r_TX_DV   <= '1';
    r_TX_BYTE <= X"AB";
    wait until rising_edge(r_CLOCK);
    r_TX_DV   <= '0';
    wait until w_TX_DONE = '1';
 
     
    -- Send a command to the UART
    wait until rising_edge(r_CLOCK);
    UART_WRITE_BYTE(X"3F", r_RX_SERIAL);
    wait until rising_edge(r_CLOCK);
 
    -- Check that the correct command was received
    if w_RX_BYTE = X"3F" then
      report "Test Passed - Correct Byte Received" severity note;
    else
      report "Test Failed - Incorrect Byte Received" severity note;
    end if;
 
    assert false report "Tests Complete" severity failure;
     
  end process;
  -----------------------------------------------------
  -- Add Finite State machine to Broke the protocol
  -----------------------------------------------------

  -- Add state change based on clock and reset option
  process(r_CLOCK, reset)
  begin
    if (reset = '1') then -- go to state zero if reset
      fsm <= zero;
    elsif (is_rising(r_CLOCK)) then -- otherwise update the states
      fsm_reg <= fsm_next;
    else
      null;
    end if; 
  end process;


  process(fsm_reg)
  variable aux_len : integer := 0;
  variable aux_crc : integer := 0;

  begin
    fsm_next <= fsm_reg;

    case fsm_reg is
      -- check if header is recieved (topic ID)
      when zero =>
        if w_RX_BYTE = X"" then -- Discover the topic id so this shit can work
          fsm_next <= one;
        end if;
      -- get len of the buffer
      when one =>
        -- cria contador para pegar os 2 bit e passar pro proximo
      -- get crc
      when two =>
        -- mesma vibe do anterior
      -- add msgs to the buffer
      when three =>
        -- vai adicionando valores na variavel de payload (ainda tenho que descobirr oq fazer com ela)
        -- vai somando os bits tbm pra pegar o crc (deve ser facil)
      -- check src
      when four =>
          -- achar uma forma de esvaziar a variavel (c++ seria mais facil)
      -- clear variables to start again
      when five =>
    end case;
  end process;
   
end behave;