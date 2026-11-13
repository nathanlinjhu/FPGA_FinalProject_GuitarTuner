-- We took heavy inspiration from the Minilab that is related to switch debouncing
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity button_inputs is
    generic (
        DEBOUNCE_CYCLES : positive := 524288 -- 19-bit counter: about 44 ms at 12 MHz
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;
        start_raw   : in  std_logic;
        stop_raw    : in  std_logic;
        start_pulse : out std_logic;
        stop_level  : out std_logic
    );
end button_inputs;

architecture arch of button_inputs is
    -- Metastability shift register signals
    signal start_d1, start_d2, start_d3 : std_logic := '0';
    signal stop_d1, stop_d2, stop_d3    : std_logic := '0';
    
    -- Counter for debounce cycles
    signal cnt : unsigned(18 downto 0) := (others => '0');
    signal out_start, out_start_d1 : std_logic := '0';
    constant CNT_MAX : unsigned(18 downto 0) :=
        to_unsigned(DEBOUNCE_CYCLES-1, 19);

    -- Placement hints for the synchronizer registers; no extra logic.
    attribute ASYNC_REG : string;
    attribute ASYNC_REG of start_d1, start_d2, start_d3,
                           stop_d1, stop_d2, stop_d3 : signal is "TRUE";

begin

    -- Active high logic (i.e. detect rising edge)
    start_pulse <= out_start and not out_start_d1;
    stop_level <= stop_d3; -- STOP bypasses debounce; controller latches STOPPED.

    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                start_d1 <= '0'; start_d2 <= '0'; start_d3 <= '0';
                stop_d1  <= '0'; stop_d2  <= '0'; stop_d3  <= '0';
                cnt <= (others => '0');
                out_start <= '0';
                out_start_d1 <= '0';
            else
                -- Metastability shift registers
                start_d1 <= start_raw;
                start_d2 <= start_d1;
                start_d3 <= start_d2;
                stop_d1 <= stop_raw;
                stop_d2 <= stop_d1;
                stop_d3 <= stop_d2;

                -- Previous filtered state for detecting a new press.
                out_start_d1 <= out_start;

                -- Saturation counter and hysteresis
                if start_d3 = '0' then
                    if cnt /= 0 then
                        cnt <= cnt - 1;
                    else
                        out_start <= '0';
                    end if;
                elsif start_d3 = '1' then
                    if cnt /= CNT_MAX then
                        cnt <= cnt + 1;
                    else
                        out_start <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process;
end arch;
