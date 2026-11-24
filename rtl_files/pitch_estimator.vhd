library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.tuner_pkg.all;


entity pitch_estimator is
    port (
       -- data_i = a complex FFT result; 16 bits of real part and 16 bits of imaginary
       -- user_i = FFT data with bin index and scaling exponent
       -- valid_i = indicates that FFT output is valid this cycle
       -- last_i =  marks final FFT output in the frame
       -- frame_good_i = indicates whether ADC passed earlier quality checks
       -- done_o =  a clock pulse when the four strings have been processed
       -- valid_o = one flag per string indicating an accepted pitch range
       -- pitch_q8_o = estimated pitches in (measured in 1/256 of an FFT bin) for 4 strings
    clk : in std_logic;
	rst : in std_logic;
    data_i : in std_logic_vector(31 downto 0);
    user_i : in std_logic_vector(23 downto 0);
    valid_i: in std_logic;
	last_i : in std_logic;
	frame_good_i : in std_logic;
    done_o : out std_logic;
    valid_o : out std_logic_vector(3 downto 0);
    pitch_q8_o : out int4_t
    );
end entity;

architecture rtl of pitch_estimator is
    -- Store only the positive-frequency band needed by these four strings.
    -- For each powers(k) it will compute the power and store it as a 32 bit number per FFT bin
    type power_mem_t is array(0 to 300) of unsigned(31 downto 0);
    signal powers : power_mem_t;

    -- Tells Vivado to use FPGA RAM
    attribute ram_style : string;
    attribute ram_style of powers : signal is "distributed";

    type state_t is (RECEIVE, SCAN, SAVE_RESULT, FINISHED);
    signal state : state_t := RECEIVE;
    
    -- s = which string is being analyzed
    -- k = which bin is being checked
    -- peak_bin = where the strongest bin was found
    -- peak = power of peak_bin
    signal s : natural range 0 to 3 := 0;
    signal k : natural range 0 to 300 := 0;
    signal peak_bin : natural range 0 to 300 := 0;
    signal peak : unsigned(31 downto 0) := (others => '0');
    
    -- sample_count = counts the received FFT outputs
    -- exponent = records FFT amplitude scale
    -- bad = remembers any problems
    -- results = holds the 4 pitches to drive pitch_q8_o
    -- flags = drives valid_o
    signal sample_count : natural range 0 to FFT_N := 0;
    signal exponent : natural range 0 to 255 := 0;
    signal bad : std_logic := '0';
    signal results : int4_t := (others => 0);
    signal flags : std_logic_vector(3 downto 0) := (others => '0');
begin
    pitch_q8_o <= results;
    valid_o <= flags;

    -- FFT output is always accepted by fft_measurement. The IP must
    -- supply natural-order bins and the re,im as listed
    process(clk)
	-- address_v = tracks the FFT bin address
	-- re = real part of FFT output
 	-- im = imaginary part of FFT output
	-- rr = real part squared
	-- ii = imaginary part squared
    variable address_v : natural range 0 to 4095;
    variable re : signed(15 downto 0);
	variable im : signed(15 downto 0);
    variable rr : signed(31 downto 0);
	variable ii : signed(31 downto 0);

    begin
        if rising_edge(clk) then
	    -- Store data when receiving a frame and FFT output is valid
            if rst = '0' and state = RECEIVE and valid_i = '1' then
  		-- Get the address of the FFT bin
                address_v := to_integer(unsigned(user_i(11 downto 0)));
		-- If said address is less than or equal to 300 (i.e. within our accepted range), 
 		-- extract real and imaginary components and compute the squares
		-- Update the power for the corresponding FFT bin
                if address_v <= 300 then
                    re := signed(data_i(15 downto 0));
                    im := signed(data_i(31 downto 16));
                    rr := re * re;
                    ii := im * im;
                    powers(address_v) <= unsigned(rr) + unsigned(ii);
                end if;
            end if;
        end if;
    end process;

    process(clk)
	-- ix = variable to keep track of the FFT output index 
        variable ix : natural range 0 to 4095;
        variable restored : unsigned(63 downto 0);
    begin
        if rising_edge(clk) then
	    -- The default completion is LOW, the FINISHED state will override it 
            done_o <= '0';
	    -- Synchronous active high reset
            if (rst = '1') then
                state <= RECEIVE;
                flags <= (others => '0');
                results <= (others => 0);
                sample_count <= 0;
                exponent <= 0;
                bad <= '0';
                s <= 0;
                k <= 0;
                peak_bin <= 0;
                peak <= (others => '0');
            else
                case state is
                    when RECEIVE =>
		 	-- If valid, then our index is provided via the FFT
			-- If it is not equal to the current number of FFT received, invalid
                        if valid_i = '1' then
                            ix := to_integer(unsigned(user_i(11 downto 0)));

                            if (ix /= sample_count) then 
                                bad <= '1'; 
                            end if;
			    -- Increment the sample count as long as it doesn't exceed size
                            if (sample_count < FFT_N) then
                                sample_count <= sample_count + 1;
                            else
                                bad <= '1';
                            end if;

			    -- exponent comes from other part of FFT data
                            exponent <= to_integer(unsigned(user_i(23 downto 16)));
			    -- If we are last sample, if we are okay, then reset and go to SCAN
                            if (last_i = '1') then
                                if (ix /= FFT_N-1) or (sample_count /= FFT_N-1)
                                   or (frame_good_i = '0') then
                                    bad <= '1';
                                end if;
                                flags <= (others => '0');
                                results <= (others => 0);
                                s <= 0;
                                k <= LOW_BIN(0);
                                peak_bin <= LOW_BIN(0);
                                peak <= (others => '0');
                                state <= SCAN;
                            end if;
                        end if;

		    -- For the current string, find the bin in the necessary range with the highest power (given by range in k and string s)
                    when SCAN =>
                        if powers(k) > peak then
                            peak <= powers(k);
                            peak_bin <= k;
                        end if;
                        if k = HIGH_BIN(s) then
                            state <= SAVE_RESULT;
                        else
                            k <= k + 1;
                        end if;

		    -- If the frame is good, has value exponent, nonzero power, and is in range, then 
		    -- restored is the result that compensates for FFT scaling (note that MIN_FFT_POWER_LOG2 = 32)
		    -- and save the results and pass it
                    when SAVE_RESULT =>
                        if (bad = '0') and (exponent <= 15) and (peak /= 0)
                           and (peak_bin > LOW_BIN(s))
                           and (peak_bin < HIGH_BIN(s)) then
                            restored := shift_left(resize(peak,64),2*exponent);
                            if (restored >= shift_left(to_unsigned(1,64),MIN_FFT_POWER_LOG2)) then
                                results(s) <= peak_bin * 256;
                                flags(s) <= '1';
                            end if;
                        end if;
		    -- If we are on the last string, then we are done, otherwise, check the next string
                        if s = 3 then
                            state <= FINISHED;
                        else
                            s <= s + 1;
                            k <= LOW_BIN(s+1);
                            peak_bin <= LOW_BIN(s+1);
                            peak <= (others => '0');
                            state <= SCAN;
                        end if;
		    
 		    -- When all strings are done, signify that we are done and reset flags and wait to RECEIVE again
                    when FINISHED =>
                        done_o <= '1';
                        sample_count <= 0;
                        bad <= '0';
                        state <= RECEIVE;
                end case;
            end if;
        end if;
    end process;
end architecture;
