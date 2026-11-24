-- This file sends STEP pulses and then stops (also controls directions); instantiate this once for each of the 4 motors
library ieee;
use ieee.std_logic_1164.all;

entity stepper_channel is
  -- HIGH_CYCLES = how long step stays high for (10 us)
  -- PERIOD_CYCLES = time between successive STEP rising edges
  -- DIR_SETUP_CYLES = Delay between setting direction and first STEP pulse
  -- MAX_COUNT = max pulses per command cycle
  generic (HIGH_CYCLES : positive := 120; 
	   PERIOD_CYCLES : positive := 120000;
           DIR_SETUP_CYCLES : positive := 120; 
	   MAX_COUNT : positive := 4);

  --abort_i = cancel movement (stop correction)
  --go_i = Start a correction when idle
  --dir_i = direction for the movement
  --count_i = STEP pulses requested (sent from tuner_control.vhd)
  --dir_o = connects to drivers DIR input
  --busy_o = high when module is executing (i.e. the correction)
  port (
	clk : in std_logic;
	rst : in std_logic;
	abort_i : in std_logic;
	go_i : in std_logic;
	dir_i : in std_logic;
  count_i : in integer;
  step_o : out std_logic;
	dir_o : out std_logic;
	busy_o : out std_logic
	);
end;

architecture rtl of stepper_channel is
  -- Types for an FSM 
  -- IDLE = wait for command
  -- SET_DIR = Allow direction signal to settle
  -- HIGH_PULSE =  hold STEP high
  -- LOW_PULSE = hold STEP low between pulses
  type state_t is (IDLE, SET_DIR, HIGH_PULSE, LOW_PULSE);

  signal state : state_t := IDLE;
  -- Counts down delay required by current state
  signal timer : natural range 0 to PERIOD_CYCLES := 0;

  -- left_count = # of pulses remaining
  signal left_count : natural range 0 to MAX_COUNT := 0;

  --step_r & dir_r store STEP and DIR output respectively
  -- cancelled = remembers an abort request while finishing a HIGH pulse.
  signal step_r : std_logic := '0';
  signal dir_r : std_logic := '0';
  signal cancelled : std_logic := '0';

begin
  step_o<=step_r; 
  dir_o<=dir_r;
  busy_o<='0' when state=IDLE else '1';

  process(clk) begin
    if rising_edge(clk) then
      -- Synchronous reset 
      if (rst='1') then 
	state<=IDLE;
	step_r<='0';
	dir_r<='0';
	timer<=0;
	cancelled<='0';
      else
        -- Finish an already high pulse to its minimum width. Never start another
        -- rising edge after abort. Holding current is controlled separately.
        if (abort_i='1') then	
	  cancelled<='1'; 
	end if;
	-- If we abort and we are not currently sending a pulse, reset (case where we are sending a pulse would put cancelled high anyway)
        if (abort_i='1') and (state/=HIGH_PULSE) then
          state<=IDLE;
	  step_r<='0';
        else
          case state is
  	    -- Capture the direction, and pulses needed to tune
            when IDLE =>
              step_r<='0';
              if (go_i='1') and (count_i>0) and (count_i<=MAX_COUNT) and (abort_i='0') then
                dir_r<=dir_i;
	        left_count<=count_i;
	        timer<=DIR_SETUP_CYCLES-1;
                cancelled<='0';
	        state<=SET_DIR;
              end if;
	    -- Let the direction settle before stepping (small delay)
            when SET_DIR =>
              if (timer=0) then 
		step_r<='1';
		timer<=HIGH_CYCLES-1;
		state<=HIGH_PULSE;
              else 
		timer<=timer-1;
	      end if;

	   -- Keep stepping until either we cancel/abort or the # of pulses for cycle runs out and progress to IDLE state.
           -- Otherwise, decrement the # of pulses and the timers and go to LOW_PULSE state ( high for 10 us)
            when HIGH_PULSE =>
              if (timer=0) then
                step_r<='0';

                if (cancelled='1') or (abort_i='1') or (left_count=1) then
		  state<=IDLE;
                else 
		  left_count<=left_count-1;
		  timer<=PERIOD_CYCLES-HIGH_CYCLES-1;
		  state<=LOW_PULSE;
		end if;
              else 
		timer<=timer-1;
	      end if;
	
	    -- Keep the steps low until the timer runs out (in this case 10 ms)
            when LOW_PULSE =>
              if (timer=0) then 
		step_r<='1';
		timer<=HIGH_CYCLES-1;
		state<=HIGH_PULSE;
              else 
		timer<=timer-1;
	      end if;

          end case;
        end if;
      end if;
    end if;
  end process;
end;
