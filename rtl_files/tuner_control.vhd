library ieee;
use ieee.std_logic_1164.all;
use work.tuner_pkg.all;

entity tuner_control is
  generic(SETTLE_TICKS : positive := SETTLE_CYCLES;
	 -- Limit an individual operation to 3 seconds
          WATCHDOG_TICKS : positive := WATCHDOG_CYCLES;
          SESSION_TICKS : positive := SESSION_CYCLES;
          MAX_SESSION_STEPS : positive := 200);
 	  -- Limit total commanded movement during attempt to tune

  port(
       -- measurement_done_i = signal that tells us when a new set of frequency estimates are available
       -- valid_i(i) = logic vector that tells us whether string i has an estimate
       -- pitch_q8_i(i) = estimated pitch in units of 1/256 FFT bins
       clk : in std_logic;
       rst : in std_logic;
       start_i : in std_logic;
       stop_i : in std_logic;
       measurement_done_i : in std_logic;
       valid_i : in std_logic_vector(3 downto 0);
       pitch_q8_i : in int4_t;
       motor_busy_i : in std_logic_vector(3 downto 0);

       -- motor_go_o, motor_dir_o, and motor_count_o go to the motor VHD file, and that file returns us motor_busy_i
       -- running_o = tuning is underway
       -- done_o = tuning finished successfully
       -- fault_o = something caused tuning to stop
       -- abort_o = cancel measurements/motor
       measure_start_o : out std_logic;
       abort_o : out std_logic;
       motor_go_o : out std_logic;
       motor_dir_o : out std_logic_vector(3 downto 0);
       motor_count_o : out int4_t;
       green_o : out std_logic_vector(3 downto 0);
       red_o : out std_logic_vector(3 downto 0);
       white_o : out std_logic_vector(3 downto 0);
       running_o : out std_logic_vector(3 downto 0);
       done_o : out std_logic_vector(3 downto 0);
       fault_o : out std_logic
       );
end;

architecture rtl of tuner_control is

  -- Custom type for the FSM that controls tuning
  -- IDLE = wait for START 
  -- REQUEST_FRAME = request one fresh measurement set
  -- WAIT_FRAME = wait and compute results
  -- LAUNCH = drive motors
  -- LAUNCH_WAIT = time for motor modules to register the commands
  -- MOVING -- wait until all motors finish
  -- SETTLING = wait for the physical mechanics to settle
  -- DONE = hold sucessful complete
  -- STOPPED = hold a user stop/fault
  type state_t is (IDLE,REQUEST_FRAME,WAIT_FRAME,LAUNCH,LAUNCH_WAIT,MOVING,SETTLING,DONE,STOPPED);

  -- settle_count = counter to measure time spent in SETTLING state
  -- watchdog = time waiting for measurement or movement (i.e. prevents tuner from getting stuck/waiting forever)
  -- session_count = counter to measure total active time since START (can create a tuning abortion)
  -- streak = # that measures consecutive measurements with all four strings in tune
  -- counts(i) = Request pulse for string i in next correction (****************************** PROBABLY NEEDS EDITS)
  -- totals(i) = total pulses commanded for string i in session
  -- dirs(i) = direction for next correction

  signal state : state_t := IDLE;
  signal settle_count : natural range 0 to SETTLE_TICKS-1 := 0;
  signal watchdog : natural range 0 to WATCHDOG_TICKS-1 := 0;
  signal session_count : natural range 0 to SESSION_TICKS-1 := 0;
  signal streak : natural range 0 to CONFIRM_FRAMES := 0;
  signal counts : int4_t := (others => 0);
  signal totals : int4_t := (others => 0);
  signal dirs,green,red,white : std_logic_vector(3 downto 0) := (others=>'0');
  signal fault : std_logic := '0';
  signal active : std_logic := '0';

begin
  -- Drive outputs based on state and/or defined signals
  active<='1' when (state/=IDLE) and (state/=DONE) and (state/=STOPPED) else '0';
  running_o<=active;

  done_o<='1' when (state=DONE) else '0';
  fault_o<=fault;
  abort_o<='1' when (stop_i='1') or (state=STOPPED) or (rst='1') else '0';
  measure_start_o<='1'
    when (state=REQUEST_FRAME) and (stop_i='0') 
    else '0';
  motor_go_o<='1' 
    when (state=LAUNCH) and (stop_i='0') 
    else '0';
  motor_count_o<=counts;
  motor_dir_o<=dirs;
  green_o<=green;
  red_o<=red;
  white_o<=white;

  process(clk)
    -- error = input frequency - target
    -- error_mag = absolute value of error
    -- n = requested pulses for given error (need 1-4 pulses) (NEEDS EDITS PROBABLY, DRIVES M0, M1, M2!!!!!! **************************)
    -- all_ok = every string is valid and within tolerance
    -- any_move = at least one string needs suning still
    -- limit_hit = proposed correction woudl exceed max movement parameters
    variable error : integer;
    variable error_mag : integer;
    variable n : integer;
    variable all_ok : boolean;
    variable any_move : boolean;
    variable limit_hit : boolean;
 -- The process takes reset as first priority, stop as 2nd, and then if the tuning is going on too long, that is the third priority. Then FSM runs
  begin
    if rising_edge(clk) then
      if rst='1' then
        state<=IDLE;
        green<=(others=>'0');
        red<=(others=>'0');
        white<=(others=>'0');
        counts<=(others=>0);
        totals<=(others=>0);
        streak<=0;
        fault<='0';
        session_count<=0;
        watchdog<=0;
      elsif stop_i='1' then
        state<=STOPPED;
        counts<=(others=>0); -- deliberately do NOT assign LED (hold color)
      elsif (active='1') and (session_count= SESSION_TICKS-1) then
        state<=STOPPED;
        counts<=(others=>0);
        fault<='1';
      else
        -- Increase session time
        if (active='1') then 
          session_count<=session_count + 1;
        end if;
        -- when the state is either IDLE, DONE, or STOPPED, they have some equivalent behavior (in terms of state). Note that this ignores a new START signal during an active tuning attempt.
        case state is
          when IDLE | DONE | STOPPED => 
            if start_i='1' then
              state<=REQUEST_FRAME;
              green<=(others=>'0');
              red<=(others=>'0');
              white<=(others=>'0');
              counts<=(others=>0);
              totals<=(others=>0);
              streak<=0;
              fault<='0';
              session_count<=0;
            end if;

          when REQUEST_FRAME => 
	    watchdog<=0;
	    state<=WAIT_FRAME;

          when WAIT_FRAME =>
            if (measurement_done_i='1') then
              all_ok:=true;
	      any_move:=false;
              limit_hit:=false;

	      -- Check each string; initialize for no movement/LED
              for i in 0 to 3 loop
                n:=0;
	        green(i)<='0';
                red(i)<='0';
                white(i)<='0';
		--Invalid string keeps zero requested pulses; LEDs off; completion will be disallowed
                if (valid_i(i)='0') then
		  all_ok:=false;
                else
		  -- Compute error based on the input pitch and target
                  error:=pitch_q8_i(i)-TARGET_Q8(i);

		  -- Negative = flat, positive = sharp, ~0 = in tune
		  -- Compute absolute value of the error (drive motor)
                  if (error<0) then
	            error_mag:=-error;
	          else 
		    error_mag:=error;
                  end if;

		  -- If error is less than tolerance, light up LED
                  if (error_mag<=TOL_Q8(i)) then 
		    green(i)<='1';
                  else
                    all_ok:=false;
		    -- Modify tuning direction + LEDs based on error 
                    if (error<0) then 
		      red(i)<='1';
                      dirs(i)<=TIGHTEN_DIR(i);
                    else 
                      white(i)<='1';
                      dirs(i)<=not TIGHTEN_DIR(i);
                    end if;

                    if error_mag>1049 then n:=MAX_STEPS; -- >~4 Hz
                    elsif error_mag>262 then n:=2;else n:=1;end if;
                  end if;
                end if;
                if n>MAX_STEPS then n:=MAX_STEPS;end if;
                counts(i)<=n;
                if n>0 then any_move:=true;end if;
                if totals(i)+n>MAX_SESSION_STEPS then limit_hit:=true;end if;
              end loop;
	-- If all four strings are valid & in tune, see if we have confirmed this behavior for enough cycles. Otherwise (in watchdog elsif),  
              if (all_ok) then
                if streak=CONFIRM_FRAMES-1 then 				          state<=DONE;
		  streak<=CONFIRM_FRAMES;
                else 
		  streak<=streak+1;
		  state<=REQUEST_FRAME;
		end if;
              else
                streak<=0;
		-- Exceed max movements, then immediately stop, otherwise then launch the FSM again
                if (limit_hit) then 				   			  state<=STOPPED;
		  fault<='1';
		  counts<=(others=>0);
                elsif (any_move) then 
		  state<=LAUNCH;
                else 
		  state<=REQUEST_FRAME;
		end if;
              end if;
            elsif watchdog=WATCHDOG_TICKS-1 then 			   	      state<=STOPPED;
	      fault<='1';
	      counts<=(others=>0);
            else 
	      watchdog<=watchdog+1;
	    end if;
          -- In this state, motor controllers accept commands and update the total commands given
          when LAUNCH =>
            for i in 0 to 3 loop 
	      totals(i)<=totals(i)+counts(i);
	    end loop;
            state<=LAUNCH_WAIT;
          -- Give time between command and checking completion
          when LAUNCH_WAIT => 
	    state<=MOVING;
	    watchdog<=0;
          -- If motors are idle then allow it to settle, otherwise wait and increment watchdog
          when MOVING =>
            if motor_busy_i="0000" then 
	      settle_count<=0;
	      state<=SETTLING;
            elsif (watchdog=WATCHDOG_TICKS-1) then           	   	  	      state<=STOPPED;
	      fault<='1';
            else 
	      watchdog<=watchdog+1;
	    end if;
	 -- Allow system to settle before another measurement (200ms)
          when SETTLING =>
            if (settle_count=SETTLE_TICKS-1) then 
	      state<=REQUEST_FRAME;
            else 
	      settle_count<=settle_count+1;
            end if;
        end case;
      end if;
    end if;
  end process;
end;
