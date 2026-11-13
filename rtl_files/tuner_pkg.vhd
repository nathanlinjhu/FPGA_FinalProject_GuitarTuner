library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

package tuner_pkg is
  constant CLK_HZ : positive := 12000000;
  constant SAMPLE_HZ : positive := 4000; -- Sample frequency
  constant FFT_N : positive := 4096;

  -- Custom type that holds info for each of the strings
  -- Indices: 0=A2, 1=D3, 2=G3, 3=B3
  type int4_t is array (0 to 3) of integer;

  --TARGET_Q8 was found by string target frequency, times 4096/4000 times 256 and then rounded to try and eliminate fractions
  constant TARGET_Q8 : int4_t := (28836,38491,51380,64734);
  constant TOL_Q8 : int4_t := (83,111,149,187); -- approximately +/-5 cents of tolerance

  -- Bins that will be accepted for the tuning range of each string
  constant LOW_BIN : int4_t := (103,141,191,243);
  constant HIGH_BIN : int4_t := (122,160,210,263);

 --MAX_STEPS is times STEP can change in a cycle, CONFIRM_FRAMES is how many times to check in tune before stopping
  constant MAX_STEPS : positive := 4;
  constant CONFIRM_FRAMES : positive := 3;

  -- MIN_FFT_POWER_LOG2 is minimum power (real^2 +imag^2)
  constant MIN_FFT_POWER_LOG2 : natural := 32; -- unscaled power threshold = 2^32
  constant MIN_ADC_PP : natural := 64; -- tune to pickup gain / measured noise
  constant CLIP_LOW : natural := 8;
  constant CLIP_HIGH : natural := 4087; --ADC has 0 to 4096, only take values between 8 and 4087

  --Settling times and timeout
  constant SETTLE_CYCLES : positive := CLK_HZ/5; -- 200 ms
  constant WATCHDOG_CYCLES : positive := CLK_HZ*3; -- Stop if measurement fails
  constant SESSION_CYCLES : positive := CLK_HZ*120; -- 2 minutes maximum per START


  -- Flip individual bits after testing unloaded motors. STEP drivers use external power.
  constant TIGHTEN_DIR : std_logic_vector(3 downto 0) := "1111";

  -- Custom data type that for each FFT bin stores its corresponding Hann number (smooth out signals)
  type hann_t is array (0 to FFT_N-1) of unsigned(14 downto 0);
  constant HANN : hann_t; -- Deferred constant declaration
end package;

package body tuner_pkg is
  -- Called only to initialize a constant at elaboration/synthesis time.
  -- No real-number or cosine hardware is created (note use of IEEE math library)
  function make_hann return hann_t is
    variable coefficients : hann_t; -- What we will be returning
    variable value : real; -- Raw Hann number without rounding
    variable quantized : integer; -- "value" after rounding
  begin
    for i in coefficients'range loop
      value := 32767.0 * 0.5 *
               (1.0 - cos(2.0 * MATH_PI * real(i) / real(FFT_N - 1)));
      quantized := integer(floor(value + 0.5));

      -- Restrict numerical roundoff to the unsigned 15-bit coefficient range.
      if quantized < 0 then
        quantized := 0;
      elsif quantized > 32767 then
        quantized := 32767;
      end if;
      coefficients(i) := to_unsigned(quantized, 15);
    end loop;
    return coefficients;
  end function;
  constant HANN : hann_t := make_hann; -- Note the deferred declaration above
end package body;
