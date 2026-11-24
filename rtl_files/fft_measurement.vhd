library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.tuner_pkg.all;

entity fft_measurement is
  -- abort_i = cancel the measurement
  -- start_i = request a new measurement while idle
  -- adc_valid_i = indicates a new ADC measurement is available
  -- adc_data_i = ADC results containing the 12-bit sample (4 other bits for filtering; native to the block)
  -- adc_id_i = tells us which XADC channel produced the result
  -- done_o = clock pulse telling us measurement is complete
  -- valid_o = validity flags for each of the strings (ADGB)
  -- pitch_q8_o = estimated pitches in (measured in 1/256 of an FFT bin)
  port(
       clk : in std_logic;
       rst : in std_logic;
       abort_i : in std_logic;
       start_i : in std_logic;
       adc_valid_i : in std_logic;
       adc_data_i : in std_logic_vector(15 downto 0);
       adc_id_i : in std_logic_vector(4 downto 0);
       done_o : out std_logic;
       valid_o : out std_logic_vector(3 downto 0);
       pitch_q8_o : out int4_t
       );
end;

architecture rtl of fft_measurement is
  -- Instantiation according to xfft.vho template
  component xfft_0 is port (
    aclk : in std_logic;
    aresetn : in std_logic;
    s_axis_config_tdata : in std_logic_vector(7 downto 0);
    s_axis_config_tvalid : in std_logic;
    s_axis_config_tready : out std_logic;
    s_axis_data_tdata : in std_logic_vector(31 downto 0);
    s_axis_data_tvalid : in std_logic;
    s_axis_data_tready : out std_logic;
    s_axis_data_tlast : in std_logic;
    m_axis_data_tdata : out std_logic_vector(31 downto 0);
    m_axis_data_tuser : out std_logic_vector(23 downto 0);
    m_axis_data_tvalid : out std_logic;
    m_axis_data_tready : in std_logic;
    m_axis_data_tlast : out std_logic;
    m_axis_status_tdata : out std_logic_vector(7 downto 0);
    m_axis_status_tvalid : out std_logic;
    m_axis_status_tready : in std_logic;
    event_frame_started : out std_logic;
    event_tlast_unexpected : out std_logic;
    event_tlast_missing : out std_logic;
    event_status_channel_halt : out std_logic;
    event_data_in_channel_halt : out std_logic;
    event_data_out_channel_halt : out std_logic
  ); end component;


  -- IDLE =  wait for a measurement request
  -- RESET_FFT =  hold FFT and the estimator in reset
  -- ARM =  discard an XADC conversion and initialize a frame
  -- CAPTURE =  store 4096 signals and evaluate signal quality
  -- CONFIGURE =  send the FFT configuration
  -- READ_SAMPLE = read one stored sample and its window coefficient
  -- PREPARE_SAMPLE = remove any DC offset,scale it, and apply window
  -- SEND_SAMPLE =  hold prepared sample until FFT accepts it 
  -- WAIT_RESULT = wait for pitch estimation to finish
  type state_t is (IDLE, RESET_FFT, ARM, CAPTURE, CONFIGURE, READ_SAMPLE,
                   PREPARE_SAMPLE, SEND_SAMPLE, WAIT_RESULT);

  -- The array of type ram_t stores 4096 samples which are 12 bits wide
  signal state : state_t := IDLE;
  type ram_t is array (0 to FFT_N-1) of unsigned(11 downto 0);
  signal samples : ram_t;

  -- Implement storage using FPGA block RAM
  attribute ram_style : string;
  attribute ram_style of samples : signal is "block";

  -- rd = read sample from memory
  -- wc = the corresponding Hann-window coefficient (i.e. weight)
  signal rd : unsigned(11 downto 0);
  signal wc : unsigned(14 downto 0);
  
  -- index_r =  where to read/write
  signal index_r : natural range 0 to FFT_N-1 := 0;

  -- total = sum samples together
  -- mean = store sample average
  signal reset_count : natural range 0 to 15 := 0;
  signal total : natural range 0 to 16777215 := 0;
  signal mean : natural range 0 to 4095 := 0;
  signal adc_minimum : natural range 0 to 4095 := 4095;
  signal adc_maximum : natural range 0 to 4095 := 0;

  -- clipped = sample was too close to ADC limits 
  -- good = frame passes quality check
  -- gap_bad = sample took long to arrive
  signal clipped : std_logic := '0';
  signal good : std_logic := '0';
  signal gap_bad : std_logic := '0';
  signal sample_gap : natural range 0 to 2*CLK_HZ/SAMPLE_HZ := 0;

  -- Signals to connect to the FFT
  signal fft_rst : std_logic;
  signal fft_resetn : std_logic;
  signal config_valid : std_logic;
  signal config_ready : std_logic;
  signal din : std_logic_vector(31 downto 0) := (others=>'0');
  signal din_valid : std_logic;
  signal din_ready : std_logic;
  signal din_last : std_logic;
  signal dout_valid : std_logic;
  signal dout_last : std_logic;
  signal dout : std_logic_vector(31 downto 0);
  signal user : std_logic_vector(23 downto 0);
  signal estimate_done : std_logic;
  signal missing : std_logic;
  signal unexpected : std_logic;

begin
  fft_rst <= '1' when (rst='1') or (abort_i='1') or (state=IDLE) or (state=RESET_FFT) else '0';
  fft_resetn<=not fft_rst; -- Needed as fft_rst is active high and we need active low
  config_valid<='1' when state=CONFIGURE else '0';
  din_valid<='1' when state=SEND_SAMPLE else '0';
  din_last<='1' when index_r=FFT_N-1 else '0';

  fft: xfft_0 port map(
    aclk=>clk,
    aresetn=>fft_resetn,
    s_axis_config_tdata=>x"01",
    s_axis_config_tvalid=>config_valid,
    s_axis_config_tready=>config_ready,
    s_axis_data_tdata=>din,
    s_axis_data_tvalid=>din_valid,
    s_axis_data_tready=>din_ready,
    s_axis_data_tlast=>din_last,
    m_axis_data_tdata=>dout,
    m_axis_data_tuser=>user,
    m_axis_data_tvalid=>dout_valid,
    m_axis_data_tready=>'1',
    m_axis_data_tlast=>dout_last,
    m_axis_status_tdata=>open,
    m_axis_status_tvalid=>open,
    m_axis_status_tready=>'1',
    event_frame_started=>open,
    event_tlast_unexpected=>unexpected,
    event_tlast_missing=>missing,
    event_status_channel_halt=>open,
    event_data_in_channel_halt=>open,
    event_data_out_channel_halt=>open
    );

  estimator: entity pitch_estimator port map(
    clk=>clk,
    rst=>fft_rst,
    data_i=>dout,
    user_i=>user,
    valid_i=>dout_valid,
    last_i=>dout_last,
    frame_good_i=>good,
    done_o=>estimate_done,
    valid_o=>valid_o,
    pitch_q8_o=>pitch_q8_o
    );

  -- Synchronous RAM and coefficient ROM reads. PREPARE_SAMPLE consumes them
  -- one clock later; SEND_SAMPLE holds the payload stable until accepted.
  -- In other words, this process handles memory access (via VAUX12)

  process(clk) 
  begin
    if rising_edge(clk) then
      if (state=CAPTURE) and (adc_valid_i='1') and (adc_id_i="11100") and (rst='0') and (abort_i='0') then
        samples(index_r)<=unsigned(adc_data_i(15 downto 4));
      end if;

      -- If we need to read the sample, store index and the Hann coefficient
      if (state=READ_SAMPLE) then 
	      rd<=samples(index_r);
	      wc<=HANN(index_r);
      end if;
    end if;
  end process;

  process(clk)
    -- a = Current ADC value in integer form
    -- mn = minimum ADC value
    -- mx = maximum ADC value
    variable a : integer;
    variable mn : integer;
    variable mx : integer;
    variable product : signed(31 downto 0);

  begin
    if rising_edge(clk) then
      done_o<='0';
      if (rst='1') or (abort_i='1') then
        state<=IDLE;
        good<='0';
      else
        if (missing='1') or (unexpected='1')
	        then good<='0';
	end if;
 	-- Offer 3032 clock cycles (i.e. the expected timing + a little extra) for capturing to occur
        if (state=CAPTURE) then
          if (sample_gap >= CLK_HZ/SAMPLE_HZ + 32) then
	          gap_bad<='1';
          else 
	          sample_gap<=sample_gap+1;
	  end if;
        end if;

        case state is
          when IDLE =>
            if (start_i='1') then
              state<=RESET_FFT;
              reset_count<=0;
              good<='0';
            end if;
	
	  -- Spend 16 clock cycles to reset the FFT + estimator
          when RESET_FFT =>
            if (reset_count=15) then 
	            state<=ARM;
	          else 
	            reset_count<=reset_count+1;
	          end if;

	  -- Discard the first sample to make a fresh frame boundary, the next sample is then sample 0
          when ARM =>
            if (adc_valid_i='1') and (adc_id_i="11100") then
              index_r<=0;
              total<=0;
              adc_minimum<=4095;
              adc_maximum<=0;
              clipped<='0';
              gap_bad<='0';
              sample_gap<=0;
              state<=CAPTURE;
            end if;

          when CAPTURE =>
            if (adc_valid_i='1') and (adc_id_i="11100") then
              sample_gap<=0;
	      -- Convert each accepted sample into an integer (other process stores it in RAM)
              a:=to_integer(unsigned(adc_data_i(15 downto 4)));
              total<=total+a;
              mn:=adc_minimum;
	            mx:=adc_maximum;

              if (a<mn) then 
		            mn:=a;
	            end if;

	            if (a>mx) then 
		            mx:=a;
	            end if;

              adc_minimum<=mn;
	            adc_maximum<=mx;
	      
 	      -- CLIP_LOW = 8 and CLIP_HIGH = 4087 (in pkg file)
              if (a<=CLIP_LOW) or (a>=CLIP_HIGH) then
	              clipped<='1';
	            end if;

	      -- On the final sample, it computes the mean and calls sample good if mx-mn is at least 64 ADC counts and there is no clipping or timing gap
              if (index_r=FFT_N-1) then
                mean<=(total+a)/FFT_N;
  	        -- the last two assignments are there to check the last sample
                if (mx-mn>=MIN_ADC_PP) and (clipped='0') and (gap_bad='0') and (a>CLIP_LOW) and (a<CLIP_HIGH) then
	                good<='1';
                else 
		              good<='0';
	              end if;
                index_r<=0;
	              state<=CONFIGURE;
	      -- If it isn't the final sample just increment (i.e. do it for all 4096 samples)
              else 
		            index_r<=index_r+1;
	            end if;
            end if;

	  -- when 4096 samples are taken, this state initiates synchronous sample and coefficient reading
          when CONFIGURE => 
            if (config_ready='1') then
              state<=READ_SAMPLE;
            end if;

          when READ_SAMPLE => 
	          state<=PREPARE_SAMPLE;

	  --to_integer(rd) - mean removes the DC offset and multply by 8 to use more of 16 bit range
	  --signed('0' & wc) is the 16 bit Hann coefficient to remove spectral leakage for each index
          when PREPARE_SAMPLE =>
            product:=to_signed((to_integer(rd)-mean)*8,16)*signed('0'&wc);
	    -- Shift the result back and pack it up (i.e. send to FFT)
            din<=x"0000" & std_logic_vector(resize(shift_right(product,15),16));
            state<=SEND_SAMPLE;

	  -- Read each sample and pack it up, once it is all done then we go to WAIT_RESULT
          when SEND_SAMPLE =>
            if (din_ready='1') then
              if (index_r=FFT_N-1) then 
		            state<=WAIT_RESULT;
              else 
                index_r<=index_r+1;
                state<=READ_SAMPLE;
	            end if;
            end if;

	  -- Finish the measurement, produce output, and send it to pitch_estimator
          when WAIT_RESULT =>
            if (estimate_done='1') then
	            done_o<='1';
	            state<=IDLE;
	          end if;

        end case;
      end if;
    end if;
  end process;
end;
