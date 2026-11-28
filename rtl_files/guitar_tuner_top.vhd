library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.tuner_pkg.all;

entity guitar_tuner_top is
  port(
       clk : in std_logic; -- 12 MHz clock
       btn : in std_logic_vector(1 downto 0); -- 0 START, 1 STOP
       vauxp12 : in std_logic;
       vauxn12 : in std_logic;
       step_o : out std_logic_vector(3 downto 0);
       dir_o : out std_logic_vector(3 downto 0);
       green_o : out std_logic_vector(3 downto 0);
       red_o: out std_logic_vector(3 downto 0);
       running_o : out std_logic;
       done_o : out std_logic;
       fault_o : out std_logic
       );
end;

architecture rtl of guitar_tuner_top is
  component xadc_wiz_0 is port (
    m_axis_tvalid : out std_logic;
    m_axis_tready : in std_logic;
    m_axis_tdata : out std_logic_vector(15 downto 0);
    m_axis_tid : out std_logic_vector(4 downto 0);
    m_axis_aclk : in std_logic;
    s_axis_aclk : in std_logic;
    m_axis_resetn : in std_logic;
    convst_in : in std_logic;
    vp_in : in std_logic;
    vn_in : in std_logic;
    channel_out : out std_logic_vector(4 downto 0);
    eoc_out : out std_logic;
    alarm_out : out std_logic;
    eos_out : out std_logic;
    busy_out : out std_logic;
    vauxp12 : in std_logic;
    vauxn12 : in std_logic
  ); 
end component;

  signal por_count : unsigned(7 downto 0) := (others=>'0');
  signal rst : std_logic := '1';
  signal convst : std_logic := '1';
  signal sample_timer : natural range 0 to CLK_HZ/SAMPLE_HZ-1 := 0;
  signal adc_valid : std_logic;
  signal adc_data : std_logic_vector(15 downto 0);
  signal adc_id : std_logic_vector(4 downto 0);
  signal start_press : std_logic;
  signal stop_press : std_logic;
  signal measure_start : std_logic;
  signal measurement_done : std_logic;
  signal abort_all : std_logic;
  signal motor_go : std_logic;
  signal valid : std_logic_vector(3 downto 0);
  signal motor_busy : std_logic_vector(3 downto 0);
  signal motor_dir : std_logic_vector(3 downto 0);
  signal pitch : int4_t;
  signal counts : int4_t;
  signal adc_resetn : std_logic;

begin
  adc_resetn <= not rst;
  -- Artix-7 initialization supplies a power-on reset without using either button.
  rst<='1' when por_count/=255 else '0';
  process(clk) 
  begin
    if rising_edge(clk) then
      if (por_count /= 255) then 
	por_count<=por_count+1;
      end if;

      convst<='0';

      -- Trigger XADC conversion every 250 us, 4000 samples/second
      if (rst='1') then
	sample_timer<=0;
      elsif (sample_timer=CLK_HZ/SAMPLE_HZ-1) then
        sample_timer<=0;
	convst<='1';
      else 
	sample_timer<=sample_timer+1;
      end if;
    end if;
  end process;

  adc: xadc_wiz_0 port map(
    m_axis_aclk=>clk,
    s_axis_aclk=>clk,
    m_axis_resetn=>adc_resetn,
    m_axis_tvalid=>adc_valid,
    m_axis_tready=>'1',
    m_axis_tdata=>adc_data,
    m_axis_tid=>adc_id,
    convst_in=>convst,
    vp_in=>'0',
    vn_in=>'0',
    vauxp12=>vauxp12,
    vauxn12=>vauxn12,
    channel_out=>open,
    eoc_out=>open,
    alarm_out=>open,
    eos_out=>open,
    busy_out=>open
    );

  buttons: entity work.button_inputs port map(
    clk,
    rst,
    btn(0),
    btn(1),
    start_press,
    stop_press
  );

  measurement: entity work.fft_measurement port map(
    clk,
    rst,
    abort_all,
    measure_start,
    adc_valid,
    adc_data,
    adc_id,
    measurement_done,
    valid,
    pitch
   );

  control: entity work.tuner_control port map(
    clk=>clk,
    rst=>rst,
    start_i=>start_press,
    stop_i=>stop_press,
    measurement_done_i=>measurement_done,
    valid_i=>valid,
    pitch_q8_i=>pitch,
    motor_busy_i=>motor_busy,
    measure_start_o=>measure_start,
    abort_o=>abort_all,
    motor_go_o=>motor_go,
    motor_dir_o=>motor_dir,
    motor_count_o=>counts,
    green_o=>green_o,
    red_o=>red_o,
    running_o=>running_o,
    done_o=>done_o,
    fault_o=>fault_o
  );

  motors: for i in 0 to 3 generate
    driver: entity work.stepper_channel
      generic map(MAX_COUNT=>MAX_STEPS)
      port map(
        clk,
        rst,
	abort_all,
	motor_go,
	motor_dir(i),
	counts(i),
	step_o(i),
	dir_o(i),
	motor_busy(i)
      );
  end generate;
end;
