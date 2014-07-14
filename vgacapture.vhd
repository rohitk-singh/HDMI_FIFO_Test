library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.NUMERIC_STD.all;
use IEEE.STD_LOGIC_UNSIGNED.all;

library UNISIM;
use UNISIM.VComponents.all;


entity VGA_Capture is
	port(
		rst         : in std_logic;
		
		DATACK		: in std_logic;	-- 65MHz Pixel clock from AD9984A
		HSOUT			: in std_logic;
		VSOUT			: in std_logic;
		SOGOUT      : in std_logic;
		
		R_in			: in std_logic_vector(7 downto 0);
		G_in			: in std_logic_vector(7 downto 0);
		B_in			: in std_logic_vector(7 downto 0);
		
		img_pclk		: in std_logic;	--65MHz Pixel clock from DCM and PLL, different from DATACK
		SW          : in std_logic_vector ( 3 downto 0);
		debug_out   : out std_logic_vector(15 downto 0);
		
		de				: out std_logic;
		HS  			: out std_logic;
		VS 			: out std_logic;
			
		Red_out		: out std_logic_vector(7 downto 0) ;
		Green_out	: out std_logic_vector(7 downto 0);
		Blue_out		: out std_logic_vector(7 downto 0) 
		
		);
end VGA_Capture;
 
architecture Behavioral of VGA_Capture is

-- All type declarations
type write_states is (state_reset, state_wait_for_hsout, state_backporch, state_active, state_frontporch);
type read_hStates is (state_hReset, state_hSyncPulse, state_hBackPorch, state_hActive, state_hFrontPorch);
type read_vStates is (state_vReset, state_vSyncPulse, state_vBackPorch, state_vActive, state_vFrontPorch);

type vStates is (vSP, vBP, vAct, vFP, vUnknown);
type hStates is (hSP, hBP, hAct, hFP, hUnknown);


-- All Constants
constant resX	: integer	:= 1024;
constant resY	: integer 	:= 768;
constant spY 	: integer	:= 6;
constant bpY 	: integer	:= 29;
constant fpY 	: integer	:= 3;
constant spX 	: integer	:= 136;
constant bpX 	: integer	:= 160;
constant fpX 	: integer	:= 24;

-- Signals for dataSyncFifo
signal fifo_rst           : std_logic := '0';
signal fifo_wr_clk        : std_logic := '0';
signal fifo_rd_clk        : std_logic := '0';
signal fifo_din           : std_logic_vector(23 downto 0) := (others => '0');
signal fifo_wr_en         : std_logic := '0';
signal fifo_rd_en         : std_logic := '0';
signal fifo_dout          : std_logic_vector(23 downto 0) := (others => '0');
signal fifo_full          : std_logic := '0';
signal fifo_almost_full   : std_logic := '0';
signal fifo_wr_ack        : std_logic := '0';
signal fifo_overflow      : std_logic := '0';
signal fifo_empty         : std_logic := '0';
signal fifo_almost_empty  : std_logic := '0';
signal fifo_valid         : std_logic := '0';
signal fifo_underflow     : std_logic := '0';
signal fifo_rd_data_count : std_logic_vector(13 downto 0) := (others => '0'); -- Read Data counter is 14-bits wide
signal fifo_wr_data_count : std_logic_vector(13 downto 0) := (others => '0'); -- Written Data counter is 14-bits wide

signal active_read_fifo  : std_logic := '0';
signal active_write_fifo : std_logic := '0';

-- Signals for dataSyncFifo0
signal fifo_rst0           : std_logic := '0';
signal fifo_wr_clk0        : std_logic := '0';
signal fifo_rd_clk0        : std_logic := '0';
signal fifo_din0           : std_logic_vector(23 downto 0) := (others => '0');
signal fifo_wr_en0         : std_logic := '0';
signal fifo_rd_en0         : std_logic := '0';
signal fifo_dout0          : std_logic_vector(23 downto 0) := (others => '0');
signal fifo_full0          : std_logic := '0';
signal fifo_almost_full0   : std_logic := '0';
signal fifo_wr_ack0        : std_logic := '0';
signal fifo_overflow0      : std_logic := '0';
signal fifo_empty0         : std_logic := '0';
signal fifo_almost_empty0  : std_logic := '0';
signal fifo_valid0         : std_logic := '0';
signal fifo_underflow0     : std_logic := '0';
signal fifo_rd_data_count0 : std_logic_vector(13 downto 0) := (others => '0'); -- Read Data counter is 14-bits wide
signal fifo_wr_data_count0 : std_logic_vector(13 downto 0) := (others => '0'); -- Written Data counter is 14-bits wide

-- Signals for dataSyncFifo1
signal fifo_rst1           : std_logic := '0';
signal fifo_wr_clk1        : std_logic := '0';
signal fifo_rd_clk1        : std_logic := '0';
signal fifo_din1           : std_logic_vector(23 downto 0) := (others => '0');
signal fifo_wr_en1         : std_logic := '0';
signal fifo_rd_en1         : std_logic := '0';
signal fifo_dout1          : std_logic_vector(23 downto 0) := (others => '0');
signal fifo_full1          : std_logic := '0';
signal fifo_almost_full1   : std_logic := '0';
signal fifo_wr_ack1        : std_logic := '0';
signal fifo_overflow1      : std_logic := '0';
signal fifo_empty1         : std_logic := '0';
signal fifo_almost_empty1  : std_logic := '0';
signal fifo_valid1         : std_logic := '0';
signal fifo_underflow1     : std_logic := '0';
signal fifo_rd_data_count1 : std_logic_vector(13 downto 0) := (others => '0'); -- Read Data counter is 14-bits wide
signal fifo_wr_data_count1 : std_logic_vector(13 downto 0) := (others => '0'); -- Written Data counter is 14-bits wide


--Signals for write_fifo process
signal write_state       : write_states := state_reset;
signal hsout_q           : std_logic := '0';
signal vsout_q           : std_logic := '0';
signal hsout_rising_edge : std_logic := '0';
signal vsout_rising_edge : std_logic := '0';

-- Signals for read_fifo process
signal state_hRead : read_hStates := state_hReset;
signal state_vRead : read_vStates := state_vReset; 
signal patanahi   : std_logic;
signal hsout_delayed_rising_edge : std_logic;
signal vsout_delayed_rising_edge : std_logic;
signal vsout_delayed             : std_logic;
signal hsout_delayed             : std_logic;


-- Video Signals
signal rgb_out  : std_logic_vector(23 downto 0) := (others => '0');
signal vSync    : std_logic := '0';
signal hSync    : std_logic := '0';
signal hActive  : std_logic := '0';
signal vActive  : std_logic := '0';
signal active   : std_logic := '0';
signal vStateWr : vStates   := vUnknown;
signal hStateWr : hStates   := hUnknown;
signal vStateRd : vStates   := vUnknown;
signal hStateRd : hStates   := hUnknown;

-- Counters
signal hCounterWr, hCounterRd : integer  := 0;
signal vCounterWr, vCounterRd : integer  := 0;

begin

vStateWr <= vSP     when ( vCounterWr >= 0) and (vCounterWr < spY) else
            vBP     when ( vCounterWr >= spY ) and (vCounterWr < (spY + bpY)) else
			   vAct    when ( vCounterWr >= (spY + bpY)) and (vCounterWr < (spY + bpY + resY)) else
			   vFP     when ( vCounterWr >= (spY + bpY + resY)) and (vCounterWr < (spy + bpY + resY + fpY)) else
			   vUnknown;

hStateWr <= hSP     when ( hCounterWr >= 0) and (hCounterWr < spX) else
            hBP     when ( hCounterWr >= spX) and (hCounterWr < (spX + bpX)) else
			   hAct    when ( hCounterWr >= (spX + bpX)) and (hCounterWr < (spX + bpX + resX)) else
			   hFP     when ( hCounterWr >= (spX + bpX + resX)) and (hCounterWr < (spX + bpX + resX + fpX)) else
			   hUnknown;

vStateRd <= vSP     when ( vCounterRd >= 0) and (vCounterRd < spY) else
            vBP     when ( vCounterRd >= spY ) and (vCounterRd < (spY + bpY)) else
			   vAct    when ( vCounterRd >= (spY + bpY)) and (vCounterRd < (spY + bpY + resY)) else
			   vFP     when ( vCounterRd >= (spY + bpY + resY)) and (vCounterRd < (spy + bpY + resY + fpY)) else
			   vUnknown;

hStateRd <= hSP     when ( hCounterRd >= 0) and (hCounterRd < spX) else
            hBP     when ( hCounterRd >= spX) and (hCounterRd < (spX + bpX)) else
			   hAct    when ( hCounterRd >= (spX + bpX)) and (hCounterRd < (spX + bpX + resX)) else
			   hFP     when ( hCounterRd >= (spX + bpX + resX)) and (hCounterRd < (spX + bpX + resX + fpX)) else
			   hUnknown;
Red_out   <= rgb_out(23 downto 16);
Green_out <= rgb_out(15 downto 8);
Blue_out  <= rgb_out(7 downto 0);
VS <= vSync;
HS <= hSync;
de <= active;
active <= hActive and vActive;

--Fifo
fifo_wr_clk <= DATACK;
fifo_rd_clk <= img_pclk;

-- Fifo_mux

			 
write_fifo: process(rst, DATACK)
begin
	
if rst = '1' then
	write_state <= state_reset;

elsif rising_edge(DATACK) then
	
	hsout_rising_edge <= ((hsout xor hsout_q) and hsout) ;
	hsout_q <= hsout;
	
	vsout_rising_edge <= ((vsout xor vsout_q) and vsout) ;
	vsout_q <= vsout;
	
	if vsout_rising_edge = '1' then
		write_state <= state_reset;
		vCounterWr <= 0;
		hCounterWr <= 0;
		fifo_wr_en <= '0';
				
	elsif vsout_rising_edge = '0' then
		
		case write_state is
			when state_reset => 
				--fifo_rst <= '1';
				--fifo_wr_en <= '0';
				hCounterWr <= 0;
				write_state <= state_wait_for_hsout;
				
			when state_wait_for_hsout =>
				fifo_rst <= '0';
				if hsout_rising_edge = '1' then
					hCounterWr <= 0;
					write_state <= state_backporch;
				end if;
				
			when state_backporch =>
				--increment hCounterWr
				hCounterWr <= hCounterWr + 1;
				if hCOunterWr = (200 - 1) then --This block moved from state_reset
					fifo_rst <= '1';        --Reset the FIFO when 200 cycles over. VSOUT delayed by 100 cycles
				   fifo_wr_en <= '0';
				else
					fifo_rst <= '0';
				end if;
				
				if hCounterWr = (136 + 160 - 1) then
					write_state <= state_active;
					fifo_wr_en <= '1';
					fifo_rst <= '0';
					fifo_din <= R_in & G_in & B_in;
				end if;
				
			when state_active =>
				--increment hCounterWr
				fifo_din <= R_in & G_in & B_in;
				hCounterWr <= hCounterWr + 1;
				if hCounterWr = (136 + 160 + 1024 - 1) then
					write_state <= state_frontporch;
					fifo_wr_en <= '0';
					fifo_din <= (others => '0');
				end if;
				
			when state_frontporch =>
				--increment hCounterWr
				-- And, do nothing else
				hCounterWr <= hCounterWr + 1;
				if hCounterWr > (136 + 160 + 1024 + 20 - 1) then
					fifo_wr_en <= '0';
					fifo_din <= (others => '0');
					write_state <= state_reset; --Reset FIFO after making sure 
				end if;
		
		end case;
	
	end if;
end if;
end process;

-- Entities of HSOUT, VSOUT delayer
-- Entities of HSOUT, VSOUT rising_edge detector
vsout_delayer: entity work.delayer PORT MAP(input => vsout, output => vsout_delayed, clk => img_pclk); --Try also img_clk
hsout_delayer: entity work.delayer PORT MAP(input => hsout, output => hsout_delayed, clk => DATACK); --Try also img_clk
vs_delayed_edge_detect: entity work.rising_edge_detector PORT MAP(CLK => img_pclk, SIGNAL_IN => vsout_delayed, OUTPUT => vsout_delayed_rising_edge);

read_fifo: process(rst, img_pclk)
begin

if rst = '1' then
	state_vRead <= state_vReset;
	state_hRead <= state_hReset;
	
elsif rising_edge(img_pclk) then
	
	if vsout_delayed_rising_edge = '1' then
		state_vRead <= state_vReset;
		vCounterRd <= 0;
		fifo_rd_en <= '0';
		hCounterRd <= 0;
		--startvCounterRd
	end if;
	
	case state_vRead is 
		
		when state_vReset =>
			vCounterRd <= 0;
			hCounterRd <= 0;
			vSync <= '1';
			vActive <= '0';
			state_vRead <= state_vSyncPulse;
			state_hRead <= state_hReset;
			
		when state_vSyncPulse =>
			vSync <= '1';
			case state_hRead is 
				when state_hReset      => hCounterRd <= 0;
											     hSync <= '1';
											     hActive <= '0';
												  fifo_rd_en <= '0';
											     state_hRead <= state_hSyncPulse;
				when state_hSyncPulse  => hSync <= '1';
												  hCounterRd <= hCounterRd + 1;
												  if hCounterRd = (spX - 1) then
												     hSync <= '0';
													  state_hRead <= state_hBackPorch;
												  end if;
				when state_hBackPorch  => hSync <= '0';
												  hCounterRd <= hCounterRd + 1;
												  if hCounterRd = (spX + bpX - 1) then
													  hActive <= '1';
													  state_hRead <= state_hActive;
													  fifo_rd_en <= '1';
													  rgb_out <= fifo_dout;
												  end if;		
				
				when state_hActive     => hActive <= '1';
												  hCounterRd <= hCounterRd + 1;
												  rgb_out <= fifo_dout;
												  fifo_rd_en <= '1';
												  if hCounterRd = (spX + bpX + resX - 1) then
													  hActive <= '0';
													  state_hRead <= state_hFrontPorch;
													  fifo_rd_en <= '0';
													  rgb_out <= (others => '0');
												  end if;
				
				when state_hFrontPorch => hActive <= '0';
												  hCounterRd <= hCounterRd + 1;
												  fifo_rd_en <= '0';
												  rgb_out <= (others => '0');
												  if hCounterRd = (spX + bpX + resX + fpX - 1) then
													  state_hRead <= state_hReset;
													  vCounterRd <= vCounterRd + 1;
												  end if;
			end case;
			if vCounterRd = (spY - 1) then
				state_vRead <= state_vBackPorch;
				state_hRead <= state_hReset;
				vSync <= '0';
			end if;
				
		when state_vBackPorch =>
			vSync <= '0';
			case state_hRead is 
				when state_hReset      => hCounterRd <= 0;
											     hSync <= '1';
											     hActive <= '0';
												  fifo_rd_en <= '0';
											     state_hRead <= state_hSyncPulse;
				when state_hSyncPulse  => hSync <= '1';
												  hCounterRd <= hCounterRd + 1;
												  if hCounterRd = (spX - 1) then
												     hSync <= '0';
													  state_hRead <= state_hBackPorch;
												  end if;
				when state_hBackPorch  => hSync <= '0';
												  hCounterRd <= hCounterRd + 1;
												  if hCounterRd = (spX + bpX - 1) then
													  hActive <= '1';
													  state_hRead <= state_hActive;
													  fifo_rd_en <= '1';
													  rgb_out <= fifo_dout;
												  end if;		
				
				when state_hActive     => hActive <= '1';
												  hCounterRd <= hCounterRd + 1;
												  rgb_out <= fifo_dout;
													fifo_rd_en <= '1';
												  if hCounterRd = (spX + bpX + resX - 1) then
													  hActive <= '0';
													  state_hRead <= state_hFrontPorch;
													  fifo_rd_en <= '0';
													  rgb_out <= (others => '0');
												  end if;
				
				when state_hFrontPorch => hActive <= '0';
												  hCounterRd <= hCounterRd + 1;
												  fifo_rd_en <= '0';
													rgb_out <= (others => '0');
												  if hCounterRd = (spX + bpX + resX + fpX - 1) then
													  state_hRead <= state_hReset;
													  vCounterRd <= vCounterRd + 1;
												  end if;
			end case;
			if vCounterRd = (spY + bpY - 1) then
				state_vRead <= state_vActive;
				state_hRead <= state_hReset;
				vActive <= '1';
			end if;
			
		when state_vActive =>
			vActive <= '1';
			case state_hRead is 
				when state_hReset      =>	hCounterRd <= 0;
													hSync <= '1';
													hActive <= '0';
													fifo_rd_en <= '0';
													state_hRead <= state_hSyncPulse;
				
				when state_hSyncPulse  =>	hSync <= '1';
													hCounterRd <= hCounterRd + 1;
													if hCounterRd = (spX - 1) then
														hSync <= '0';
														state_hRead <= state_hBackPorch;
													end if;
				
				when state_hBackPorch  =>	hSync <= '0';
													hCounterRd <= hCounterRd + 1;
													if hCounterRd = (spX + bpX - 1) then
														hActive <= '1';
														state_hRead <= state_hActive;
														fifo_rd_en <= '1';
														rgb_out <= fifo_dout;
													end if;
				
				when state_hActive     =>	hActive <= '1';
													hCounterRd <= hCounterRd + 1;
													rgb_out <= fifo_dout;
													fifo_rd_en <= '1';
													if hCounterRd = (spX + bpX + resX - 1) then
														hActive <= '0';
														state_hRead <= state_hFrontPorch;
														fifo_rd_en <= '0';
														rgb_out <= (others => '0');
													end if; 
				
				when state_hFrontPorch =>	hActive <= '0';
													hCounterRd <= hCounterRd + 1;
													fifo_rd_en <= '0';
													rgb_out <= (others => '0');
													if hCounterRd = (spX + bpX + resX + fpX - 1) then
														state_hRead <= state_hReset;
														vCounterRd <= vCounterRd + 1;
													end if;
			end case;
			if vCounterRd = (spY + bpY + resY - 1) then
				state_vRead <= state_vFrontPorch;
				state_hRead <= state_hReset;
				vActive <= '0';
			end if;
			
		when state_vFrontPorch =>
			vActive <= '0';
			case state_hRead is 
				when state_hReset      => hCounterRd <= 0;
											     hSync <= '1';
											     hActive <= '0';
												  fifo_rd_en <= '0';
											     state_hRead <= state_hSyncPulse;
				when state_hSyncPulse  => hSync <= '1';
												  hCounterRd <= hCounterRd + 1;
												  if hCounterRd = (spX - 1) then
												     hSync <= '0';
													  state_hRead <= state_hBackPorch;
												  end if;
				when state_hBackPorch  => hSync <= '0';
												  hCounterRd <= hCounterRd + 1;
												  if hCounterRd = (spX + bpX - 1) then
													  hActive <= '1';
													  state_hRead <= state_hActive;
													  fifo_rd_en <= '1';
													  rgb_out <= fifo_dout;
												  end if;		
				
				when state_hActive     => hActive <= '1';
												  hCounterRd <= hCounterRd + 1;
												  rgb_out <= fifo_dout;
												  fifo_rd_en <= '1';
												  if hCounterRd = (spX + bpX + resX - 1) then
													  hActive <= '0';
													  state_hRead <= state_hFrontPorch;
													  fifo_rd_en <= '0';
													  rgb_out <= (others => '0');
												  end if;
				
				when state_hFrontPorch => hActive <= '0';
												  hCounterRd <= hCounterRd + 1;
												  fifo_rd_en <= '0';
												  rgb_out <= (others => '0');
												  if hCounterRd = (spX + bpX + resX + fpX - 1) then
													  state_hRead <= state_hReset;
													  vCounterRd <= vCounterRd + 1;
												  end if;
			end case;
			if vCounterRd = (spY + bpY + resY + fpY - 1) then
				state_vRead <= state_vReset;
				state_hRead <= state_hReset;
			end if;
				
	end case;
		
end if;
end process;

fifo_mux: process(rst, active_read_fifo, active_write_fifo)
begin

if rst = '1' then
end if;

if active_read_fifo = '0' then
end if;
end process;

dataSyncFifo : entity work.dataFIFO
  PORT MAP (
    rst           => fifo_rst,
    wr_clk        => fifo_wr_clk,
    rd_clk        => fifo_rd_clk,
    din           => fifo_din,
    wr_en         => fifo_wr_en,
    rd_en         => fifo_rd_en,
    dout          => fifo_dout,
    full          => fifo_full,
    almost_full   => fifo_almost_full,
    wr_ack        => fifo_wr_ack,
    overflow      => fifo_overflow,
    empty         => fifo_empty,
    almost_empty  => fifo_almost_empty,
    valid         => fifo_valid,
    underflow     => fifo_underflow,
    rd_data_count => fifo_rd_data_count,
    wr_data_count => fifo_wr_data_count
  );		
  
  
--dataSyncFifo0 : entity work.dataFIFO
--  PORT MAP (
--    rst           => fifo_rst,
--    wr_clk        => fifo_wr_clk,
--    rd_clk        => fifo_rd_clk,
--    din           => fifo_din,
--    wr_en         => fifo_wr_en,
--    rd_en         => fifo_rd_en,
--    dout          => fifo_dout,
--    full          => fifo_full,
--    almost_full   => fifo_almost_full,
--    wr_ack        => fifo_wr_ack,
--    overflow      => fifo_overflow,
--    empty         => fifo_empty,
--    almost_empty  => fifo_almost_empty,
--    valid         => fifo_valid,
--    underflow     => fifo_underflow,
--    rd_data_count => fifo_rd_data_count,
--    wr_data_count => fifo_wr_data_count
--  );
--dataSyncFifo1 : entity work.dataFIFO
--  PORT MAP (
--    rst           => fifo_rst,
--    wr_clk        => fifo_wr_clk,
--    rd_clk        => fifo_rd_clk,
--    din           => fifo_din,
--    wr_en         => fifo_wr_en,
--    rd_en         => fifo_rd_en,
--    dout          => fifo_dout,
--    full          => fifo_full,
--    almost_full   => fifo_almost_full,
--    wr_ack        => fifo_wr_ack,
--    overflow      => fifo_overflow,
--    empty         => fifo_empty,
--    almost_empty  => fifo_almost_empty,
--    valid         => fifo_valid,
--    underflow     => fifo_underflow,
--    rd_data_count => fifo_rd_data_count,
--    wr_data_count => fifo_wr_data_count
--  );		
end Behavioral;

