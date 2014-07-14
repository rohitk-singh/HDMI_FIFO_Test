--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   04:15:12 07/13/2014
-- Design Name:   
-- Module Name:   E:/HDMI_FIFO_Test/tb_vgacapture.vhd
-- Project Name:  HDMI_FIFO_Test
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: VGA_Capture
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
 USE ieee.std_logic_arith.all;		 
USE ieee.std_logic_unsigned.all;
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY tb_vgacapture IS
END tb_vgacapture;
 
ARCHITECTURE behavior OF tb_vgacapture IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT VGA_Capture
    PORT(
         rst : IN  std_logic;
         DATACK : IN  std_logic;
         HSOUT : IN  std_logic;
         VSOUT : IN  std_logic;
         SOGOUT : IN  std_logic;
         R_in : IN  std_logic_vector(7 downto 0);
         G_in : IN  std_logic_vector(7 downto 0);
         B_in : IN  std_logic_vector(7 downto 0);
         img_pclk : IN  std_logic;
         SW : IN  std_logic_vector(3 downto 0);
         debug_out : OUT  std_logic_vector(15 downto 0);
         de : OUT  std_logic;
         HS : OUT  std_logic;
         VS : OUT  std_logic;
         Red_out : OUT  std_logic_vector(7 downto 0);
         Green_out : OUT  std_logic_vector(7 downto 0);
         Blue_out : OUT  std_logic_vector(7 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal rst : std_logic := '0';
   signal DATACK : std_logic := '0';
   signal HSOUT : std_logic := '0';
   signal VSOUT : std_logic := '0';
   signal SOGOUT : std_logic := '0';
   signal R_in : std_logic_vector(7 downto 0) := (others => '0');
   signal G_in : std_logic_vector(7 downto 0) := (others => '0');
   signal B_in : std_logic_vector(7 downto 0) := (others => '0');
   signal img_pclk : std_logic := '0';
   signal SW : std_logic_vector(3 downto 0) := (others => '0');

 	--Outputs
   signal debug_out : std_logic_vector(15 downto 0);
   signal de : std_logic;
   signal HS : std_logic;
   signal VS : std_logic;
   signal Red_out : std_logic_vector(7 downto 0);
   signal Green_out : std_logic_vector(7 downto 0);
   signal Blue_out : std_logic_vector(7 downto 0);

signal counterX : std_logic_vector(15 downto 0) := (others => '0');
signal counterY : std_logic_vector(15 downto 0) := (others => '0');
signal resX_i : std_logic_vector(15 downto 0) := (others => '0');
signal resY_i : std_logic_vector(15 downto 0) := (others => '0');

signal spY : integer;
signal bpY : integer;
signal fpY : integer;
signal spX : integer;
signal bpX : integer;
signal fpX : integer;

signal data : std_logic_vector(23 downto 0) := (others => '0');


signal pclk_i : std_logic := '0';
signal vsync_i : std_logic := '0';
signal hsync_i : std_logic := '0';
signal vActive : std_logic := '0';
signal hActive : std_logic := '0';
signal rst_n : std_logic := '1';

   -- Clock period definitions
   constant img_pclk_period : time := 10 ns;
 
BEGIN

DATACK <= img_pclk;
pclk_i <= img_pclk;
HSOUT <= not hsync_i;
vsout <= not vsync_i;

R_in <= data(23 downto 16);
G_in <= data(15 downto 8);
B_in <= data(7 downto 0);

  
	-- Instantiate the Unit Under Test (UUT)
   uut: VGA_Capture PORT MAP (
          rst => rst,
          DATACK => DATACK,
          HSOUT => HSOUT,
          VSOUT => VSOUT,
          SOGOUT => SOGOUT,
          R_in => R_in,
          G_in => G_in,
          B_in => B_in,
          img_pclk => img_pclk,
          SW => SW,
          debug_out => debug_out,
          de => de,
          HS => HS,
          VS => VS,
          Red_out => Red_out,
          Green_out => Green_out,
          Blue_out => Blue_out
        );

   -- Clock process definitions
   img_pclk_process :process
   begin
		img_pclk <= '0';
		wait for img_pclk_period/2;
		img_pclk <= '1';
		wait for img_pclk_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
      wait for 100 ns;	

      wait for img_pclk_period*10;

      -- insert stimulus here 

      wait;
   end process;
	resX_i <= X"0400";
	resY_i <= X"0300";
		
	spY <= 6;
	bpY <= 29;
	fpY <= 3;
	
	spX <= 136;
	bpX <= 160;
	fpX <= 24;






process(rst_n,pclk_i)
begin
	if rst_n = '0' then
	
		data 	 <= (others => '0');
		counterX <= (others => '0');
		counterY <= (others => '0');
		vsync_i <= '0';
		vActive <= '0';
		hsync_i <= '0';
		hActive <= '0';
		
	elsif rising_edge(pclk_i) then
	
		counterX <= counterX + 1;
		
		if counterY = 0 then 
			vsync_i <= '0';
			vActive <= '0';		
		elsif counterY = spY then
			vsync_i <= '1';
		elsif counterY = (spY+bpY) then
			vActive <= '1';		
		elsif counterY = (spY+bpY+CONV_INTEGER(resY_i)) then
			vActive <= '0';		
		elsif counterY = (spY+bpY+CONV_INTEGER(resY_i)+fpY) then
			counterY <= (others => '0');
			data <= (others => '0');
		end if;
	

		if counterX = 0 then 
			hsync_i <= '0';
			hActive <= '0';		
		elsif counterX = spX then
			hsync_i <= '1';
		elsif counterX = (spX+bpX) then
			hActive <= '1';		
		elsif counterX = (spX+bpX+CONV_INTEGER(resX_i)) then
			hActive <= '0';		
		elsif counterX = (spX+bpX+CONV_INTEGER(resX_i)+fpX) then
			counterX <= (others => '0');
			counterY <= counterY +1;
			counterX <= (others => '0');			
		end if;
	
		if vActive = '1' and hActive = '1' then	
			data <= data +1;
		end if;

		
	end if;
end process;
END;
