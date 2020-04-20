-- Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2018.2 (win64) Build 2258646 Thu Jun 14 20:03:12 MDT 2018
-- Date        : Mon Apr 13 12:02:10 2020
-- Host        : DESKTOP-J4B3MVP running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode synth_stub
--               {c:/Users/ME/OneDrive/Documents/School/PSU/Spring2020/ECE544/Projects/Project
--               0-Getting_Started/project_0/project_0.srcs/sources_1/bd/embsys/ip/embsys_clk_wiz_1_1/embsys_clk_wiz_1_1_stub.vhdl}
-- Design      : embsys_clk_wiz_1_1
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7a50tcsg324-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity embsys_clk_wiz_1_1 is
  Port ( 
    clk_out1 : out STD_LOGIC;
    clk_out2 : out STD_LOGIC;
    locked : out STD_LOGIC;
    clk_in1 : in STD_LOGIC
  );

end embsys_clk_wiz_1_1;

architecture stub of embsys_clk_wiz_1_1 is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "clk_out1,clk_out2,locked,clk_in1";
begin
end;
