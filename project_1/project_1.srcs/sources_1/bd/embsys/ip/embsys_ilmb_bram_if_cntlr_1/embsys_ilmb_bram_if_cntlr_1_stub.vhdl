-- Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2018.2 (win64) Build 2258646 Thu Jun 14 20:03:12 MDT 2018
-- Date        : Mon Apr 13 12:03:37 2020
-- Host        : DESKTOP-J4B3MVP running 64-bit major release  (build 9200)
-- Command     : write_vhdl -force -mode synth_stub
--               {c:/Users/ME/OneDrive/Documents/School/PSU/Spring2020/ECE544/Projects/Project
--               0-Getting_Started/project_0/project_0.srcs/sources_1/bd/embsys/ip/embsys_ilmb_bram_if_cntlr_1/embsys_ilmb_bram_if_cntlr_1_stub.vhdl}
-- Design      : embsys_ilmb_bram_if_cntlr_1
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7a50tcsg324-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity embsys_ilmb_bram_if_cntlr_1 is
  Port ( 
    LMB_Clk : in STD_LOGIC;
    LMB_Rst : in STD_LOGIC;
    LMB_ABus : in STD_LOGIC_VECTOR ( 0 to 31 );
    LMB_WriteDBus : in STD_LOGIC_VECTOR ( 0 to 31 );
    LMB_AddrStrobe : in STD_LOGIC;
    LMB_ReadStrobe : in STD_LOGIC;
    LMB_WriteStrobe : in STD_LOGIC;
    LMB_BE : in STD_LOGIC_VECTOR ( 0 to 3 );
    Sl_DBus : out STD_LOGIC_VECTOR ( 0 to 31 );
    Sl_Ready : out STD_LOGIC;
    Sl_Wait : out STD_LOGIC;
    Sl_UE : out STD_LOGIC;
    Sl_CE : out STD_LOGIC;
    BRAM_Rst_A : out STD_LOGIC;
    BRAM_Clk_A : out STD_LOGIC;
    BRAM_Addr_A : out STD_LOGIC_VECTOR ( 0 to 31 );
    BRAM_EN_A : out STD_LOGIC;
    BRAM_WEN_A : out STD_LOGIC_VECTOR ( 0 to 3 );
    BRAM_Dout_A : out STD_LOGIC_VECTOR ( 0 to 31 );
    BRAM_Din_A : in STD_LOGIC_VECTOR ( 0 to 31 )
  );

end embsys_ilmb_bram_if_cntlr_1;

architecture stub of embsys_ilmb_bram_if_cntlr_1 is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "LMB_Clk,LMB_Rst,LMB_ABus[0:31],LMB_WriteDBus[0:31],LMB_AddrStrobe,LMB_ReadStrobe,LMB_WriteStrobe,LMB_BE[0:3],Sl_DBus[0:31],Sl_Ready,Sl_Wait,Sl_UE,Sl_CE,BRAM_Rst_A,BRAM_Clk_A,BRAM_Addr_A[0:31],BRAM_EN_A,BRAM_WEN_A[0:3],BRAM_Dout_A[0:31],BRAM_Din_A[0:31]";
attribute x_core_info : string;
attribute x_core_info of stub : architecture is "lmb_bram_if_cntlr,Vivado 2018.2";
begin
end;