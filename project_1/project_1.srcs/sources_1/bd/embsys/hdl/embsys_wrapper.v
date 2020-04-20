//Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2018.2 (win64) Build 2258646 Thu Jun 14 20:03:12 MDT 2018
//Date        : Sun Apr 19 23:59:08 2020
//Host        : DESKTOP-J4B3MVP running 64-bit major release  (build 9200)
//Command     : generate_target embsys_wrapper.bd
//Design      : embsys_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module embsys_wrapper
   (PmodOLEDrgb_out_0_pin10_io,
    PmodOLEDrgb_out_0_pin1_io,
    PmodOLEDrgb_out_0_pin2_io,
    PmodOLEDrgb_out_0_pin3_io,
    PmodOLEDrgb_out_0_pin4_io,
    PmodOLEDrgb_out_0_pin7_io,
    PmodOLEDrgb_out_0_pin8_io,
    PmodOLEDrgb_out_0_pin9_io,
    Pmod_out_0_pin10_io,
    Pmod_out_0_pin1_io,
    Pmod_out_0_pin2_io,
    Pmod_out_0_pin3_io,
    Pmod_out_0_pin4_io,
    Pmod_out_0_pin7_io,
    Pmod_out_0_pin8_io,
    Pmod_out_0_pin9_io,
    RGB1_Blue_0,
    RGB1_Green_0,
    RGB1_Red_0,
    RGB2_Blue_0,
    RGB2_Green_0,
    RGB2_Red_0,
    an_0,
    btnC_0,
    btnD_0,
    btnL_0,
    btnR_0,
    btnU_0,
    dp_0,
    gpio_rtl_0_tri_i,
    gpio_rtl_1_tri_o,
    gpio_rtl_2_tri_io,
    led_0,
    reset_rtl_0,
    seg_0,
    sw_0,
    sysclk,
    sysreset_n,
    uart_rtl_0_rxd,
    uart_rtl_0_txd);
  inout PmodOLEDrgb_out_0_pin10_io;
  inout PmodOLEDrgb_out_0_pin1_io;
  inout PmodOLEDrgb_out_0_pin2_io;
  inout PmodOLEDrgb_out_0_pin3_io;
  inout PmodOLEDrgb_out_0_pin4_io;
  inout PmodOLEDrgb_out_0_pin7_io;
  inout PmodOLEDrgb_out_0_pin8_io;
  inout PmodOLEDrgb_out_0_pin9_io;
  inout Pmod_out_0_pin10_io;
  inout Pmod_out_0_pin1_io;
  inout Pmod_out_0_pin2_io;
  inout Pmod_out_0_pin3_io;
  inout Pmod_out_0_pin4_io;
  inout Pmod_out_0_pin7_io;
  inout Pmod_out_0_pin8_io;
  inout Pmod_out_0_pin9_io;
  output RGB1_Blue_0;
  output RGB1_Green_0;
  output RGB1_Red_0;
  output RGB2_Blue_0;
  output RGB2_Green_0;
  output RGB2_Red_0;
  output [7:0]an_0;
  input btnC_0;
  input btnD_0;
  input btnL_0;
  input btnR_0;
  input btnU_0;
  output dp_0;
  input [7:0]gpio_rtl_0_tri_i;
  output [7:0]gpio_rtl_1_tri_o;
  inout [31:0]gpio_rtl_2_tri_io;
  output [15:0]led_0;
  input reset_rtl_0;
  output [6:0]seg_0;
  input [15:0]sw_0;
  input sysclk;
  input sysreset_n;
  input uart_rtl_0_rxd;
  output uart_rtl_0_txd;

  wire PmodOLEDrgb_out_0_pin10_i;
  wire PmodOLEDrgb_out_0_pin10_io;
  wire PmodOLEDrgb_out_0_pin10_o;
  wire PmodOLEDrgb_out_0_pin10_t;
  wire PmodOLEDrgb_out_0_pin1_i;
  wire PmodOLEDrgb_out_0_pin1_io;
  wire PmodOLEDrgb_out_0_pin1_o;
  wire PmodOLEDrgb_out_0_pin1_t;
  wire PmodOLEDrgb_out_0_pin2_i;
  wire PmodOLEDrgb_out_0_pin2_io;
  wire PmodOLEDrgb_out_0_pin2_o;
  wire PmodOLEDrgb_out_0_pin2_t;
  wire PmodOLEDrgb_out_0_pin3_i;
  wire PmodOLEDrgb_out_0_pin3_io;
  wire PmodOLEDrgb_out_0_pin3_o;
  wire PmodOLEDrgb_out_0_pin3_t;
  wire PmodOLEDrgb_out_0_pin4_i;
  wire PmodOLEDrgb_out_0_pin4_io;
  wire PmodOLEDrgb_out_0_pin4_o;
  wire PmodOLEDrgb_out_0_pin4_t;
  wire PmodOLEDrgb_out_0_pin7_i;
  wire PmodOLEDrgb_out_0_pin7_io;
  wire PmodOLEDrgb_out_0_pin7_o;
  wire PmodOLEDrgb_out_0_pin7_t;
  wire PmodOLEDrgb_out_0_pin8_i;
  wire PmodOLEDrgb_out_0_pin8_io;
  wire PmodOLEDrgb_out_0_pin8_o;
  wire PmodOLEDrgb_out_0_pin8_t;
  wire PmodOLEDrgb_out_0_pin9_i;
  wire PmodOLEDrgb_out_0_pin9_io;
  wire PmodOLEDrgb_out_0_pin9_o;
  wire PmodOLEDrgb_out_0_pin9_t;
  wire Pmod_out_0_pin10_i;
  wire Pmod_out_0_pin10_io;
  wire Pmod_out_0_pin10_o;
  wire Pmod_out_0_pin10_t;
  wire Pmod_out_0_pin1_i;
  wire Pmod_out_0_pin1_io;
  wire Pmod_out_0_pin1_o;
  wire Pmod_out_0_pin1_t;
  wire Pmod_out_0_pin2_i;
  wire Pmod_out_0_pin2_io;
  wire Pmod_out_0_pin2_o;
  wire Pmod_out_0_pin2_t;
  wire Pmod_out_0_pin3_i;
  wire Pmod_out_0_pin3_io;
  wire Pmod_out_0_pin3_o;
  wire Pmod_out_0_pin3_t;
  wire Pmod_out_0_pin4_i;
  wire Pmod_out_0_pin4_io;
  wire Pmod_out_0_pin4_o;
  wire Pmod_out_0_pin4_t;
  wire Pmod_out_0_pin7_i;
  wire Pmod_out_0_pin7_io;
  wire Pmod_out_0_pin7_o;
  wire Pmod_out_0_pin7_t;
  wire Pmod_out_0_pin8_i;
  wire Pmod_out_0_pin8_io;
  wire Pmod_out_0_pin8_o;
  wire Pmod_out_0_pin8_t;
  wire Pmod_out_0_pin9_i;
  wire Pmod_out_0_pin9_io;
  wire Pmod_out_0_pin9_o;
  wire Pmod_out_0_pin9_t;
  wire RGB1_Blue_0;
  wire RGB1_Green_0;
  wire RGB1_Red_0;
  wire RGB2_Blue_0;
  wire RGB2_Green_0;
  wire RGB2_Red_0;
  wire [7:0]an_0;
  wire btnC_0;
  wire btnD_0;
  wire btnL_0;
  wire btnR_0;
  wire btnU_0;
  wire dp_0;
  wire [7:0]gpio_rtl_0_tri_i;
  wire [7:0]gpio_rtl_1_tri_o;
  wire [0:0]gpio_rtl_2_tri_i_0;
  wire [1:1]gpio_rtl_2_tri_i_1;
  wire [10:10]gpio_rtl_2_tri_i_10;
  wire [11:11]gpio_rtl_2_tri_i_11;
  wire [12:12]gpio_rtl_2_tri_i_12;
  wire [13:13]gpio_rtl_2_tri_i_13;
  wire [14:14]gpio_rtl_2_tri_i_14;
  wire [15:15]gpio_rtl_2_tri_i_15;
  wire [16:16]gpio_rtl_2_tri_i_16;
  wire [17:17]gpio_rtl_2_tri_i_17;
  wire [18:18]gpio_rtl_2_tri_i_18;
  wire [19:19]gpio_rtl_2_tri_i_19;
  wire [2:2]gpio_rtl_2_tri_i_2;
  wire [20:20]gpio_rtl_2_tri_i_20;
  wire [21:21]gpio_rtl_2_tri_i_21;
  wire [22:22]gpio_rtl_2_tri_i_22;
  wire [23:23]gpio_rtl_2_tri_i_23;
  wire [24:24]gpio_rtl_2_tri_i_24;
  wire [25:25]gpio_rtl_2_tri_i_25;
  wire [26:26]gpio_rtl_2_tri_i_26;
  wire [27:27]gpio_rtl_2_tri_i_27;
  wire [28:28]gpio_rtl_2_tri_i_28;
  wire [29:29]gpio_rtl_2_tri_i_29;
  wire [3:3]gpio_rtl_2_tri_i_3;
  wire [30:30]gpio_rtl_2_tri_i_30;
  wire [31:31]gpio_rtl_2_tri_i_31;
  wire [4:4]gpio_rtl_2_tri_i_4;
  wire [5:5]gpio_rtl_2_tri_i_5;
  wire [6:6]gpio_rtl_2_tri_i_6;
  wire [7:7]gpio_rtl_2_tri_i_7;
  wire [8:8]gpio_rtl_2_tri_i_8;
  wire [9:9]gpio_rtl_2_tri_i_9;
  wire [0:0]gpio_rtl_2_tri_io_0;
  wire [1:1]gpio_rtl_2_tri_io_1;
  wire [10:10]gpio_rtl_2_tri_io_10;
  wire [11:11]gpio_rtl_2_tri_io_11;
  wire [12:12]gpio_rtl_2_tri_io_12;
  wire [13:13]gpio_rtl_2_tri_io_13;
  wire [14:14]gpio_rtl_2_tri_io_14;
  wire [15:15]gpio_rtl_2_tri_io_15;
  wire [16:16]gpio_rtl_2_tri_io_16;
  wire [17:17]gpio_rtl_2_tri_io_17;
  wire [18:18]gpio_rtl_2_tri_io_18;
  wire [19:19]gpio_rtl_2_tri_io_19;
  wire [2:2]gpio_rtl_2_tri_io_2;
  wire [20:20]gpio_rtl_2_tri_io_20;
  wire [21:21]gpio_rtl_2_tri_io_21;
  wire [22:22]gpio_rtl_2_tri_io_22;
  wire [23:23]gpio_rtl_2_tri_io_23;
  wire [24:24]gpio_rtl_2_tri_io_24;
  wire [25:25]gpio_rtl_2_tri_io_25;
  wire [26:26]gpio_rtl_2_tri_io_26;
  wire [27:27]gpio_rtl_2_tri_io_27;
  wire [28:28]gpio_rtl_2_tri_io_28;
  wire [29:29]gpio_rtl_2_tri_io_29;
  wire [3:3]gpio_rtl_2_tri_io_3;
  wire [30:30]gpio_rtl_2_tri_io_30;
  wire [31:31]gpio_rtl_2_tri_io_31;
  wire [4:4]gpio_rtl_2_tri_io_4;
  wire [5:5]gpio_rtl_2_tri_io_5;
  wire [6:6]gpio_rtl_2_tri_io_6;
  wire [7:7]gpio_rtl_2_tri_io_7;
  wire [8:8]gpio_rtl_2_tri_io_8;
  wire [9:9]gpio_rtl_2_tri_io_9;
  wire [0:0]gpio_rtl_2_tri_o_0;
  wire [1:1]gpio_rtl_2_tri_o_1;
  wire [10:10]gpio_rtl_2_tri_o_10;
  wire [11:11]gpio_rtl_2_tri_o_11;
  wire [12:12]gpio_rtl_2_tri_o_12;
  wire [13:13]gpio_rtl_2_tri_o_13;
  wire [14:14]gpio_rtl_2_tri_o_14;
  wire [15:15]gpio_rtl_2_tri_o_15;
  wire [16:16]gpio_rtl_2_tri_o_16;
  wire [17:17]gpio_rtl_2_tri_o_17;
  wire [18:18]gpio_rtl_2_tri_o_18;
  wire [19:19]gpio_rtl_2_tri_o_19;
  wire [2:2]gpio_rtl_2_tri_o_2;
  wire [20:20]gpio_rtl_2_tri_o_20;
  wire [21:21]gpio_rtl_2_tri_o_21;
  wire [22:22]gpio_rtl_2_tri_o_22;
  wire [23:23]gpio_rtl_2_tri_o_23;
  wire [24:24]gpio_rtl_2_tri_o_24;
  wire [25:25]gpio_rtl_2_tri_o_25;
  wire [26:26]gpio_rtl_2_tri_o_26;
  wire [27:27]gpio_rtl_2_tri_o_27;
  wire [28:28]gpio_rtl_2_tri_o_28;
  wire [29:29]gpio_rtl_2_tri_o_29;
  wire [3:3]gpio_rtl_2_tri_o_3;
  wire [30:30]gpio_rtl_2_tri_o_30;
  wire [31:31]gpio_rtl_2_tri_o_31;
  wire [4:4]gpio_rtl_2_tri_o_4;
  wire [5:5]gpio_rtl_2_tri_o_5;
  wire [6:6]gpio_rtl_2_tri_o_6;
  wire [7:7]gpio_rtl_2_tri_o_7;
  wire [8:8]gpio_rtl_2_tri_o_8;
  wire [9:9]gpio_rtl_2_tri_o_9;
  wire [0:0]gpio_rtl_2_tri_t_0;
  wire [1:1]gpio_rtl_2_tri_t_1;
  wire [10:10]gpio_rtl_2_tri_t_10;
  wire [11:11]gpio_rtl_2_tri_t_11;
  wire [12:12]gpio_rtl_2_tri_t_12;
  wire [13:13]gpio_rtl_2_tri_t_13;
  wire [14:14]gpio_rtl_2_tri_t_14;
  wire [15:15]gpio_rtl_2_tri_t_15;
  wire [16:16]gpio_rtl_2_tri_t_16;
  wire [17:17]gpio_rtl_2_tri_t_17;
  wire [18:18]gpio_rtl_2_tri_t_18;
  wire [19:19]gpio_rtl_2_tri_t_19;
  wire [2:2]gpio_rtl_2_tri_t_2;
  wire [20:20]gpio_rtl_2_tri_t_20;
  wire [21:21]gpio_rtl_2_tri_t_21;
  wire [22:22]gpio_rtl_2_tri_t_22;
  wire [23:23]gpio_rtl_2_tri_t_23;
  wire [24:24]gpio_rtl_2_tri_t_24;
  wire [25:25]gpio_rtl_2_tri_t_25;
  wire [26:26]gpio_rtl_2_tri_t_26;
  wire [27:27]gpio_rtl_2_tri_t_27;
  wire [28:28]gpio_rtl_2_tri_t_28;
  wire [29:29]gpio_rtl_2_tri_t_29;
  wire [3:3]gpio_rtl_2_tri_t_3;
  wire [30:30]gpio_rtl_2_tri_t_30;
  wire [31:31]gpio_rtl_2_tri_t_31;
  wire [4:4]gpio_rtl_2_tri_t_4;
  wire [5:5]gpio_rtl_2_tri_t_5;
  wire [6:6]gpio_rtl_2_tri_t_6;
  wire [7:7]gpio_rtl_2_tri_t_7;
  wire [8:8]gpio_rtl_2_tri_t_8;
  wire [9:9]gpio_rtl_2_tri_t_9;
  wire [15:0]led_0;
  wire reset_rtl_0;
  wire [6:0]seg_0;
  wire [15:0]sw_0;
  wire sysclk;
  wire sysreset_n;
  wire uart_rtl_0_rxd;
  wire uart_rtl_0_txd;

  IOBUF PmodOLEDrgb_out_0_pin10_iobuf
       (.I(PmodOLEDrgb_out_0_pin10_o),
        .IO(PmodOLEDrgb_out_0_pin10_io),
        .O(PmodOLEDrgb_out_0_pin10_i),
        .T(PmodOLEDrgb_out_0_pin10_t));
  IOBUF PmodOLEDrgb_out_0_pin1_iobuf
       (.I(PmodOLEDrgb_out_0_pin1_o),
        .IO(PmodOLEDrgb_out_0_pin1_io),
        .O(PmodOLEDrgb_out_0_pin1_i),
        .T(PmodOLEDrgb_out_0_pin1_t));
  IOBUF PmodOLEDrgb_out_0_pin2_iobuf
       (.I(PmodOLEDrgb_out_0_pin2_o),
        .IO(PmodOLEDrgb_out_0_pin2_io),
        .O(PmodOLEDrgb_out_0_pin2_i),
        .T(PmodOLEDrgb_out_0_pin2_t));
  IOBUF PmodOLEDrgb_out_0_pin3_iobuf
       (.I(PmodOLEDrgb_out_0_pin3_o),
        .IO(PmodOLEDrgb_out_0_pin3_io),
        .O(PmodOLEDrgb_out_0_pin3_i),
        .T(PmodOLEDrgb_out_0_pin3_t));
  IOBUF PmodOLEDrgb_out_0_pin4_iobuf
       (.I(PmodOLEDrgb_out_0_pin4_o),
        .IO(PmodOLEDrgb_out_0_pin4_io),
        .O(PmodOLEDrgb_out_0_pin4_i),
        .T(PmodOLEDrgb_out_0_pin4_t));
  IOBUF PmodOLEDrgb_out_0_pin7_iobuf
       (.I(PmodOLEDrgb_out_0_pin7_o),
        .IO(PmodOLEDrgb_out_0_pin7_io),
        .O(PmodOLEDrgb_out_0_pin7_i),
        .T(PmodOLEDrgb_out_0_pin7_t));
  IOBUF PmodOLEDrgb_out_0_pin8_iobuf
       (.I(PmodOLEDrgb_out_0_pin8_o),
        .IO(PmodOLEDrgb_out_0_pin8_io),
        .O(PmodOLEDrgb_out_0_pin8_i),
        .T(PmodOLEDrgb_out_0_pin8_t));
  IOBUF PmodOLEDrgb_out_0_pin9_iobuf
       (.I(PmodOLEDrgb_out_0_pin9_o),
        .IO(PmodOLEDrgb_out_0_pin9_io),
        .O(PmodOLEDrgb_out_0_pin9_i),
        .T(PmodOLEDrgb_out_0_pin9_t));
  IOBUF Pmod_out_0_pin10_iobuf
       (.I(Pmod_out_0_pin10_o),
        .IO(Pmod_out_0_pin10_io),
        .O(Pmod_out_0_pin10_i),
        .T(Pmod_out_0_pin10_t));
  IOBUF Pmod_out_0_pin1_iobuf
       (.I(Pmod_out_0_pin1_o),
        .IO(Pmod_out_0_pin1_io),
        .O(Pmod_out_0_pin1_i),
        .T(Pmod_out_0_pin1_t));
  IOBUF Pmod_out_0_pin2_iobuf
       (.I(Pmod_out_0_pin2_o),
        .IO(Pmod_out_0_pin2_io),
        .O(Pmod_out_0_pin2_i),
        .T(Pmod_out_0_pin2_t));
  IOBUF Pmod_out_0_pin3_iobuf
       (.I(Pmod_out_0_pin3_o),
        .IO(Pmod_out_0_pin3_io),
        .O(Pmod_out_0_pin3_i),
        .T(Pmod_out_0_pin3_t));
  IOBUF Pmod_out_0_pin4_iobuf
       (.I(Pmod_out_0_pin4_o),
        .IO(Pmod_out_0_pin4_io),
        .O(Pmod_out_0_pin4_i),
        .T(Pmod_out_0_pin4_t));
  IOBUF Pmod_out_0_pin7_iobuf
       (.I(Pmod_out_0_pin7_o),
        .IO(Pmod_out_0_pin7_io),
        .O(Pmod_out_0_pin7_i),
        .T(Pmod_out_0_pin7_t));
  IOBUF Pmod_out_0_pin8_iobuf
       (.I(Pmod_out_0_pin8_o),
        .IO(Pmod_out_0_pin8_io),
        .O(Pmod_out_0_pin8_i),
        .T(Pmod_out_0_pin8_t));
  IOBUF Pmod_out_0_pin9_iobuf
       (.I(Pmod_out_0_pin9_o),
        .IO(Pmod_out_0_pin9_io),
        .O(Pmod_out_0_pin9_i),
        .T(Pmod_out_0_pin9_t));
  embsys embsys_i
       (.PmodOLEDrgb_out_0_pin10_i(PmodOLEDrgb_out_0_pin10_i),
        .PmodOLEDrgb_out_0_pin10_o(PmodOLEDrgb_out_0_pin10_o),
        .PmodOLEDrgb_out_0_pin10_t(PmodOLEDrgb_out_0_pin10_t),
        .PmodOLEDrgb_out_0_pin1_i(PmodOLEDrgb_out_0_pin1_i),
        .PmodOLEDrgb_out_0_pin1_o(PmodOLEDrgb_out_0_pin1_o),
        .PmodOLEDrgb_out_0_pin1_t(PmodOLEDrgb_out_0_pin1_t),
        .PmodOLEDrgb_out_0_pin2_i(PmodOLEDrgb_out_0_pin2_i),
        .PmodOLEDrgb_out_0_pin2_o(PmodOLEDrgb_out_0_pin2_o),
        .PmodOLEDrgb_out_0_pin2_t(PmodOLEDrgb_out_0_pin2_t),
        .PmodOLEDrgb_out_0_pin3_i(PmodOLEDrgb_out_0_pin3_i),
        .PmodOLEDrgb_out_0_pin3_o(PmodOLEDrgb_out_0_pin3_o),
        .PmodOLEDrgb_out_0_pin3_t(PmodOLEDrgb_out_0_pin3_t),
        .PmodOLEDrgb_out_0_pin4_i(PmodOLEDrgb_out_0_pin4_i),
        .PmodOLEDrgb_out_0_pin4_o(PmodOLEDrgb_out_0_pin4_o),
        .PmodOLEDrgb_out_0_pin4_t(PmodOLEDrgb_out_0_pin4_t),
        .PmodOLEDrgb_out_0_pin7_i(PmodOLEDrgb_out_0_pin7_i),
        .PmodOLEDrgb_out_0_pin7_o(PmodOLEDrgb_out_0_pin7_o),
        .PmodOLEDrgb_out_0_pin7_t(PmodOLEDrgb_out_0_pin7_t),
        .PmodOLEDrgb_out_0_pin8_i(PmodOLEDrgb_out_0_pin8_i),
        .PmodOLEDrgb_out_0_pin8_o(PmodOLEDrgb_out_0_pin8_o),
        .PmodOLEDrgb_out_0_pin8_t(PmodOLEDrgb_out_0_pin8_t),
        .PmodOLEDrgb_out_0_pin9_i(PmodOLEDrgb_out_0_pin9_i),
        .PmodOLEDrgb_out_0_pin9_o(PmodOLEDrgb_out_0_pin9_o),
        .PmodOLEDrgb_out_0_pin9_t(PmodOLEDrgb_out_0_pin9_t),
        .Pmod_out_0_pin10_i(Pmod_out_0_pin10_i),
        .Pmod_out_0_pin10_o(Pmod_out_0_pin10_o),
        .Pmod_out_0_pin10_t(Pmod_out_0_pin10_t),
        .Pmod_out_0_pin1_i(Pmod_out_0_pin1_i),
        .Pmod_out_0_pin1_o(Pmod_out_0_pin1_o),
        .Pmod_out_0_pin1_t(Pmod_out_0_pin1_t),
        .Pmod_out_0_pin2_i(Pmod_out_0_pin2_i),
        .Pmod_out_0_pin2_o(Pmod_out_0_pin2_o),
        .Pmod_out_0_pin2_t(Pmod_out_0_pin2_t),
        .Pmod_out_0_pin3_i(Pmod_out_0_pin3_i),
        .Pmod_out_0_pin3_o(Pmod_out_0_pin3_o),
        .Pmod_out_0_pin3_t(Pmod_out_0_pin3_t),
        .Pmod_out_0_pin4_i(Pmod_out_0_pin4_i),
        .Pmod_out_0_pin4_o(Pmod_out_0_pin4_o),
        .Pmod_out_0_pin4_t(Pmod_out_0_pin4_t),
        .Pmod_out_0_pin7_i(Pmod_out_0_pin7_i),
        .Pmod_out_0_pin7_o(Pmod_out_0_pin7_o),
        .Pmod_out_0_pin7_t(Pmod_out_0_pin7_t),
        .Pmod_out_0_pin8_i(Pmod_out_0_pin8_i),
        .Pmod_out_0_pin8_o(Pmod_out_0_pin8_o),
        .Pmod_out_0_pin8_t(Pmod_out_0_pin8_t),
        .Pmod_out_0_pin9_i(Pmod_out_0_pin9_i),
        .Pmod_out_0_pin9_o(Pmod_out_0_pin9_o),
        .Pmod_out_0_pin9_t(Pmod_out_0_pin9_t),
        .RGB1_Blue_0(RGB1_Blue_0),
        .RGB1_Green_0(RGB1_Green_0),
        .RGB1_Red_0(RGB1_Red_0),
        .RGB2_Blue_0(RGB2_Blue_0),
        .RGB2_Green_0(RGB2_Green_0),
        .RGB2_Red_0(RGB2_Red_0),
        .an_0(an_0),
        .btnC_0(btnC_0),
        .btnD_0(btnD_0),
        .btnL_0(btnL_0),
        .btnR_0(btnR_0),
        .btnU_0(btnU_0),
        .dp_0(dp_0),
        .gpio_rtl_0_tri_i(gpio_rtl_0_tri_i),
        .gpio_rtl_1_tri_o(gpio_rtl_1_tri_o),
        .gpio_rtl_2_tri_i({gpio_rtl_2_tri_i_31,gpio_rtl_2_tri_i_30,gpio_rtl_2_tri_i_29,gpio_rtl_2_tri_i_28,gpio_rtl_2_tri_i_27,gpio_rtl_2_tri_i_26,gpio_rtl_2_tri_i_25,gpio_rtl_2_tri_i_24,gpio_rtl_2_tri_i_23,gpio_rtl_2_tri_i_22,gpio_rtl_2_tri_i_21,gpio_rtl_2_tri_i_20,gpio_rtl_2_tri_i_19,gpio_rtl_2_tri_i_18,gpio_rtl_2_tri_i_17,gpio_rtl_2_tri_i_16,gpio_rtl_2_tri_i_15,gpio_rtl_2_tri_i_14,gpio_rtl_2_tri_i_13,gpio_rtl_2_tri_i_12,gpio_rtl_2_tri_i_11,gpio_rtl_2_tri_i_10,gpio_rtl_2_tri_i_9,gpio_rtl_2_tri_i_8,gpio_rtl_2_tri_i_7,gpio_rtl_2_tri_i_6,gpio_rtl_2_tri_i_5,gpio_rtl_2_tri_i_4,gpio_rtl_2_tri_i_3,gpio_rtl_2_tri_i_2,gpio_rtl_2_tri_i_1,gpio_rtl_2_tri_i_0}),
        .gpio_rtl_2_tri_o({gpio_rtl_2_tri_o_31,gpio_rtl_2_tri_o_30,gpio_rtl_2_tri_o_29,gpio_rtl_2_tri_o_28,gpio_rtl_2_tri_o_27,gpio_rtl_2_tri_o_26,gpio_rtl_2_tri_o_25,gpio_rtl_2_tri_o_24,gpio_rtl_2_tri_o_23,gpio_rtl_2_tri_o_22,gpio_rtl_2_tri_o_21,gpio_rtl_2_tri_o_20,gpio_rtl_2_tri_o_19,gpio_rtl_2_tri_o_18,gpio_rtl_2_tri_o_17,gpio_rtl_2_tri_o_16,gpio_rtl_2_tri_o_15,gpio_rtl_2_tri_o_14,gpio_rtl_2_tri_o_13,gpio_rtl_2_tri_o_12,gpio_rtl_2_tri_o_11,gpio_rtl_2_tri_o_10,gpio_rtl_2_tri_o_9,gpio_rtl_2_tri_o_8,gpio_rtl_2_tri_o_7,gpio_rtl_2_tri_o_6,gpio_rtl_2_tri_o_5,gpio_rtl_2_tri_o_4,gpio_rtl_2_tri_o_3,gpio_rtl_2_tri_o_2,gpio_rtl_2_tri_o_1,gpio_rtl_2_tri_o_0}),
        .gpio_rtl_2_tri_t({gpio_rtl_2_tri_t_31,gpio_rtl_2_tri_t_30,gpio_rtl_2_tri_t_29,gpio_rtl_2_tri_t_28,gpio_rtl_2_tri_t_27,gpio_rtl_2_tri_t_26,gpio_rtl_2_tri_t_25,gpio_rtl_2_tri_t_24,gpio_rtl_2_tri_t_23,gpio_rtl_2_tri_t_22,gpio_rtl_2_tri_t_21,gpio_rtl_2_tri_t_20,gpio_rtl_2_tri_t_19,gpio_rtl_2_tri_t_18,gpio_rtl_2_tri_t_17,gpio_rtl_2_tri_t_16,gpio_rtl_2_tri_t_15,gpio_rtl_2_tri_t_14,gpio_rtl_2_tri_t_13,gpio_rtl_2_tri_t_12,gpio_rtl_2_tri_t_11,gpio_rtl_2_tri_t_10,gpio_rtl_2_tri_t_9,gpio_rtl_2_tri_t_8,gpio_rtl_2_tri_t_7,gpio_rtl_2_tri_t_6,gpio_rtl_2_tri_t_5,gpio_rtl_2_tri_t_4,gpio_rtl_2_tri_t_3,gpio_rtl_2_tri_t_2,gpio_rtl_2_tri_t_1,gpio_rtl_2_tri_t_0}),
        .led_0(led_0),
        .reset_rtl_0(reset_rtl_0),
        .seg_0(seg_0),
        .sw_0(sw_0),
        .sysclk(sysclk),
        .sysreset_n(sysreset_n),
        .uart_rtl_0_rxd(uart_rtl_0_rxd),
        .uart_rtl_0_txd(uart_rtl_0_txd));
  IOBUF gpio_rtl_2_tri_iobuf_0
       (.I(gpio_rtl_2_tri_o_0),
        .IO(gpio_rtl_2_tri_io[0]),
        .O(gpio_rtl_2_tri_i_0),
        .T(gpio_rtl_2_tri_t_0));
  IOBUF gpio_rtl_2_tri_iobuf_1
       (.I(gpio_rtl_2_tri_o_1),
        .IO(gpio_rtl_2_tri_io[1]),
        .O(gpio_rtl_2_tri_i_1),
        .T(gpio_rtl_2_tri_t_1));
  IOBUF gpio_rtl_2_tri_iobuf_10
       (.I(gpio_rtl_2_tri_o_10),
        .IO(gpio_rtl_2_tri_io[10]),
        .O(gpio_rtl_2_tri_i_10),
        .T(gpio_rtl_2_tri_t_10));
  IOBUF gpio_rtl_2_tri_iobuf_11
       (.I(gpio_rtl_2_tri_o_11),
        .IO(gpio_rtl_2_tri_io[11]),
        .O(gpio_rtl_2_tri_i_11),
        .T(gpio_rtl_2_tri_t_11));
  IOBUF gpio_rtl_2_tri_iobuf_12
       (.I(gpio_rtl_2_tri_o_12),
        .IO(gpio_rtl_2_tri_io[12]),
        .O(gpio_rtl_2_tri_i_12),
        .T(gpio_rtl_2_tri_t_12));
  IOBUF gpio_rtl_2_tri_iobuf_13
       (.I(gpio_rtl_2_tri_o_13),
        .IO(gpio_rtl_2_tri_io[13]),
        .O(gpio_rtl_2_tri_i_13),
        .T(gpio_rtl_2_tri_t_13));
  IOBUF gpio_rtl_2_tri_iobuf_14
       (.I(gpio_rtl_2_tri_o_14),
        .IO(gpio_rtl_2_tri_io[14]),
        .O(gpio_rtl_2_tri_i_14),
        .T(gpio_rtl_2_tri_t_14));
  IOBUF gpio_rtl_2_tri_iobuf_15
       (.I(gpio_rtl_2_tri_o_15),
        .IO(gpio_rtl_2_tri_io[15]),
        .O(gpio_rtl_2_tri_i_15),
        .T(gpio_rtl_2_tri_t_15));
  IOBUF gpio_rtl_2_tri_iobuf_16
       (.I(gpio_rtl_2_tri_o_16),
        .IO(gpio_rtl_2_tri_io[16]),
        .O(gpio_rtl_2_tri_i_16),
        .T(gpio_rtl_2_tri_t_16));
  IOBUF gpio_rtl_2_tri_iobuf_17
       (.I(gpio_rtl_2_tri_o_17),
        .IO(gpio_rtl_2_tri_io[17]),
        .O(gpio_rtl_2_tri_i_17),
        .T(gpio_rtl_2_tri_t_17));
  IOBUF gpio_rtl_2_tri_iobuf_18
       (.I(gpio_rtl_2_tri_o_18),
        .IO(gpio_rtl_2_tri_io[18]),
        .O(gpio_rtl_2_tri_i_18),
        .T(gpio_rtl_2_tri_t_18));
  IOBUF gpio_rtl_2_tri_iobuf_19
       (.I(gpio_rtl_2_tri_o_19),
        .IO(gpio_rtl_2_tri_io[19]),
        .O(gpio_rtl_2_tri_i_19),
        .T(gpio_rtl_2_tri_t_19));
  IOBUF gpio_rtl_2_tri_iobuf_2
       (.I(gpio_rtl_2_tri_o_2),
        .IO(gpio_rtl_2_tri_io[2]),
        .O(gpio_rtl_2_tri_i_2),
        .T(gpio_rtl_2_tri_t_2));
  IOBUF gpio_rtl_2_tri_iobuf_20
       (.I(gpio_rtl_2_tri_o_20),
        .IO(gpio_rtl_2_tri_io[20]),
        .O(gpio_rtl_2_tri_i_20),
        .T(gpio_rtl_2_tri_t_20));
  IOBUF gpio_rtl_2_tri_iobuf_21
       (.I(gpio_rtl_2_tri_o_21),
        .IO(gpio_rtl_2_tri_io[21]),
        .O(gpio_rtl_2_tri_i_21),
        .T(gpio_rtl_2_tri_t_21));
  IOBUF gpio_rtl_2_tri_iobuf_22
       (.I(gpio_rtl_2_tri_o_22),
        .IO(gpio_rtl_2_tri_io[22]),
        .O(gpio_rtl_2_tri_i_22),
        .T(gpio_rtl_2_tri_t_22));
  IOBUF gpio_rtl_2_tri_iobuf_23
       (.I(gpio_rtl_2_tri_o_23),
        .IO(gpio_rtl_2_tri_io[23]),
        .O(gpio_rtl_2_tri_i_23),
        .T(gpio_rtl_2_tri_t_23));
  IOBUF gpio_rtl_2_tri_iobuf_24
       (.I(gpio_rtl_2_tri_o_24),
        .IO(gpio_rtl_2_tri_io[24]),
        .O(gpio_rtl_2_tri_i_24),
        .T(gpio_rtl_2_tri_t_24));
  IOBUF gpio_rtl_2_tri_iobuf_25
       (.I(gpio_rtl_2_tri_o_25),
        .IO(gpio_rtl_2_tri_io[25]),
        .O(gpio_rtl_2_tri_i_25),
        .T(gpio_rtl_2_tri_t_25));
  IOBUF gpio_rtl_2_tri_iobuf_26
       (.I(gpio_rtl_2_tri_o_26),
        .IO(gpio_rtl_2_tri_io[26]),
        .O(gpio_rtl_2_tri_i_26),
        .T(gpio_rtl_2_tri_t_26));
  IOBUF gpio_rtl_2_tri_iobuf_27
       (.I(gpio_rtl_2_tri_o_27),
        .IO(gpio_rtl_2_tri_io[27]),
        .O(gpio_rtl_2_tri_i_27),
        .T(gpio_rtl_2_tri_t_27));
  IOBUF gpio_rtl_2_tri_iobuf_28
       (.I(gpio_rtl_2_tri_o_28),
        .IO(gpio_rtl_2_tri_io[28]),
        .O(gpio_rtl_2_tri_i_28),
        .T(gpio_rtl_2_tri_t_28));
  IOBUF gpio_rtl_2_tri_iobuf_29
       (.I(gpio_rtl_2_tri_o_29),
        .IO(gpio_rtl_2_tri_io[29]),
        .O(gpio_rtl_2_tri_i_29),
        .T(gpio_rtl_2_tri_t_29));
  IOBUF gpio_rtl_2_tri_iobuf_3
       (.I(gpio_rtl_2_tri_o_3),
        .IO(gpio_rtl_2_tri_io[3]),
        .O(gpio_rtl_2_tri_i_3),
        .T(gpio_rtl_2_tri_t_3));
  IOBUF gpio_rtl_2_tri_iobuf_30
       (.I(gpio_rtl_2_tri_o_30),
        .IO(gpio_rtl_2_tri_io[30]),
        .O(gpio_rtl_2_tri_i_30),
        .T(gpio_rtl_2_tri_t_30));
  IOBUF gpio_rtl_2_tri_iobuf_31
       (.I(gpio_rtl_2_tri_o_31),
        .IO(gpio_rtl_2_tri_io[31]),
        .O(gpio_rtl_2_tri_i_31),
        .T(gpio_rtl_2_tri_t_31));
  IOBUF gpio_rtl_2_tri_iobuf_4
       (.I(gpio_rtl_2_tri_o_4),
        .IO(gpio_rtl_2_tri_io[4]),
        .O(gpio_rtl_2_tri_i_4),
        .T(gpio_rtl_2_tri_t_4));
  IOBUF gpio_rtl_2_tri_iobuf_5
       (.I(gpio_rtl_2_tri_o_5),
        .IO(gpio_rtl_2_tri_io[5]),
        .O(gpio_rtl_2_tri_i_5),
        .T(gpio_rtl_2_tri_t_5));
  IOBUF gpio_rtl_2_tri_iobuf_6
       (.I(gpio_rtl_2_tri_o_6),
        .IO(gpio_rtl_2_tri_io[6]),
        .O(gpio_rtl_2_tri_i_6),
        .T(gpio_rtl_2_tri_t_6));
  IOBUF gpio_rtl_2_tri_iobuf_7
       (.I(gpio_rtl_2_tri_o_7),
        .IO(gpio_rtl_2_tri_io[7]),
        .O(gpio_rtl_2_tri_i_7),
        .T(gpio_rtl_2_tri_t_7));
  IOBUF gpio_rtl_2_tri_iobuf_8
       (.I(gpio_rtl_2_tri_o_8),
        .IO(gpio_rtl_2_tri_io[8]),
        .O(gpio_rtl_2_tri_i_8),
        .T(gpio_rtl_2_tri_t_8));
  IOBUF gpio_rtl_2_tri_iobuf_9
       (.I(gpio_rtl_2_tri_o_9),
        .IO(gpio_rtl_2_tri_io[9]),
        .O(gpio_rtl_2_tri_i_9),
        .T(gpio_rtl_2_tri_t_9));
endmodule
