`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:   Michael Escue
// 
// Create Date: 04/20/2020 02:10:42 AM
// Design Name: 
// Module Name: ff_reg
// Project Name: ECE544 Project 1
// Target Devices: Xilinx A7 Microblaze core
// Tool Versions: 
// Description: Used for easy hierarchical design.
//              WIDTH is the signal width in bits.
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ff_reg #(
    parameter   WIDTH = 8
    )(
    input                       clk,
    input       [WIDTH-1:0]     d,
    output reg  [WIDTH-1:0]     q
    );
    
    // Synchronizer
    always @(posedge clk)   // Sechnronized-side clock.
        begin
            q <= d;    // Clock in d, the asynchronous input.
        end
             
endmodule
