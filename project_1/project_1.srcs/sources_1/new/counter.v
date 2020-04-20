`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/20/2020 03:46:26 AM
// Design Name: 
// Module Name: counter
// Project Name: ECE544 Project 1
// Target Devices: Xilinx A7 Microblaze core.
// Tool Versions: 
// Description: Used for instantiating counter in heirarchical design.
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module counter #(
    parameter COUNTER_BITS = 32
    )(
    input                               clk,
    input                               reset,
    input                               increment,
    output reg [COUNTER_BITS-1:0]       count
    );
    
    initial count = 0;
    
    always@(posedge clk)
        begin
            if(reset)
                begin
                    count <= 0;
                end
            else if(increment)
                begin
                    count <= count + 1'b1;
                end
        end
        
endmodule