`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/20/2020 04:11:25 AM
// Design Name: 
// Module Name: testbench
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module testbench();

parameter TRUE = 1'b1;
parameter FALSE = 1'b0;
parameter CLOCK_CYCLE = 20 ;
parameter CLOCK_WIDTH = CLOCK_CYCLE/2;
parameter IDLE_CLOCKS = 2;
parameter  WIDTH = 8;

// Outputs of DUT.
wire q;

// Simulated Inputs to DUT.
reg Clock, Reset, d;

ff_reg #(WIDTH) ff0(Clock, d, q);

// Establish monitor block
initial
    begin
        $monitor($time, "%b", q);
    end
    
// Initiate free running clock.
initial
begin
    Clock = FALSE;
    forever #CLOCK_WIDTH Clock = ~Clock;
end

 // Reset for 2 cycles.
initial
    begin
        Reset = TRUE;
        repeat(IDLE_CLOCKS) @(negedge Clock);
        Reset = FALSE;
    end
    
// STimulus generation.
initial
    begin
        d = 0;
        repeat(10) @(negedge Clock);
        d = 1;
        repeat(10) @(negedge Clock);
        d = 0;
        repeat(10) @(negedge Clock);
        d = 1;
        repeat(10) @(negedge Clock);
        d = 0;
        repeat(10) @(negedge Clock);
        d = 0;
        repeat(10) @(negedge Clock);
        d = 1;
        repeat(10) @(negedge Clock);
        d = 0;
        repeat(10) @(negedge Clock);
        d = 1;
        repeat(10) @(negedge Clock);
        $stop;
    end
      

endmodule
