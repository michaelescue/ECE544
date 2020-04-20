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
parameter  WIDTH = 1;
parameter COUNTER_BITS = 32;
parameter RED_BIT = 8'h1;
parameter BLUE_BIT = 8'h2;
parameter GREEN_BIT = 8'h4;

// Outputs of DUT.
wire [31:0] pdc;

// Simulated Inputs to DUT.
reg Clock, Reset;
reg [15:0] sw;
reg [7:0] pwm;


// DUTs
//ff_reg ff0(Clock, d, q);
//ff_reg ff1(Clock, q, q1);
//counter counter0(Clock, Reset, q1, count0);
//counter counter1(Clock, Reset, 1'b1, count1);
    pwdet detector1 (Clock, Reset, sw, pwm, pdc);

// Establish monitor block
initial
    begin
        $monitor($time, "   %b  %b", sw, pwm);
    end
    
// Initiate free running clock.
initial
begin
    Clock = FALSE;
    Reset = TRUE;
    pwm = 0;
    sw = 0;
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
        sw = 32'h0;
        repeat(1) @(negedge Clock);
        pwm = pwm ^ RED_BIT;
        repeat(1) @(negedge Clock);
        pwm = pwm ^ RED_BIT;
        repeat(1) @(negedge Clock);
        pwm = pwm ^ RED_BIT;
        repeat(1) @(negedge Clock);
        pwm = pwm ^ RED_BIT;
        repeat(1) @(negedge Clock);
        pwm = pwm ^ RED_BIT;
        repeat(1) @(negedge Clock);
        pwm = pwm ^ RED_BIT;
        repeat(1) @(negedge Clock);
        pwm = pwm ^ RED_BIT;
        repeat(1) @(negedge Clock);
        pwm = pwm ^ RED_BIT;
        repeat(1) @(negedge Clock);
        
                        $stop;
    end
      

endmodule
