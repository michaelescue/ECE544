`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:    Michael Escue 
// 
// Create Date: 04/19/2020 10:30:53 PM
// Design Name: 
// Module Name: pwdet
// Project Name: ECE544 Project 1
// Target Devices: Xilinx A7 Microblaze core.
// Tool Versions: 
// Description: Used for detection of Pulse Width Modulated signal duty cycle.
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module pwdet #(
    parameter   DIN_SIZE =          8,
                DOUT_SIZE =         32,
                COUNTER_BITS =      32,
                STATE_BITS =        2,
                SWITCH_SIZE =       16,
                NUM_OF_COUNTERS =   1,
                RED_BIT =          0,
                BLUE_BIT =         1,
                GREEN_BIT =        2      // Mask for Green PWM signal.             
    )(
    input                           clk,      // External clock 5MHz for project 1.
    input                           reset,
    input       [SWITCH_SIZE-1:0]   sw,
    input       [DIN_SIZE-1:0]      pwm_channel,
    output reg  [DOUT_SIZE-1:0]     pdc
    );
    
      
    // Synchronizer signals.
    wire    [DIN_SIZE-1:0]      sync_data,      // Intermediary value.
                                sync_output;    // Synchronized value.
    wire                        red,
                                blue,
                                green;
    
    // Counter Signals                                
    wire     [COUNTER_BITS-1:0]      count;
    
    wire                            reset_counter;
    reg     [NUM_OF_COUNTERS-1:0]   counter_reset;

    // State Machine signals
    wire                            ud_signal;

    reg     [DOUT_SIZE-1:0]         high_count,
                                    period;

     // Wires Extracted from input channel.
    assign red =    sync_output[RED_BIT];
    assign blue =   sync_output[BLUE_BIT];
    assign green =  sync_output[GREEN_BIT];
    
    // Input wire
    assign ud_signal = ((~sw[3] & sw[2]) & green) | ((sw[3] & ~sw[2]) & blue) | ((~sw[3] & ~sw[2]) & red);
    
    // Reset Sources
    assign reset_counter = reset | counter_reset;
    
    // Counters
    counter counter0(clk, reset_counter, count);
    
    // Synchronize input: 2 clock cycle delay.
    ff_reg ff_0(clk, pwm_channel, sync_data);       // First stage ff.
    ff_reg ff_1(clk, sync_data, sync_output);       // Second stage ff.
    
    // State machine
    // Initial State
    initial
        begin
            pdc = 0;
            period = 0;
            high_count = 0;
       end
    
    // Output.
    always @(posedge clk)
        begin
            if(reset)
                begin
                    pdc <= 0;
                    counter_reset <= 1;
                end
            else
                begin
                    counter_reset <= 0;
                    pdc <= ((high_count / period) * 100);
                end
         end

    always @(posedge ud_signal)
        begin
            period <= count;
            counter_reset <= 1;
        end

    always @(negedge ud_signal)
        begin
            high_count <= count;
        end

    
endmodule


