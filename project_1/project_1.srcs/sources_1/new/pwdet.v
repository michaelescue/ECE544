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
                MASK0 =             8'h04,      // Mask for Red PWM signal.
                MASK1 =             8'h02,      // Mask for Blue PWM signal.
                MASK2 =             8'h01       // Mask for Green PWM signal.                
    )(
    input                           clk,      // External clock 5MHz for project 1.
    input                           reset,
    input       [DIN_SIZE-1:0]      pwm_channel,
    output reg  [DOUT_SIZE-1:0]     pdc
    );
    
    // Pwdet Signals.
    wire                        red, 
                                blue, 
                                green;
    reg     [STATE_BITS-1:0]    state,
                                next_state;
    
    // Pwdet State
    localparam  red_toggle = 2'b00,
                blue_toggle = 2'b01,
                green_toggle = 2'b10,
                init_state = 2'b00;
        
    // Synchronizer signals
    wire    [DIN_SIZE-1:0]      sync_data,  // Intermediary value.
                                sync_output;    // Synchronized value.
    
    // Extracted Signals from synchronized input channel.
    assign red =    (sync_output & MASK0);
    assign blue =   (sync_output & MASK1);
    assign green =  (sync_output & MASK2);
    
    // Synchronize input
    ff_reg ff_0(clk, pwm_channel, sync_data);   // First stage ff.
    ff_reg ff_1(clk, sync_data, sync_output);       // Second stage ff.
    
    // State machine
    // Next state clocking.
    always @(posedge clk)
        begin
            if(reset)
                begin
                    state <= init_state;
                end
            else
                begin
                    state <= next_state;
                    load <= 0;
                end
        end
    
    // Next state logic
    always @(state or red or blue or green)
        begin
        end
    
    // Current State Output
    always @(state)
        begin
        end
endmodule


