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
                NUM_OF_COUNTERS =   2,
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
    
    // Pwdet Signals.
    wire                            red,
                                    blue,
                                    green;
    
    // Counter Signals
    wire     [DOUT_SIZE-1:0]        count_high,
                                    count_low;
                                    
    reg     [STATE_BITS-1:0]        state,
                                    next_state; 
                                    
    reg     [NUM_OF_COUNTERS-1:0]   counter_reset,
                                    increment;
                                                           
    
    // Pwdet State localparams
    localparam  red_toggle =    2'b00,
                blue_toggle =   2'b01,
                green_toggle =  2'b10,
                init_state =    2'b11;
                
    // Counter Enable localparams.
    localparam HIGH_CNTR =    2'b10,
               LOW_CNTR =     2'b01,
               ALL_OFF =      2'b00,
               ALL_ON =       2'b11;
        
    // Synchronizer signals.
    wire    [DIN_SIZE-1:0]      sync_data,      // Intermediary value.
                                sync_output;    // Synchronized value.
    
    // Signals Extracted from input channel.
    assign red =    sync_output[RED_BIT];
    assign blue =   sync_output[BLUE_BIT];
    assign green =  sync_output[GREEN_BIT];
    
    // Synchronize input: 2 clock cycle delay.
    ff_reg ff_0(clk, pwm_channel, sync_data);       // First stage ff.
    ff_reg ff_1(clk, sync_data, sync_output);       // Second stage ff.
    
    // Counters
    counter high(clk, counter_reset[1], increment[1], count_high);
    counter low(clk, counter_reset[0], increment[0], count_low);
    
    // State machine
    // Initial State
    initial
        begin
            pdc = 0;
            state = init_state;
            next_state = init_state;
            counter_reset = ALL_ON;
            increment = ALL_OFF;
       end
    
    // State Control Clocking.
    always @(posedge clk)
        begin
            if(reset)
                begin
                    state <= init_state;
                    counter_reset = ALL_ON;
                end
            else
                begin
                    state <= next_state;
                    counter_reset = ALL_OFF;
                end
        end
    
    // Current State Output
    always @(posedge red or posedge blue or posedge green or negedge red or negedge blue or negedge green)
            begin
                case(state)
                    red_toggle:         // = 2'b00,
                        begin
                            if(red)
                                begin
                                    counter_reset = HIGH_CNTR;
                                    increment = HIGH_CNTR;
                                    pdc = (count_high  * 99) / (count_high + count_low);
                                end
                            else
                                begin
                                    counter_reset = LOW_CNTR;
                                    increment = LOW_CNTR;
                                end
                        end
                    blue_toggle:        // = 2'b01,
                        begin
                            if(blue)    
                                begin
                                    counter_reset = HIGH_CNTR;
                                    increment = HIGH_CNTR;
                                    pdc = (count_high  * 99) / (count_high + count_low);
                                end
                            else        
                                begin
                                    counter_reset = LOW_CNTR;
                                    increment = LOW_CNTR;
                                end
                        end
                    green_toggle:       // = 2'b10,
                        begin
                            if(green)
                                begin
                                    counter_reset = HIGH_CNTR;
                                    increment = HIGH_CNTR;
                                    pdc = (count_high  * 99) / (count_high + count_low);
                                end
                            else       
                                begin
                                    counter_reset = LOW_CNTR;
                                    increment = LOW_CNTR;
                                end
                        end
                    init_state:         // = 2'b11;
                        begin
                            increment =     ALL_OFF;
                            counter_reset = ALL_ON;
                            pdc = 0;
                        end
                    default:
                        begin
                            increment =     ALL_OFF;
                            counter_reset = ALL_ON;
                            pdc = 0;
                        end
                endcase                
            end

    // Next State Logic
    always @(state or red or blue or green)
        begin
            case(sw[3:2])
                red_toggle:         // = 2'b00,
                    begin
                        next_state = red_toggle;
                    end
                blue_toggle:        // = 2'b01,
                    begin
                        next_state = blue_toggle;
                    end
                green_toggle:       // = 2'b10,
                    begin
                        next_state = green_toggle;
                    end
                init_state:         // = 2'b11;
                    begin
                        next_state = init_state;
                    end
                default:
                    begin
                        next_state = init_state;
                    end
            endcase
            
//            // Check for next_state change from current.
//            if((next_state ^ state))
//                begin
//                    counter_reset = ALL_ON;       // Both counters need to reset if next_state differs.
//                    increment = ALL_OFF;
//                end
        end
        
endmodule


