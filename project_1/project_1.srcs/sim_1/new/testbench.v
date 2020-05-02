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
parameter RED_BIT = 0;
parameter BLUE_BIT = 1;
parameter GREEN_BIT = 2;
parameter p0 = CLOCK_WIDTH;
parameter p1 = p0 * 2,
            p2 = p0 + p1,
            p3 = p2 + p1;
parameter PEAK_TO_PEAK = 4;
parameter PTH_RATIO = 1; // duty cycle = (1 / PTH_RATIO) * 100
parameter RED_DC = PTH_RATIO * 2;
parameter BLUE_DC = PTH_RATIO * 4;
parameter GREEN_DC = PTH_RATIO * 3;
integer t;
integer t_high,
        t_low;

// Outputs of DUT.
wire [7:0] pdc;

// Simulated Inputs to DUT.
reg Clock, Reset;
wire wClock;
reg [15:0] sw;
reg [7:0] pwm;

assign wClock = Clock;

//    pwdet detector1 (Clock, Reset, sw, pwm, pdc);
        pwdet detector1 (wClock, Reset, sw, pwm, pdc);

// Establish monitor block
initial
    begin
        $monitor($time,"\tsw:%-4b\tpwm:%-3b\tt:%-d\tpdc:%-2d\tt_high:%-d\tt_low:%-d\tduty:%-2d", sw, pwm, t, pdc, t_high, t_low, (t_high * 100)/(t_high+t_low) );
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
        for(integer i = 0; i < 10; i = i + 1) 
            begin
                $display("Iteration %d", i);
                if(i % 2) t = p0;
                else if( i % 3) t = p1;
                else t = p3;
                

                
                //***********************************************************************************//
                sw = 32'h0;
               // repeat(t) @(negedge Clock);    // Not included
               
                t_high = t;
                t_low = t * RED_DC;

                repeat(4*PEAK_TO_PEAK)
                    begin 
                        pwm[RED_BIT] = ~pwm[RED_BIT];   // Bit goes high.
                        if(pwm[RED_BIT]) repeat(t_high) @(negedge Clock);
                        else repeat(t_low) @(negedge Clock);
                    end
        
                //***********************************************************************************//
                sw = 32'h4;                     // Bit still remains low here (adding to counter).
              //  repeat(1) @(negedge Clock);

                t_high = t;
                t_low = t * BLUE_DC;

                repeat(4*PEAK_TO_PEAK)
                    begin
                        pwm[BLUE_BIT] = ~pwm[BLUE_BIT];   // Bit goes high.
                        if(pwm[BLUE_BIT]) repeat(t_high) @(negedge Clock);
                        else repeat(t_low) @(negedge Clock);
                    end
        
                //***********************************************************************************//        sw = 32'h8;
                sw = 32'h8; 
               // repeat(t) @(negedge Clock);     
                
                t_high = t;
                t_low = t * GREEN_DC;
            
                repeat(4*PEAK_TO_PEAK)
                    begin
                        pwm[GREEN_BIT] = ~pwm[GREEN_BIT];   // Bit goes high.
                        if(pwm[GREEN_BIT]) repeat(t_high) @(negedge Clock);
                        else               repeat(t_low) @(negedge Clock);
                    end
            end
            
        //*******************SECOND FOR LOOP *****************************************************************//
        for( integer i = 0; i < 10; i = i + 1) 
                    begin
                        $display("Iteration:\t%d", i);
                        if(i % 3) t = p0;
                        else if( i % 2) t = p1;
                        else t = p3;
                        
                        
                        //***********************************************************************************//
                        sw = 32'h0;
                       // repeat(t) @(negedge Clock);    // Not included
                       
                        repeat(4*PEAK_TO_PEAK)
                                                begin 
                                                    pwm[RED_BIT] = ~pwm[RED_BIT];   // Bit goes high.
                                                    pwm[BLUE_BIT] = ~pwm[BLUE_BIT];
                                                    pwm[GREEN_BIT] = ~pwm[GREEN_BIT];
                                                    if(pwm[GREEN_BIT] | pwm[RED_BIT] | pwm[BLUE_BIT]) repeat(t_high) @(negedge Clock);
                                                    else               repeat(t_low*i) @(negedge Clock);
                                                end
                
                        //***********************************************************************************//
                        sw = 32'h4;                     // Bit still remains low here (adding to counter).
                      //  repeat(1) @(negedge Clock);
        
                        repeat(4*PEAK_TO_PEAK)
                                                  begin 
                                                      pwm[RED_BIT] = ~pwm[RED_BIT];   // Bit goes high.
                                                      pwm[BLUE_BIT] = ~pwm[BLUE_BIT];
                                                      pwm[GREEN_BIT] = ~pwm[GREEN_BIT];
                                                    if(pwm[GREEN_BIT] | pwm[RED_BIT] | pwm[BLUE_BIT]) repeat(t_high) @(negedge Clock);
                                                    else               repeat(t_low) @(negedge Clock);
                                                end

                
                        //***********************************************************************************//        sw = 32'h8;
                        sw = 32'h8; 
                       // repeat(t) @(negedge Clock);     
                        
                        repeat(4*PEAK_TO_PEAK)
                                                   begin 
                                                       pwm[RED_BIT] = ~pwm[RED_BIT];   // Bit goes high.
                                                       pwm[BLUE_BIT] = ~pwm[BLUE_BIT];
                                                       pwm[GREEN_BIT] = ~pwm[GREEN_BIT];
                                                        if(pwm[GREEN_BIT] | pwm[RED_BIT] | pwm[BLUE_BIT]) repeat(t_high) @(negedge Clock);
                                                        else               repeat(t_low) @(negedge Clock);
                                                end

                    end
                        $stop;
    end
      

endmodule
