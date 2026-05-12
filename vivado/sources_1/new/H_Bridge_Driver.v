`timescale 1ns / 1ps

module H_Bridge_Driver(
    input clk,
    input [7:0] duty_in,
    input [2:0] clk_divisor,
    input dir,
    input en,
    
    output out1,
    output out2
    );
    
    wire pwm;
    
    PWM_Generator pwm_gen(
        .clk (clk), 
        .duty_in (duty_in),
        .clk_divisor (clk_divisor),
        .en (en),
        .pwm (pwm)
       );
    
    assign out1 = ((~pwm) | dir) & en;
    assign out2 = ~(pwm & dir) & en;
    
endmodule
