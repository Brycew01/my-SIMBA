`timescale 1ns / 1ps

module PWM_Generator(
 input clk,
 input [7:0] duty_in,
 input [2:0] clk_divisor,
 input en,
 
 output pwm
);

 reg PWM_OUT = 0;
 reg[15:0] clk_cntr=0;
 reg[7:0] pwm_cntr=0;
 reg[7:0] pwm_duty=0;
 
 always @(posedge clk)
 begin
   clk_cntr <= clk_cntr + 1;
   pwm_duty <= duty_in;
   PWM_OUT <= (pwm_cntr < pwm_duty);
 end
 
 assign clk_div = clk_cntr[clk_divisor];
 
 always @(posedge clk_div)
 begin
    pwm_cntr <= pwm_cntr+1;
 end 
 
 assign pwm = en & PWM_OUT;
 
endmodule
