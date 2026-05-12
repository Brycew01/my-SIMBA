`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/14/2023 01:04:12 PM
// Design Name: 
// Module Name: motor_driver_intf
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
`include "motor_controller_signals.vh"

module control_signal_decoder(
    // connecting to axi block
    input wire [CNTRL_VEC_WIDTH:0]     cntrl_vector_in,
    // connecting to driver
    // h-bridge inputs
    output wire [7:0] motor_duty_in,
    output wire [2:0] clk_divisor,
    output wire motor_dir,
    output wire h_bridge_en,
    // quad-decoder inputs
    output wire dec_clr, dec_en
    );
    
    // axi->driver
    assign motor_duty_in    = cntrl_vector_in[MOTOR_DUTY_IN_END:MOTOR_DUTY_IN_START];
    assign clk_divisor      = cntrl_vector_in[CLK_DIV_END:CLK_DIV_START];
    assign motor_dir        = cntrl_vector_in[MOTOR_DIR_IDX];
    assign h_bridge_en      = cntrl_vector_in[H_BRIDGE_EN_IDX];
    
    assign dec_clr          = cntrl_vector_in[DEC_CLR_IDX];
    assign dec_en           = cntrl_vector_in[DEC_EN_IDX];
endmodule