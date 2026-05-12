`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/09/2023 01:25:06 PM
// Design Name: 
// Module Name: motor_controller
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

module motor_controller(
    // connecting to axi block
    input wire [CNTRL_VEC_WIDTH:0]     control_vector_in,
    // from watchdog
    input wire en,
    output wire [15:0]   motor_pos,
    // encoder signals
    input wire [1:0] enc_in,
    // output to h-bridge
    output wire [1:0] h_bridge_cntrl,
    input clk
    );
    
    // defining control signals
    wire [7:0] motor_duty_in;
    wire [2:0] clk_divisor;
    wire motor_dir, h_bridge_en;
    wire dec_clr, dec_en;
    
    // Decoding control vector into individual control signals
    control_signal_decoder CNTRL_DEC (
        .cntrl_vector_in         (control_vector_in),
        // connecting to driver
        // h-bridge inputs
        .motor_duty_in  (motor_duty_in),
        .clk_divisor    (clk_divisor),
        .motor_dir      (motor_dir),
        .h_bridge_en    (h_bridge_en),
        // quad-decoder inputs
        .dec_clr        (dec_clr),
        .dec_en         (dec_en)
    );
    
    // H Bridge motor controller
    H_Bridge_Driver H_BR_DRV (
        .clk            (clk),
        .duty_in        (motor_duty_in),
        .clk_divisor    (clk_divisor),
        .dir            (motor_dir),
        .en             (h_bridge_en && en), //if controller enables it, and watchdog enables it
        
        .out1           (h_bridge_cntrl[0]),
        .out2           (h_bridge_cntrl[1])
    );
    
    // Position Decoder
    quadDecoder #(.WIDTH(16)) DEC(
        .enc_a  (enc_in[0]),
        .enc_b  (enc_in[1]),
        .clear  (dec_clr),
        .clk    (clk),
        .en     (dec_en),
        .count  (motor_pos)
    );
endmodule
