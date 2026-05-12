`timescale 1ns / 1ps

module quadDecoder #(parameter WIDTH=16) (
    input enc_a,
    input enc_b,
    input clear,
    input clk,
    input en,
    output [WIDTH-1:0] count
);

reg A, B, A_FF, B_FF;

// negative logic for encoders
assign neg_enc_a = ~enc_a;
assign neg_enc_b = ~enc_b;

// if A is active now, but B was active last clock cycle, moving up...
assign counter_up = A ^ B_FF;

// if B is active now, but A was active last clock cycle, moving up...
assign counter_down = B ^ A_FF;

// counter increments or decrements if the encoder is moving, and decoder is enabled
assign counter_en = (counter_up ^ counter_down) && en;

// flopping input signals
always @(posedge clk)
begin
     A <= neg_enc_a; 
     B <= neg_enc_b; 
     A_FF <= A; 
     B_FF <= B; 
end 

// Count the reletive position of the encoders with a counter
cntr_udclr_nb #(.n(WIDTH)) cntr (
    .clk(clk),
    .clr(clear),
    .up(counter_up),
    .dn(counter_down),
    .en(counter_en),
    .ld(0),
    .D(0),
    .count(count)
);

endmodule
