`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/24/2023 01:42:13 PM
// Design Name: 
// Module Name: WDT
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


module watchdog(
    input clk,       // 100MHz clock signal
    input wdt_sig,   // input signal to be monitored
    output reg watchdog_out   // output signal
);

reg [20:0] count = 0;  // counter to measure 5ms interval
reg prev_signal = 0;   // previous input signal value

always @(posedge clk) begin
    if (wdt_sig != prev_signal) begin
        count <= 0;   // reset counter if input signal toggles
        watchdog_out <= 1;
    end else if (count < 500000) begin
        count <= count + 1;   // increment counter every clock cycle
    end else begin
        count <= 0;   // reset counter if 5ms interval has elapsed
        watchdog_out <= 0;      // turn off output signal if no toggling within 5ms
    end

    prev_signal <= wdt_sig;  // update previous signal value

end

endmodule