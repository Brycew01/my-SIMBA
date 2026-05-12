`ifndef MOTOR_CONTROLLER_SIGNALS_VH
`define MOTOR_CONTROLLER_SIGNALS_VH
// defining feild widths AXI->driver
parameter MOTOR_DUTY_IN_WIDTH   = 8;
parameter CLK_DIV_WIDTH         = 3;

// defining field widths driver->AXI
parameter MOTOR_POS_WIDTH       = 16;

// defining slice inputs from AXI->DRIVER
// h-brige inputs
parameter MOTOR_DUTY_IN_START   = 0;
parameter MOTOR_DUTY_IN_END     = MOTOR_DUTY_IN_START + (MOTOR_DUTY_IN_WIDTH - 1); // 7
parameter CLK_DIV_START         = MOTOR_DUTY_IN_END + 1;   // 8  
parameter CLK_DIV_END           = CLK_DIV_START + (CLK_DIV_WIDTH - 1); // 10
parameter MOTOR_DIR_IDX         = CLK_DIV_END + 1;      // 11
parameter H_BRIDGE_EN_IDX       = MOTOR_DIR_IDX + 1;    // 12
// quad-decoder inputs
parameter DEC_CLR_IDX           = H_BRIDGE_EN_IDX + 1;        // 13
parameter DEC_EN_IDX            = DEC_CLR_IDX + 1;      // 14

parameter CNTRL_VEC_WIDTH          = DEC_EN_IDX;
`endif