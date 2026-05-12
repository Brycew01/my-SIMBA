########################   RPI I2C    ########################
set_property PACKAGE_PIN AE15 [get_ports {iic_sda_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {iic_sda_io}]
set_property PULLUP true [get_ports {iic_sda_io}]

set_property PACKAGE_PIN AE14 [get_ports {iic_scl_io}]
set_property IOSTANDARD LVCMOS33 [get_ports {iic_scl_io}]
set_property PULLUP true [get_ports {iic_scl_io}]

######################## PMOD 1 Upper ########################
# pmod1[1], rear right steer motor (0), motor 0
set_property PACKAGE_PIN H12 [get_ports {motor0[0]}]        
set_property IOSTANDARD LVCMOS33 [get_ports {motor0[0]}]

# pmod1[2], rear right steer encoder (0), encoder 0
set_property PACKAGE_PIN E10 [get_ports {encoder0[0]}]        
set_property IOSTANDARD LVCMOS33 [get_ports {encoder0[0]}]

# pmod1[3], rear right wheel motor (0), motor 1
set_property PACKAGE_PIN D10 [get_ports {motor1[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor1[0]}]

# pmod1[4], rear right wheel encoder (0), encoder 1
set_property PACKAGE_PIN C11 [get_ports {encoder1[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder1[0]}]

######################## PMOD 1 Lower ########################
# pmod1[5], rear right steer motor (1), motor 0
set_property PACKAGE_PIN B10 [get_ports {motor0[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor0[1]}]

# pmod1[6], rear right steer encoder (1), encoder 0
set_property PACKAGE_PIN E12 [get_ports {encoder0[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder0[1]}]

# pmod1[7], rear right wheel motor (1), motor 1
set_property PACKAGE_PIN D11 [get_ports {motor1[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor1[1]}]

# pmod1[8], rear right wheel encoder (1), encoder 1
set_property PACKAGE_PIN B11 [get_ports {encoder1[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder1[1]}]

######################## PMOD 2 Upper ########################
# pmod2[1], front right steer motor (0), motor 2
set_property PACKAGE_PIN J11 [get_ports {motor2[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor2[0]}]

# pmod2[2], front right steer encoder (0), encoder 2
set_property PACKAGE_PIN J10 [get_ports {encoder2[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder2[0]}]

# pmod2[3], front right wheel motor (0), motor 3
set_property PACKAGE_PIN K13 [get_ports {motor3[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor3[0]}]

# pmod2[4], front right wheel encoder (0), encoder 3
set_property PACKAGE_PIN K12 [get_ports {encoder3[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder3[0]}]

######################## PMOD 2 Lower ########################
# pmod2[5], front right steer motor (1), motor 2
set_property PACKAGE_PIN H11 [get_ports {motor2[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor2[1]}]

# pmod2[6], front right steer encoder (1), encoder 2
set_property PACKAGE_PIN G10 [get_ports {encoder2[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder2[1]}]

# pmod2[7], front right wheel motor (1), motor 3
set_property PACKAGE_PIN F12 [get_ports {motor3[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor3[1]}]

# pmod2[8], front right wheel encoder (1), encoder 3
set_property PACKAGE_PIN F11 [get_ports {encoder3[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder3[1]}]

######################## PMOD 3 Upper ########################
# pmod3[1], front left steer motor (0), motor 4
set_property PACKAGE_PIN AE12 [get_ports {motor4[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor4[0]}]

# pmod3[2], front left steer encoder (0), encoder 4
set_property PACKAGE_PIN AF12 [get_ports {encoder4[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder4[0]}]

# pmod3[3], front left wheel motor (0), motor 5
set_property PACKAGE_PIN AG10 [get_ports {motor5[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor5[0]}]

# pmod3[4], front left wheel encoder (0), encoder 5
set_property PACKAGE_PIN AH10 [get_ports {encoder5[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder5[0]}]

######################## PMOD 3 Lower ########################
# pmod3[5], front left steer motor (1), motor 4
set_property PACKAGE_PIN AF11 [get_ports {motor4[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor4[1]}]

# pmod3[6], front left steer encoder (1), encoder 4
set_property PACKAGE_PIN AG11 [get_ports {encoder4[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder4[1]}]

# pmod3[7], front left wheel motor (1), motor 5
set_property PACKAGE_PIN AH12 [get_ports {motor5[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor5[1]}]

# pmod3[8], front left wheel encoder (1), encoder 5
set_property PACKAGE_PIN AH11 [get_ports {encoder5[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder5[1]}]

######################## PMOD 4 Upper ########################
# pmod4[1], rear left steer motor (0), motor 6 
set_property PACKAGE_PIN AC12 [get_ports {motor6[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor6[0]}]

# pmod4[2], rear left steer encoder (0), encoder 6
set_property PACKAGE_PIN AD12 [get_ports {encoder6[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder6[0]}]

# pmod4[3], rear left wheel motor (0), motor 7
set_property PACKAGE_PIN AE10 [get_ports {motor7[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor7[0]}]

# pmod4[4], rear left wheel encoder (0), encoder 7
set_property PACKAGE_PIN AF10 [get_ports {encoder7[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder7[0]}]

######################## PMOD 4 Lower ########################
# pmod4[5], rear left steer motor (1), motor 6
set_property PACKAGE_PIN AD11 [get_ports {motor6[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor6[1]}]

# pmod4[6], rear left steer encoder (1), encoder 6
set_property PACKAGE_PIN AD10 [get_ports {encoder6[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder6[1]}]

# pmod4[7], rear left wheel motor (1), motor 7
set_property PACKAGE_PIN AA11 [get_ports {motor7[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor7[1]}]

# pmod4[8], rear left wheel encoder (1), encoder 7
set_property PACKAGE_PIN AA10 [get_ports {encoder7[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder7[1]}]

######################## Raspberry Pi GPIO Header ########################
### AXI GPIO ###
# rpi_gpio[0], unused2, motor 11 (1)
set_property PACKAGE_PIN AD15 [get_ports {motor11[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor11[1]}]

# rpi_gpio[1], unused2, encoder 11 (0)
set_property PACKAGE_PIN AD14 [get_ports {encoder11[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder11[0]}]

#set_property PACKAGE_PIN AE15 [get_ports {rpi_gpio[2]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio[2]}]

#set_property PACKAGE_PIN AE14 [get_ports {rpi_gpio[3]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio[3]}]

#set_property PACKAGE_PIN AG14 [get_ports {rpi_gpio[4]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio[4]}]

# rpi_gpio[5], unused2, encoder 11 (1)
set_property PACKAGE_PIN AH14 [get_ports {encoder11[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder11[1]}]

# rpi_gpio[6], unused3, motor 12 (0)
set_property PACKAGE_PIN AG13 [get_ports {motor12[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor12[0]}]

# rpi_gpio[7], unused2, motor 11 (0)
set_property PACKAGE_PIN AH13 [get_ports {motor11[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor11[0]}]

# rpi_gpio[8], unused1, encoder 10 (1)
set_property PACKAGE_PIN AC14 [get_ports {encoder10[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder10[1]}]

# rpi_gpio[9], unused1 (0), motor 10
set_property PACKAGE_PIN AC13 [get_ports {motor10[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor10[0]}]

# rpi_gpio[10], middle right encoder (1), encoder 9
set_property PACKAGE_PIN AE13 [get_ports {encoder9[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder9[1]}]

# rpi_gpio[11], unused1, encoder 10 (0)
set_property PACKAGE_PIN AF13 [get_ports {encoder10[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder10[0]}]

# rpi_gpio[12], unused3, motor 12 (1)
set_property PACKAGE_PIN AA13 [get_ports {motor12[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor12[1]}]

# rpi_gpio[13], unused3, encoder 12 (0)
set_property PACKAGE_PIN AB13 [get_ports {encoder12[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder12[0]}]

#set_property PACKAGE_PIN W14 [get_ports {rpi_gpio[14]}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio[14]}]

# rpi_gpio[15], middle left wheel (0), motor 8
set_property PACKAGE_PIN W13 [get_ports {motor8[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor8[0]}]

# rpi_gpio[16], unused4, motor 13 (0)
set_property PACKAGE_PIN AB15 [get_ports {motor13[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor13[0]}]

# rpi_gpio[17], middle left wheel (1), motor 8
set_property PACKAGE_PIN AB14 [get_ports {motor8[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor8[1]}]

# rpi_gpio[18], middle left encoder (0), encooder 8
set_property PACKAGE_PIN Y14 [get_ports {encoder8[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder8[0]}]

# rpi_gpio[19], unused3, encoder 12 (1)
set_property PACKAGE_PIN Y13 [get_ports {encoder12[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder12[1]}]

# rpi_gpio[20], unused4, encoder 13 (0)
set_property PACKAGE_PIN W12 [get_ports {encoder13[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder13[0]}]

# rpi_gpio[21], unused4, encoder 13 (1)
set_property PACKAGE_PIN W11 [get_ports {encoder13[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder13[1]}]

# rpi_gpio[22], middle right motor (0), motor 9
set_property PACKAGE_PIN Y12 [get_ports {motor9[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor9[0]}]

# rpi_gpio[23], middle right motor (0), motor 9
set_property PACKAGE_PIN AA12 [get_ports {motor9[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor9[1]}]

# rpi_gpio[24], middle right encoder (0), encoder 9
set_property PACKAGE_PIN Y9 [get_ports {encoder9[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder9[0]}]

# rpi_gpio[25], unused10 (1), motor 10 
set_property PACKAGE_PIN AA8 [get_ports {motor10[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor10[1]}]

# rpi_gpio[26], unused4, motor 13 (1)
set_property PACKAGE_PIN AB10 [get_ports {motor13[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {motor13[1]}]

# rpi_gpio[27], middle left encoder (1), encoder 8
set_property PACKAGE_PIN AB9 [get_ports {encoder8[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {encoder8[1]}]

### USER LEDS (added by Curtis) ###
set_property PACKAGE_PIN F8 [get_ports { user_led1 }]
set_property IOSTANDARD LVCMOS18 [get_ports { user_led1 }]

set_property PACKAGE_PIN E8 [get_ports { user_led2 }]
set_property IOSTANDARD LVCMOS18 [get_ports { user_led2 }]
### Special Functions ###
#set_property PACKAGE_PIN AD15 [get_ports {rpi_gpio0_id_sd}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio0_id_sd}]

#set_property PACKAGE_PIN AD14 [get_ports {rpi_gpio1_id_sc}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio1_id_sc}]

#set_property PACKAGE_PIN AE15 [get_ports {rpi_gpio2_sda}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio2_sda}]

#set_property PACKAGE_PIN AE14 [get_ports {rpi_gpio3_scl}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio3_scl}]

#set_property PACKAGE_PIN AG14 [get_ports {rpi_gpio4_gpclk0}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio4_gpclk0}]

#set_property PACKAGE_PIN AH14 [get_ports {rpi_gpio5}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio5}]

#set_property PACKAGE_PIN AG13 [get_ports {rpi_gpio6}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio6}]

#set_property PACKAGE_PIN AH13 [get_ports {rpi_gpio7_ce1}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio7_ce1}]

#set_property PACKAGE_PIN AC14 [get_ports {rpi_gpio8_ce0}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio8_ce0}]

#set_property PACKAGE_PIN AC13 [get_ports {rpi_gpio9_miso}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio9_miso}]

#set_property PACKAGE_PIN AE13 [get_ports {rpi_gpio10_mosi}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio10_mosi}]

#set_property PACKAGE_PIN AF13 [get_ports {rpi_gpio11_sclk}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio11_sclk}]

#set_property PACKAGE_PIN AA13 [get_ports {rpi_gpio12_pwm0}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio12_pwm0}]

#set_property PACKAGE_PIN AB13 [get_ports {rpi_gpio13_pwm1}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio13_pwm1}]

#set_property PACKAGE_PIN W14 [get_ports {rpi_gpio14_txd}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio14_txd}]

#set_property PACKAGE_PIN W13 [get_ports {rpi_gpio15_rxd}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio15_rxd}]

#set_property PACKAGE_PIN AB15 [get_ports {rpi_gpio16}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio16}]

#set_property PACKAGE_PIN AB14 [get_ports {rpi_gpio17}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio17}]

#set_property PACKAGE_PIN Y14 [get_ports {rpi_gpio18_pcm_clk}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio18_pcm_clk}]

#set_property PACKAGE_PIN Y13 [get_ports {rpi_gpio19_pcm_fs}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio19_pcm_fs}]

#set_property PACKAGE_PIN W12 [get_ports {rpi_gpio20_pcm_din}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio20_pcm_din}]

#set_property PACKAGE_PIN W11 [get_ports {rpi_gpio21_pcm_dout}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio21_pcm_dout}]

#set_property PACKAGE_PIN Y12 [get_ports {rpi_gpio22}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio22}]

#set_property PACKAGE_PIN AA12 [get_ports {rpi_gpio23}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio23}]

#set_property PACKAGE_PIN Y9 [get_ports {rpi_gpio24}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio24}]

#set_property PACKAGE_PIN AA8 [get_ports {rpi_gpio25}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio25}]

#set_property PACKAGE_PIN AB10 [get_ports {rpi_gpio26}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio26}]

#set_property PACKAGE_PIN AB9 [get_ports {rpi_gpio27}]
#set_property IOSTANDARD LVCMOS33 [get_ports {rpi_gpio27}]