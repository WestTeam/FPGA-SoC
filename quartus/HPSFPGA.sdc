
create_clock -period "50 MHz" -name {FPGA_CLK1_50} {FPGA_CLK1_50}
create_clock -period "50 MHz" -name {FPGA_CLK2_50} {FPGA_CLK2_50}
create_clock -period "50 MHz" -name {FPGA_CLK3_50} {FPGA_CLK3_50}


derive_pll_clocks 
derive_clock_uncertainty
