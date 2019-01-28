FPGA-SoC:
====

This is the project running on the DE0 Nano or DE10 Nano FPGA DevKit, including an FPGA firmware and several embeddeed softwares targetted on RISCV soft-core cpu (Orca from VectorBlox). 

This project is the central control part of WestTeam robot, it handles all interactions between low level interfaces (SPI,I2C,ADC,UART,QEI,H-Bridge, etc) and upper layers (PID, odometry, position control). 

The project also leverage the capability to receive configuration and orders from a linux based software running on the ARM HPS (hard processor system).

Repository setup:
===
* The repository is module based, after checkout, please use the following commands:

<code>
    cd <repo_base\>
    
    $git submodule update --init orca/

    $git submodule update --init orca/software/generic/cvra-modules/
</code>

* For the first checkout, you need to have installed GCC for cross compilation targetting RISCV. Please follow tutorial here: 

<code>
    $cd <repo_base\>
    
    $cat orca/tools/README.me
</code>

Required Software:
===
  * Quartus Prime Software v16.1 Update 2 installed (to be found here: http://dl.altera.com/16.1/?edition=lite)



Compilation:
===

  * Quartus Project compilation with .sof file generation (<repo_base\>/quartus/outputs/HPSFPGA.sof for DE0 and <repo_base\>/quartus/outputs/DE10.sof for DE10):

<code>
    cd <repo_base\>/quartus

    $[QUARTUS_INSTALLDIR]nios2eds/nios2_command_shell.sh
    
    $make sof    (DE0 Nano)
    
    $make sof PROJECT_REV=DE10    (DE10 Nano)
    
</code>
    
  * DTS (device tree string) and DTB (device tree blob) generation:

<code>
    cd <repo_base\>/quartus

    $[QUARTUS_INSTALLDIR]nios2eds/nios2_command_shell.sh
    
    $make dts && make dtb
    
    DTB -> ./quartus/hps_fpga.dtb (to be renamed soc_system.dtb in boot partition) 
    
</code>

  * Orca softwares compilation with .sof internal memory update to integrate genarated RISCV cpu code, and RBF generation:

<code>
    $cd <repo_base\>
    
    $[QUARTUS_INSTALLDIR]nios2eds/nios2_command_shell.sh
    
    $./tools/orca_compile.sh
    
    DE0 Nano  -> ./quartus/outputs/soc_system.rbf
    
    DE10 Nano -> ./quartus/outputs/soc_system_de10.rbf (to be renamed soc_system.rbf in boot partition)
    
</code>