###############################################################################
#                                                                             #
#        Copyright © 2011 Infineon Technologies AG. All rights reserved.      #
#                                                                             #
#                                                                             #
#                              IMPORTANT NOTICE                               #
#                                                                             #
#                                                                             #
# Infineon Technologies AG (Infineon) is supplying this file for use          #
# exclusively with Infineonís microcontroller products. This file can be      #
# freely distributed within development tools that are supporting such        #
# microcontroller products.                                                   #
#                                                                             #
# THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED #
# OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF          #
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.#
# INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,#
# OR CONSEQUENTIAL DAMAGES, FOR	ANY REASON WHATSOEVER.                        #
#                                                                             #
###############################################################################
# Subdir make file for 0_Src/4_McHal/Tricore/Cpu
# This is for core type MAIN and Tasking compiler !!!!
###############################################################################


include 1_ToolEnv/0_Build/9_Make/4_McHal/Tricore/Cpu/CStart/4_McHal_CStart.mk 1_ToolEnv/0_Build/9_Make/4_McHal/Tricore/Cpu/Irq/4_McHal_Irq.mk 1_ToolEnv/0_Build/9_Make/4_McHal/Tricore/Cpu/Std/4_McHal_Std.mk 1_ToolEnv/0_Build/9_Make/4_McHal/Tricore/Cpu/Trap/4_McHal_Trap.mk  

# current directory relative to project
L_DIR:=0_Src/4_McHal/Tricore/Cpu								



