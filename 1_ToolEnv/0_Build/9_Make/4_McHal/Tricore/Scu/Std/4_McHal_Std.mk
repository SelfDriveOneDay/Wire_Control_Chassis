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
# Subdir make file for 0_Src/4_McHal/Tricore/Scu/Std
# This is for core type MAIN and Tasking compiler !!!!
###############################################################################




# current directory relative to project
L_DIR:=0_Src/4_McHal/Tricore/Scu/Std								
														
OBJ_FILES_MAIN+=2_Out/Tasking/0_Src/4_McHal/Tricore/Scu/Std/IfxScuCcu.o 2_Out/Tasking/0_Src/4_McHal/Tricore/Scu/Std/IfxScuWdt.o				
DEP_FILES_MAIN+=2_Out/Tasking/0_Src/4_McHal/Tricore/Scu/Std/IfxScuCcu.d 2_Out/Tasking/0_Src/4_McHal/Tricore/Scu/Std/IfxScuWdt.d				
														
$(OUT_DIR)/0_Src/4_McHal/Tricore/Scu/Std/%.o: 0_Src/4_McHal/Tricore/Scu/Std/%.c	
	@mkdir -p $(@D)										
	@rm -f $(ELF_BIN_MAIN) $(HEX_BIN_MAIN) $(MAP_FILE_MAIN) (LIB_OBJ_MAIN)					
	@echo 'Compiling $<'								
	$(CC) $(CC_OPTS) $< -o $(@D)/$*.src	--dep-file=$(@D)/$*.dep	
	$(AS) $(ASM_OPTS) $(@D)/$*.src -o $@				
	@echo Tasking dependency files converted to GNU format $<	
	@sed $(call TaskingDepConversionString,$(@F),$(@D)) $(@:.o=.dep) >$(@:.o=.d) 
	@rm -f $(@:.o=.dep)									
	@echo ' '											
														



