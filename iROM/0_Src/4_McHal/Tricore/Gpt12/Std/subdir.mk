################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../0_Src/4_McHal/Tricore/Gpt12/Std/IfxGpt12.c 

OBJS += \
./0_Src/4_McHal/Tricore/Gpt12/Std/IfxGpt12.o 

C_DEPS += \
./0_Src/4_McHal/Tricore/Gpt12/Std/IfxGpt12.d 


# Each subdirectory must supply rules for building sources it contributes
0_Src/4_McHal/Tricore/Gpt12/Std/%.o: ../0_Src/4_McHal/Tricore/Gpt12/Std/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TriCore C Compiler'
	"$(TRICORE_TOOLS)/bin/tricore-gcc" -c -I"F:\HTws\Wire_Control_Chassis" -I"F:\HTws\Wire_Control_Chassis\0_Src" -I"F:\HTws\Wire_Control_Chassis\0_Src\0_AppSw" -I"F:\HTws\Wire_Control_Chassis\0_Src\0_AppSw\Config" -I"F:\HTws\Wire_Control_Chassis\0_Src\0_AppSw\Config\Common" -I"F:\HTws\Wire_Control_Chassis\0_Src\0_AppSw\Tricore" -I"F:\HTws\Wire_Control_Chassis\0_Src\0_AppSw\Tricore\Cfg_Illd" -I"F:\HTws\Wire_Control_Chassis\0_Src\0_AppSw\Tricore\Main" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\_Utilities" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\If" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\If\Ccu6If" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\Platform" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\Platform\Tricore" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\Platform\Tricore\Compilers" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\StdIf" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\SysSe" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\SysSe\Bsp" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\SysSe\Comm" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\SysSe\General" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\SysSe\Math" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\SysSe\Time" -I"F:\HTws\Wire_Control_Chassis\0_Src\1_SrvSw\SysSe\VirtualDevice" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\_Build" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\_Impl" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\_Lib" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\_Lib\DataHandling" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\_Lib\InternalMux" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\_PinMap" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\_Reg" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Asclin" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Asclin\Asc" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Asclin\Lin" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Asclin\Spi" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Asclin\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Ccu6" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Ccu6\Icu" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Ccu6\PwmBc" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Ccu6\PwmHl" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Ccu6\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Ccu6\Timer" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Ccu6\TimerWithTrigger" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Ccu6\TPwm" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Cif" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Cif\Cam" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Cif\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Cpu" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Cpu\CStart" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Cpu\Irq" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Cpu\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Cpu\Trap" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Dma" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Dma\Dma" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Dma\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Dsadc" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Dsadc\Dsadc" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Dsadc\Rdc" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Dsadc\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Dts" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Dts\Dts" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Dts\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Emem" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Emem\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Eray" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Eray\Eray" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Eray\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Eth" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Eth\Phy_Pef7071" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Eth\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Fce" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Fce\Crc" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Fce\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Flash" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Flash\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gpt12" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gpt12\IncrEnc" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gpt12\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gtm" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gtm\Atom" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gtm\Atom\Pwm" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gtm\Atom\PwmHl" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gtm\Atom\Timer" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gtm\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gtm\Tim" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gtm\Tim\In" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gtm\Tom" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gtm\Tom\Pwm" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gtm\Tom\PwmHl" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gtm\Tom\Timer" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Gtm\Trig" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Hssl" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Hssl\Hssl" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Hssl\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\I2c" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\I2c\I2c" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\I2c\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Iom" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Iom\Driver" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Iom\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Msc" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Msc\Msc" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Msc\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Mtu" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Mtu\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Multican" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Multican\Can" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Multican\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Port" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Port\Io" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Port\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Psi5" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Psi5\Psi5" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Psi5\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Psi5s" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Psi5s\Psi5s" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Psi5s\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Qspi" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Qspi\SpiMaster" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Qspi\SpiSlave" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Qspi\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Scu" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Scu\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Sent" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Sent\Sent" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Sent\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Smu" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Smu\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Src" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Src\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Stm" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Stm\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Stm\Timer" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Vadc" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Vadc\Adc" -I"F:\HTws\Wire_Control_Chassis\0_Src\4_McHal\Tricore\Vadc\Std" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\0_Utilities" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\1_Config" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\0_AppSw" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\0_AppSw\Config" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\0_AppSw\Config\Tricore" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\0_AppSw\Tricore" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\0_AppSw\Tricore\Main" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\1_SrvSw" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\1_SrvSw\Tricore" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\1_SrvSw\Tricore\Compilers" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\_Impl" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Cpu" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Cpu\CStart" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Cpu\Irq" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Cpu\Std" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Cpu\Trap" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Dma" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Dma\Dma" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Dma\Std" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Port" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Port\Std" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Scu" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Scu\Std" -I"F:\HTws\Wire_Control_Chassis\0_Src\0_AppSw\Tricore\APP_driver" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Stm" -I"F:\HTws\Wire_Control_Chassis\1_ToolEnv\0_Build\9_Make\4_McHal\Tricore\Stm\Std" -I"F:\HTws\QianQin_Hightec_TC275_PORT\0_Src\0_AppSw\Tricore\APP_driver\ASW" -I"F:\HTws\QianQin_Hightec_TC275_PORT\0_Src\0_AppSw\Tricore\APP_driver\BSW" -fno-common -Os -g3 -W -Wall -Wextra -Wdiv-by-zero -Warray-bounds -Wcast-align -Wignored-qualifiers -Wformat -Wformat-security -pipe -DTRIBOARD_TC275C -fshort-double -mcpu=tc27xx -mversion-info -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


