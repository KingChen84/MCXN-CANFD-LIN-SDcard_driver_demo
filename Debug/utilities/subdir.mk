################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../utilities/fsl_assert.c 

S_UPPER_SRCS += \
../utilities/fsl_memcpy.S 

C_DEPS += \
./utilities/fsl_assert.d 

OBJS += \
./utilities/fsl_assert.o \
./utilities/fsl_memcpy.o 


# Each subdirectory must supply rules for building sources it contributes
utilities/%.o: ../utilities/%.c utilities/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MCXN947VDF -DCPU_MCXN947VDF_cm33 -DCPU_MCXN947VDF_cm33_core0 -DMCUXPRESSO_SDK -DSDK_DEBUGCONSOLE=1 -DMCUX_META_BUILD -DUSE_RTOS=0 -DSD_ENABLED=1 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\source" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\drivers" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\CMSIS" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\CMSIS\m-profile" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\device" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\device\periph" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\utilities" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\component\lists" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\utilities\str" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\utilities\debug_console_lite" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\component\gpio" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\component\uart" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\component\osa\config" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\component\osa" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\fatfs\source" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\fatfs\source\fsl_sd_disk" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\sdmmc\common" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\sdmmc\osa" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\sdmmc\sd" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\sdmmc\host\usdhc" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\board" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\source\template\sd" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\source\template" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\sdmmc\template\usdhc" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -fno-builtin -imacros "D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\source\mcux_config.h" -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

utilities/%.o: ../utilities/%.S utilities/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU Assembler'
	arm-none-eabi-gcc -c -x assembler-with-cpp -D__REDLIB__ -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\source" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\drivers" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\CMSIS" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\CMSIS\m-profile" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\device" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\device\periph" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\utilities" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\component\lists" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\utilities\str" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\utilities\debug_console_lite" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\component\gpio" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\component\uart" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\component\osa\config" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\component\osa" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\fatfs\source" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\fatfs\source\fsl_sd_disk" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\sdmmc\common" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\sdmmc\osa" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\sdmmc\sd" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\sdmmc\host\usdhc" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\source\template\sd" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\source\template" -I"D:\MCUXpresso-Project\frdmmcxn947_CANFD-LIN-Log\sdmmc\template\usdhc" -g3 -gdwarf-4 -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-utilities

clean-utilities:
	-$(RM) ./utilities/fsl_assert.d ./utilities/fsl_assert.o ./utilities/fsl_memcpy.o

.PHONY: clean-utilities

