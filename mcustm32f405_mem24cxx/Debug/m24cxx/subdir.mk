################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../m24cxx/m24cxx.c 

OBJS += \
./m24cxx/m24cxx.o 

C_DEPS += \
./m24cxx/m24cxx.d 


# Each subdirectory must supply rules for building sources it contributes
m24cxx/%.o m24cxx/%.su m24cxx/%.cyclo: ../m24cxx/%.c m24cxx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -DM24CXX_MODEL=X4M24M01 -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/lth/src/stm32world_firmware/mcustm32f405_mem24cxx/m24cxx" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-m24cxx

clean-m24cxx:
	-$(RM) ./m24cxx/m24cxx.cyclo ./m24cxx/m24cxx.d ./m24cxx/m24cxx.o ./m24cxx/m24cxx.su

.PHONY: clean-m24cxx

