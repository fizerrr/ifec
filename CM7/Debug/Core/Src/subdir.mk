################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/adc.c \
../Core/Src/adc_config.c \
../Core/Src/controll.c \
../Core/Src/dma.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/no_load_controller.c \
../Core/Src/ocp.c \
../Core/Src/pi_controller.c \
../Core/Src/pi_controller_current.c \
../Core/Src/power_on.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/tim.c 

OBJS += \
./Core/Src/adc.o \
./Core/Src/adc_config.o \
./Core/Src/controll.o \
./Core/Src/dma.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/no_load_controller.o \
./Core/Src/ocp.o \
./Core/Src/pi_controller.o \
./Core/Src/pi_controller_current.o \
./Core/Src/power_on.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/tim.o 

C_DEPS += \
./Core/Src/adc.d \
./Core/Src/adc_config.d \
./Core/Src/controll.d \
./Core/Src/dma.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/no_load_controller.d \
./Core/Src/ocp.d \
./Core/Src/pi_controller.d \
./Core/Src/pi_controller_current.d \
./Core/Src/power_on.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/tim.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DSTM32H755xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=25000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=64000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/adc_config.cyclo ./Core/Src/adc_config.d ./Core/Src/adc_config.o ./Core/Src/adc_config.su ./Core/Src/controll.cyclo ./Core/Src/controll.d ./Core/Src/controll.o ./Core/Src/controll.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/no_load_controller.cyclo ./Core/Src/no_load_controller.d ./Core/Src/no_load_controller.o ./Core/Src/no_load_controller.su ./Core/Src/ocp.cyclo ./Core/Src/ocp.d ./Core/Src/ocp.o ./Core/Src/ocp.su ./Core/Src/pi_controller.cyclo ./Core/Src/pi_controller.d ./Core/Src/pi_controller.o ./Core/Src/pi_controller.su ./Core/Src/pi_controller_current.cyclo ./Core/Src/pi_controller_current.d ./Core/Src/pi_controller_current.o ./Core/Src/pi_controller_current.su ./Core/Src/power_on.cyclo ./Core/Src/power_on.d ./Core/Src/power_on.o ./Core/Src/power_on.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su

.PHONY: clean-Core-2f-Src

