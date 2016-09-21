################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Interrupt_Lib/Interrupt_Lib.c 

OBJS += \
./Interrupt_Lib/Interrupt_Lib.o 

C_DEPS += \
./Interrupt_Lib/Interrupt_Lib.d 


# Each subdirectory must supply rules for building sources it contributes
Interrupt_Lib/%.o: ../Interrupt_Lib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega644p -DF_CPU=18432000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


