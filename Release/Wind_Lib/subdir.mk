################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Wind_Lib/Wind_Lib.c 

OBJS += \
./Wind_Lib/Wind_Lib.o 

C_DEPS += \
./Wind_Lib/Wind_Lib.d 


# Each subdirectory must supply rules for building sources it contributes
Wind_Lib/%.o: ../Wind_Lib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega644p -DF_CPU=18432000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


