################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Wind_Lib/subdir.mk
-include UART/subdir.mk
-include Interrupt_Lib/subdir.mk
-include AS3935/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
Stacja_meteo_SHT11.lss \

FLASH_IMAGE += \
Stacja_meteo_SHT11.hex \

EEPROM_IMAGE += \
Stacja_meteo_SHT11.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: Stacja_meteo_SHT11.elf secondary-outputs

# Tool invocations
Stacja_meteo_SHT11.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,Stacja_meteo_SHT11.map -mmcu=atmega644p -o "Stacja_meteo_SHT11.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Stacja_meteo_SHT11.lss: Stacja_meteo_SHT11.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S Stacja_meteo_SHT11.elf  >"Stacja_meteo_SHT11.lss"
	@echo 'Finished building: $@'
	@echo ' '

Stacja_meteo_SHT11.hex: Stacja_meteo_SHT11.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex Stacja_meteo_SHT11.elf  "Stacja_meteo_SHT11.hex"
	@echo 'Finished building: $@'
	@echo ' '

Stacja_meteo_SHT11.eep: Stacja_meteo_SHT11.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex Stacja_meteo_SHT11.elf  "Stacja_meteo_SHT11.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Stacja_meteo_SHT11.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega644p Stacja_meteo_SHT11.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) Stacja_meteo_SHT11.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
