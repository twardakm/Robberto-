################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
Robberto.lss \

FLASH_IMAGE += \
Robberto.hex \

EEPROM_IMAGE += \
Robberto.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: Robberto.elf secondary-outputs

# Tool invocations
Robberto.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,Robberto.map -mmcu=atmega8 -o "Robberto.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Robberto.lss: Robberto.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S Robberto.elf  >"Robberto.lss"
	@echo 'Finished building: $@'
	@echo ' '

Robberto.hex: Robberto.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex Robberto.elf  "Robberto.hex"
	@echo 'Finished building: $@'
	@echo ' '

Robberto.eep: Robberto.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex Robberto.elf  "Robberto.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Robberto.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega8 Robberto.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) Robberto.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
