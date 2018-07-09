################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/subdir.mk
-include Library/Library/subdir.mk
-include CMSIS/CMSIS/gcc/subdir.mk
-include CMSIS/CMSIS/subdir.mk
-include .metadata/.plugins/org.eclipse.cdt.managedbuilder.core/subdir.mk
-include .metadata/.plugins/org.eclipse.cdt.make.core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
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
SECONDARY_FLASH += \
CRC_CCITT.hex \

SECONDARY_SIZE += \
CRC_CCITT.siz \


# All Target
all: CRC_CCITT.elf secondary-outputs

# Tool invocations
CRC_CCITT.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: Cross ARM GNU C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -T "C:\git\NUC126_0619\Library\Device\Nuvoton\NUC126\Source\gcc\gcc_arm.ld" -Xlinker --gc-sections -Wl,-Map,"CRC_CCITT.map" -o "CRC_CCITT.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

CRC_CCITT.hex: CRC_CCITT.elf
	@echo 'Invoking: Cross ARM GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex "CRC_CCITT.elf"  "CRC_CCITT.hex"
	@echo 'Finished building: $@'
	@echo ' '

CRC_CCITT.siz: CRC_CCITT.elf
	@echo 'Invoking: Cross ARM GNU Print Size'
	arm-none-eabi-size --format=berkeley "CRC_CCITT.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) CRC_CCITT.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
