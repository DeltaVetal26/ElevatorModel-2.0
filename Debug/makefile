################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
MainBoardFirmware.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MainBoardFirmware.list \

OBJCOPY_BIN += \
MainBoardFirmware.bin \


# All Target
all: MainBoardFirmware.elf secondary-outputs

# Tool invocations
MainBoardFirmware.elf: $(OBJS) $(USER_OBJS) C:\Users\alexm\STM32CubeIDE\workspace_1.2.0\MainBoardFirmware\STM32F334K8TX_FLASH.ld
	arm-none-eabi-gcc -o "MainBoardFirmware.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\alexm\STM32CubeIDE\workspace_1.2.0\MainBoardFirmware\STM32F334K8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="MainBoardFirmware.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MainBoardFirmware.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MainBoardFirmware.list"
	@echo 'Finished building: $@'
	@echo ' '

MainBoardFirmware.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "MainBoardFirmware.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
