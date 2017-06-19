################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Generated_Code/Cpu.c \
../Generated_Code/Vectors.c \
../Generated_Code/Vectors_Backup.c 

OBJS += \
./Generated_Code/Cpu.o \
./Generated_Code/Vectors.o \
./Generated_Code/Vectors_Backup.o 

C_DEPS += \
./Generated_Code/Cpu.d \
./Generated_Code/Vectors.d \
./Generated_Code/Vectors_Backup.d 


# Each subdirectory must supply rules for building sources it contributes
Generated_Code/%.o: ../Generated_Code/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:\Users\12195032\Documents\PROJECT\Project\Library" -I"C:/Users/12195032/Documents/PROJECT/Project/Static_Code/IO_Map" -I"C:/Users/12195032/Documents/PROJECT/Project/Sources" -I"C:/Users/12195032/Documents/PROJECT/Project/Generated_Code" -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


