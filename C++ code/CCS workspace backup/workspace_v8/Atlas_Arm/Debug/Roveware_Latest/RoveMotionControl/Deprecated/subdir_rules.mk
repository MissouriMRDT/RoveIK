################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Roveware_Latest/RoveMotionControl/Deprecated/DynamixelController.obj: ../Roveware_Latest/RoveMotionControl/Deprecated/DynamixelController.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/Chris/Documents/Roveboard" --include_path="C:/Users/Chris/workspace_v8/Atlas_Arm" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.1.LTS/include" --define=ccs="ccs" --define=PART_TM4C129ENCPDT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="Roveware_Latest/RoveMotionControl/Deprecated/DynamixelController.d_raw" --obj_directory="Roveware_Latest/RoveMotionControl/Deprecated" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

Roveware_Latest/RoveMotionControl/Deprecated/Sdc2130.obj: ../Roveware_Latest/RoveMotionControl/Deprecated/Sdc2130.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.1.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="C:/Users/Chris/Documents/Roveboard" --include_path="C:/Users/Chris/workspace_v8/Atlas_Arm" --include_path="C:/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.1.LTS/include" --define=ccs="ccs" --define=PART_TM4C129ENCPDT -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="Roveware_Latest/RoveMotionControl/Deprecated/Sdc2130.d_raw" --obj_directory="Roveware_Latest/RoveMotionControl/Deprecated" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


