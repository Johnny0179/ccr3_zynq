# add sources here
set(src_list 
    main.cc
    hw/can.cc
    hw/maxon.cc
    modbus/modbus.c
    modbus/modbus-tcp.c
    modbus/modbus-data.c
    )

set(elf_file ${project_name}.elf)

set(INC_DIR inc
            inc/modbus)

# add library
include_directories(${INC_DIR})
# include_directories(inc/modbus)

#add sources to executable
add_executable(${elf_file} ${src_list})

# add head files
# target_include_directories(elf_file PUBLIC inc)

# link can library to project
# target_link_libraries(hardware_lib PUBLIC stm32f4xx)

# show size of resulting firmware image
# add_custom_target(${elf_file}-size DEPENDS ${elf_file} COMMAND ${ARM_SIZE_EXECUTABLE} ${elf_file})