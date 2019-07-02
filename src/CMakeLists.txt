# add sources here
set(src_list 
    main.cc
    hw/can.cc)

set(elf_file ${project_name}.elf)

# add library
include_directories(inc)

#add sources to executable
add_executable(${elf_file} ${src_list})

# add head files
# target_include_directories(elf_file PUBLIC inc)

# link can library to project
# target_link_libraries(hardware_lib PUBLIC stm32f4xx)

# show size of resulting firmware image
# add_custom_target(${elf_file}-size DEPENDS ${elf_file} COMMAND ${ARM_SIZE_EXECUTABLE} ${elf_file})