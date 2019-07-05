# add sources here
# set(src_list 
#     main.cc
#     hw/can.cc
#     hw/maxon.cc
#     libmodbus/modbus.c
#     libmodbus/modbus-tcp.c
#     libmodbus/modbus-data.c
#     )

#add freemodbus demo source
set(freemodbus 
    demo.c)

set(elf_file ${project_name}.elf)



set(INC_DIR inc
            inc/libmodbus)

# add library
include_directories(${INC_DIR})
# include_directories(inc/modbus)

#add sources to executable
add_executable(${elf_file} ${freemodbus})

# link freemodbus library to project
target_link_libraries(${elf_file} PUBLIC freemodbus_lib)

# add pthread support
find_package(Threads)
target_link_libraries(${elf_file} PUBLIC ${CMAKE_THREAD_LIBS_INIT})

# add head files
# target_include_directories(elf_file PUBLIC inc)

# link can library to project
# target_link_libraries(hardware_lib PUBLIC stm32f4xx)

# show size of resulting firmware image
# add_custom_target(${elf_file}-size DEPENDS ${elf_file} COMMAND ${ARM_SIZE_EXECUTABLE} ${elf_file})