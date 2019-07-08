# add sources here
set(src_list 
    main.cc
    hw/can.cc
    robot/robot.cc
    robot/maxon.cc
    freemodbus_tcp/freemodbus_tcp.c
    )

#add freemodbus demo source
# set(freemodbus 
#     demo.c)

set(elf_file ${project_name}.elf)



set(INC_DIR inc
            inc/freemodbus_tcp
            )

# add library
include_directories(${INC_DIR})

#add sources to executable
add_executable(${elf_file} ${src_list})

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