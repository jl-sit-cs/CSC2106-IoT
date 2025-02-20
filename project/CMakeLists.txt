add_executable(project
        project.c
        )

# pull in common dependencies
target_link_libraries(project pico_stdlib)

# enable usb output, enable uart output
pico_enable_stdio_usb(project 1)
pico_enable_stdio_uart(project 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(project)

# add url via pico_set_program_url
example_auto_set_url(project)
