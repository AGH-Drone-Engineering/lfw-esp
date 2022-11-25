idf_component_register(
    SRCS
        main.c
        line_sensor.c
        motors.c
        turn_pid.c
        control_loop.c
        wifi_man.c
        tcp_server.c
        cmd_parser.c
        turbine.c
    INCLUDE_DIRS .
)
