idf_component_register(SRCS "main.c"
                            
                            "uart_task/uart.c"
                            "uart_task/uart_task.c"

                            "input_task/radar_manager.c"

                       INCLUDE_DIRS "uart_task"
                                    "input_task"
                                    )