idf_component_register(SRCS "main.c"
                            
                            "input_task/radar_manager.c"
                            "input_task/input_task.c"

                            "uart_task/radar_uart.c"
                            "uart_task/radar_uart_task.c"

                            "wifi_task/UDP_clinet.c"

                            "communication_protocol/mod_bus.c"

                            "steering_task/steering_task.c"

                       INCLUDE_DIRS "uart_task"
                                    "input_task"
                                    "wifi_task"
                                    "steering_task"
                                    "communication_protocol"
                                    )