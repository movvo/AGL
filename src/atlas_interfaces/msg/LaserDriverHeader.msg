#Message to send the header recived of the omron os32c laser scan when doing a scan
#For more info see omron os32c z336_os32c-dm_users_manual_en.pdf datasheet
#Table 8: Common Measurment Report Header Format (56 bytes)


std_msgs/Header header #Standard ROS2 header

uint32 scan_count                   # 0 to 4294967298
uint32 scan_rate  
uint32 scan_time_stamp
uint32 scan_beam_period
uint16 machine_state                # 0 = POST = Power on self test
                                    # 1 = STOP = Machine Stop, Protection Zone Violation
                                    # 2 = INTERLOCK = Start Interlock
                                    # 3 = RUN = Machine Run
                                    # 4 = SANTBY = System Standby
                                    # 6 = CONFIGURE = Configuration process in session
                                    # 7 = FAULT = System Faulted, OSSD's are OFF

uint16 machine_stop_reasons         # 0 = DIRTY WINDOW
                                    # 1 = ZONE_SELECT_NUM_ACTIVE
                                    # 2 = ZONE_SELECT_INVALID
                                    # 3 = QUALIFIED_ZONE_VIOLATION
                                    # 4 = STAN_BY_REQUEST
                                    # 5 = RX_ERROR
                                    # 6 = TOO_CLEAN_WINDOW
                                    # 7 = NOT IN MACHINE STOP_1
                                    # 8 = SYSTEM FAULTED
                                    # 9 = NOT IN MACHINE STOP_2

uint16 active_protection_zone_set   # AZS_01 = (0x00)
                                    # ...
                                    # AZS_70 = (0x45)
                                    # INVALID AZS = 32,768 (0x8000)

uint16 zone_inputs                  # Zinput 1 = bit 0
                                    # Zinput 2 = bit 1
                                    # Zinput 3 = bit 2
                                    # ...
                                    # Zinput 8 = bit 7
                                
uint16 detection_zone_status        # Protection Zone = bit 0
                                    # Warning Zone 1 = bit 1
                                    # Warning Zone 2 = bit 2
                                    # Window Contamination = bit 3

uint16 output_status                # OSSD Output = bit 0
                                    # Auxiliary Output = bit 1
                                    # Warning Output = bit 2

uint16 input_status                 # StandBy Input = bit 0
                                    # Start Input = bit 1
                                    # EDM Input = bit 2
                    
uint16 seven_segment_display        # Digit Low followed by Digit High
                                    # Each Digit Displays 1 to 9 for normal operation
                                    # 0x1B and 0x1B for dashes "-" during machine stop operation

uint16 non_safety_configuration_checksum_value
uint16 safety_configuration_checksum_value

uint16 range_report_format
uint16 reflectivity_report_format

uint16 number_of_beams