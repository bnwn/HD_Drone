#include "PN020Series.h"
#include "uart_console.h"
#include "ahrs.h"
#include "attitude_control.h"
#include "motor_control.h"
#include "common.h"

#ifdef __DEVELOP__

void uart_console_init(UART_T* _uart, uint32_t _baudrate)
{
    UART_Open(_uart, _baudrate);
	
	    /* Enable Interrupt and install the call back function */
    //UART_ENABLE_INT(_uart, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
		UART_ENABLE_INT(_uart, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
	
		if (_uart == UART0) {
				NVIC_EnableIRQ(UART0_IRQn);
				//NVIC_SetPriority(UART0_IRQn, 0);
		} else {
				NVIC_EnableIRQ(UART1_IRQn);
				//NVIC_SetPriority(UART1_IRQn, 0);
		}
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
uint16_t buf_index = 0;
uint8_t rev_buf[CONSOLE_BUF_SIZE];
void UART0_IRQHandler(void)
{
	  uint8_t u8InChar=0xFF;
    uint32_t u32IntSts= UART0->INTSTS;
		static bool command_receive_complete = false;
		static bool command_receive_start = false;

    if(u32IntSts & UART_INTSTS_RDAINT_Msk) {
        /* Get all the input characters */
        while(UART_IS_RX_READY(UART0)) {
            /* Get the character from UART Buffer */
            //u8InChar = UART_READ(UART0);           /* Rx trigger level is 1 byte*/
						if(!UART_Read(UART0, &u8InChar, 1)) continue;
						
						if (command_receive_start) {
								if (u8InChar == ASCII_REC_END) {
										command_receive_complete = true;
								}
								rev_buf[buf_index] = u8InChar;
								buf_index++;
						} else if (u8InChar == ASCII_REC_START) {
								command_receive_start = true;
						}
					
            /* Check if buffer full */
            if(buf_index >= CONSOLE_BUF_SIZE) {
                buf_index = 0;
								memset(rev_buf, 0, CONSOLE_BUF_SIZE);
								command_receive_start = false;
								command_receive_complete = false;
            }
        }
    }
				
		if (command_receive_complete) {
//				printf("%s\n", rev_buf);
				if (!console_input_handle()) 
						printf ("unknow command!\n");
				buf_index = 0;
				memset(rev_buf, 0, CONSOLE_BUF_SIZE);
				command_receive_start = false;
				command_receive_complete = false;
		}
				
    if(u32IntSts & UART_INTSTS_THREINT_Msk) {
        // TODO
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
bool console_input_handle(void)
{	
		uint32_t tmp_buf = {0};
		uint16_t tmp_index = 0;
		
		tmp_buf = (rev_buf[tmp_index] << 16) | (rev_buf[tmp_index+1] << 8) | rev_buf[tmp_index+2];
		if (tmp_buf == ASCII_SET) {
				tmp_index += 3;
				tmp_index += 1; // space
				tmp_buf = (rev_buf[tmp_index] << 24) | (rev_buf[tmp_index+1] << 16) | (rev_buf[tmp_index+2] << 8) | rev_buf[tmp_index+3];
			
				if (tmp_buf == ASCII_AHRS) {
						tmp_index += 4;
						tmp_index += 1; // space
						if (rev_buf[tmp_index] == ASCII_P) {
								float _data = 0.0f;
								tmp_index += 2;
								_data = get_float(tmp_index);
							
								printf("set ahrs success.\n        kp: %.6f\n", _data);
								AHRS_set_complementary_filter_kp(_data);
							
								return true;
						} else if (rev_buf[tmp_index] == ASCII_I) {
								float _data = 0.0f;
								tmp_index += 2;
								_data = get_float(tmp_index);
								printf("set ahrs success.\n        ki: %.6f\n", _data);
								AHRS_set_complementary_filter_ki(_data);
							
								return true;
						}	
				} else if ((rev_buf[tmp_index] == ASCII_P) && (rev_buf[tmp_index+1] == ASCII_I) && (rev_buf[tmp_index+2] == ASCII_D)) { // SET PID
						tmp_index += 4;
						tmp_buf = 0x00ffffff & ((rev_buf[tmp_index] << 16) | (rev_buf[tmp_index+1] <<8) | rev_buf[tmp_index+2]);
						tmp_index +=4;
						
						switch (tmp_buf) {
								case ASCII_ANG:
										tmp_buf = 0x00ffffff & ((rev_buf[tmp_index] << 16) | (rev_buf[tmp_index+1] <<8) | rev_buf[tmp_index+2]);
										tmp_index +=4;;
										switch (tmp_buf) {
												case ASCII_RLL:
														if (rev_buf[tmp_index] == ASCII_P) {
																float _data = 0.0f;
																tmp_index += 2;
																
																_data = get_float(tmp_index);
																set_pid_param_p(&ctrl_loop.angle.roll, _data);
																set_pid_param_p(&ctrl_loop.angle.pitch, _data);
																printf("set anlge loop roll kp: %.6f\n", _data);
																return true;
															
														} else if (rev_buf[tmp_index] == ASCII_I) {
																float _data = 0.0f;
																tmp_index += 2;
																
																_data = get_float(tmp_index);
																set_pid_param_i(&ctrl_loop.angle.roll, _data);
																set_pid_param_i(&ctrl_loop.angle.pitch, _data);
																printf("set anlge loop roll ki: %.6f\n", _data);
																return true;
															
														} else if (rev_buf[tmp_index] == ASCII_D) {
																float _data = 0.0f;
																tmp_index += 2;
																
																_data = get_float(tmp_index);
																set_pid_param_d(&ctrl_loop.angle.roll, _data);
																set_pid_param_d(&ctrl_loop.angle.pitch, _data);
																printf("set anlge loop roll kd: %.6f\n", _data);
																return true;
														}
														break;
												case ASCII_PTH:
														if (rev_buf[tmp_index] == ASCII_P) {
																float _data = 0.0f;
																tmp_index += 2;
																
																_data = get_float(tmp_index);
																set_pid_param_p(&ctrl_loop.angle.pitch, _data);
																printf("set anlge loop pitch kp: %.6f\n", _data);
																return true;
															
														} else if (rev_buf[tmp_index] == ASCII_I) {
																float _data = 0.0f;
																tmp_index += 2;
																
																_data = get_float(tmp_index);
																set_pid_param_i(&ctrl_loop.angle.pitch, _data);
																printf("set anlge loop pitch ki: %.6f\n", _data);
																return true;
															
														} else if (rev_buf[tmp_index] == ASCII_D) {
																float _data = 0.0f;
																tmp_index += 2;
																
																_data = get_float(tmp_index);
																set_pid_param_d(&ctrl_loop.angle.pitch, _data);
																printf("set anlge loop pitch kd: %.6f\n", _data);
																return true;
														}
														break;
												case ASCII_YAW:
														if (rev_buf[tmp_index] == ASCII_P) {
																float _data = 0.0f;
																tmp_index += 2;
																
																_data = get_float(tmp_index);
																set_pid_param_p(&ctrl_loop.angle.yaw, _data);
																printf("set anlge loop yaw kp: %.6f\n", _data);
																return true;
															
														} else if (rev_buf[tmp_index] == ASCII_I) {
																float _data = 0.0f;
																tmp_index += 2;
																
																_data = get_float(tmp_index);
																set_pid_param_i(&ctrl_loop.angle.yaw, _data);
																printf("set anlge loop yaw ki: %.6f\n", _data);
																return true;
															
														} else if (rev_buf[tmp_index] == ASCII_D) {
																float _data = 0.0f;
																tmp_index += 2;
																
																_data = get_float(tmp_index);
																set_pid_param_d(&ctrl_loop.angle.yaw, _data);
																printf("set anlge loop yaw kd: %.6f\n", _data);
																return true;
														}
														break;
												default:
														break;
										}
										break;
							case ASCII_RAT:
									tmp_buf = 0x00ffffff & ((rev_buf[tmp_index] << 16) | (rev_buf[tmp_index+1] <<8) | rev_buf[tmp_index+2]);
									tmp_index +=4;;
									switch (tmp_buf) {
											case ASCII_RLL:
													if (rev_buf[tmp_index] == ASCII_P) {
															float _data = 0.0f;
															tmp_index += 2;
															
															_data = get_float(tmp_index);
															set_pid_param_p(&ctrl_loop.rate.roll, _data);
															set_pid_param_p(&ctrl_loop.rate.pitch, _data);
															printf("set rate loop roll kp: %.6f\n", _data);
															return true;
														
													} else if (rev_buf[tmp_index] == ASCII_I) {
															float _data = 0.0f;
															tmp_index += 2;
															
															_data = get_float(tmp_index);
															set_pid_param_i(&ctrl_loop.rate.roll, _data);
															set_pid_param_i(&ctrl_loop.rate.pitch, _data);
															printf("set rate loop roll ki: %.6f\n", _data);
															return true;
														
													} else if (rev_buf[tmp_index] == ASCII_D) {
															float _data = 0.0f;
															tmp_index += 2;
															
															_data = get_float(tmp_index);
															set_pid_param_d(&ctrl_loop.rate.roll, _data);
															set_pid_param_d(&ctrl_loop.rate.pitch, _data);
															printf("set rate loop roll kd: %.6f\n", _data);
															return true;
													}
													break;
											case ASCII_PTH:
													if (rev_buf[tmp_index] == ASCII_P) {
															float _data = 0.0f;
															tmp_index += 2;
															
															_data = get_float(tmp_index);
															set_pid_param_p(&ctrl_loop.rate.pitch, _data);
															printf("set rate loop pitch kp: %.6f\n", _data);
															return true;
														
													} else if (rev_buf[tmp_index] == ASCII_I) {
															float _data = 0.0f;
															tmp_index += 2;
															
															_data = get_float(tmp_index);
															set_pid_param_i(&ctrl_loop.rate.pitch, _data);
															printf("set rate loop pitch ki: %.6f\n", _data);
															return true;
														
													} else if (rev_buf[tmp_index] == ASCII_D) {
															float _data = 0.0f;
															tmp_index += 2;
															
															_data = get_float(tmp_index);
															set_pid_param_d(&ctrl_loop.rate.pitch, _data);
															printf("set rate loop pitch kd: %.6f\n", _data);
															return true;
													}
													break;
											case ASCII_YAW:
													if (rev_buf[tmp_index] == ASCII_P) {
															float _data = 0.0f;
															tmp_index += 2;
															
															_data = get_float(tmp_index);
															set_pid_param_p(&ctrl_loop.rate.yaw, _data);
															printf("set rate loop yaw kp: %.6f\n", _data);
															return true;
														
													} else if (rev_buf[tmp_index] == ASCII_I) {
															float _data = 0.0f;
															tmp_index += 2;
															
															_data = get_float(tmp_index);
															set_pid_param_i(&ctrl_loop.rate.yaw, _data);
															printf("set rate loop yaw ki: %.6f\n", _data);
															return true;
														
													} else if (rev_buf[tmp_index] == ASCII_D) {
															float _data = 0.0f;
															tmp_index += 2;
															
															_data = get_float(tmp_index);
															set_pid_param_d(&ctrl_loop.rate.yaw, _data);
															printf("set rate loop yaw kd: %.6f\n", _data);
															return true;
													}
													break;
											default:
													break;
									}
									break;
						default:
									break;
						}
				} else if ((rev_buf[tmp_index] == ASCII_A) && (rev_buf[tmp_index+1] == ASCII_T) && (rev_buf[tmp_index+2] == ASCII_T)) { // set attitude
						float _data = 0.0f;
						tmp_index += 4;
						tmp_buf = 0x00ffffff & ((rev_buf[tmp_index] << 16) | (rev_buf[tmp_index+1] <<8) | rev_buf[tmp_index+2]);
						tmp_index +=4;
						switch (tmp_buf) {
								case ASCII_ANG:
										tmp_buf = 0x00ffffff & ((rev_buf[tmp_index] << 16) | (rev_buf[tmp_index+1] <<8) | rev_buf[tmp_index+2]);
										tmp_index +=4;
								
										switch (tmp_buf) {
												case ASCII_RLL: 
														_data = get_float(tmp_index);
														attitude_target_ang.roll = _data;
														printf("set roll angle : %.6f\n", _data);
														return true;
														break;
												case ASCII_PTH:
														_data = get_float(tmp_index);
														attitude_target_ang.pitch = _data;
														printf("set pitch angle : %.6f\n", _data);
														return true;
														break;
												case ASCII_YAW:
														_data = get_float(tmp_index);
														attitude_target_ang.yaw = _data;
														printf("set yaw angle : %.6f\n", _data);
														return true;
														break;
												default:
														break;
										}
										break;
								case ASCII_RAT:
										tmp_buf = 0x00ffffff & ((rev_buf[tmp_index] << 16) | (rev_buf[tmp_index+1] <<8) | rev_buf[tmp_index+2]);
										tmp_index +=4;
								
										switch (tmp_buf) {
												case ASCII_RLL: 
														_data = get_float(tmp_index);
														attitude_target_ang_vel.roll = _data;
														printf("set roll rate : %.6f\n", _data);
														return true;
														break;
												case ASCII_PTH:
														_data = get_float(tmp_index);
														attitude_target_ang_vel.pitch = _data;
														printf("set pitch rate : %.6f\n", _data);
														return true;
														break;
												case ASCII_YAW:
														_data = get_float(tmp_index);
														attitude_target_ang_vel.yaw = _data;
														printf("set yaw rate : %.6f\n", _data);
														return true;
														break;
												default:
														break;
										}
										break;
								
								default:
										break;
						}
				} else if ((rev_buf[tmp_index] == ASCII_T) && (rev_buf[tmp_index+1] == ASCII_H) && (rev_buf[tmp_index+2] == ASCII_R)) { // set throttle
						float _data = 0.0f;
						tmp_index += 4;
						_data = get_float(tmp_index);
						set_trace_throttle(_data);
						printf("set throttle thrust: %.6f\n", _data);
						return true;
				}
		}
		
		return false;
}

float get_float(uint16_t _buf_index)
{
		float tmp = 0;
		float e = 0.1;
		bool demical_flag = false;
		uint8_t byte;
		while (((rev_buf[_buf_index] <= 0x39) && (rev_buf[_buf_index] >= 0x30)) || (rev_buf[_buf_index] == ASCII_RADIX)) {
			byte = rev_buf[_buf_index];
			
			if (byte == ASCII_RADIX) {
					demical_flag = true;
					_buf_index += 1;
					byte = rev_buf[_buf_index];
			}
			
			byte -= 0x30;
			if (demical_flag) {
					tmp += (byte * e);
					e *= 0.1;
			} else {
					tmp *= 10;
					tmp += byte;
			}
			_buf_index += 1;
		}
		
		return tmp;
}

#endif
