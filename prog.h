
//#include <stdint.h>
//#include <string.h>
//#include "nordic_common.h"
//#include "nrf_nvmc.h"





 //y = 0,1246x + 2,0698


//void programm_init(struct set_programm *impulse_prog);
//void set_programm(struct set_programm *impulse_prog);
uint16_t us_to_tim1(float prog_time);
uint16_t us_to_tim2(uint16_t prog_time);
uint16_t tim1_to_us(uint16_t tim1_time);