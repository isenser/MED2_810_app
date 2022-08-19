
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"

#include "prog.h"


//struct impulse_prog {
//  uint16_t prog_id;        //
//  uint16_t imp_voltage;    //19V
//  uint16_t imp_t_pause;    //ms время между импульсами одного канала
//  uint16_t imp_t_min;      //us
//  uint16_t imp_t_max;      //us
//  uint16_t prog_time;      //sec
//  uint16_t prog_num;       //число шагов
//  uint16_t prog_time_num;  //sec време перехода к конечному уровню
//  uint16_t prog_button_mode; //по кнопкам
//  } ;
//
// //y = 0,1246x + 2,0698


uint16_t us_to_tim1(float prog_time) {
  float ret=0;
  ret = ((prog_time-2.0698)/0.1246);
  return (uint16_t)ret;
}

uint16_t us_to_tim2(uint16_t prog_time) {
  float t_us=0;
  float ret=0;
  t_us=prog_time*1000;
  t_us=t_us/16;
  //return progtime_to_tim1(t_us);
  ret = ((t_us-1.9052)/0.1248);
  return (uint16_t)ret;
  //y = 0,1248x + 1,9052
}

uint16_t tim1_to_us(uint16_t tim1_time) {
   float ret=0;
   ret = 0.1246*tim1_time + 2.0698;
   return (uint16_t)(ret*10);
}
