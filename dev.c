#include <stdint.h>
#include <string.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf_gpio.h"

#include "dev.h"

/*--------------------extern FUNCTIONS----------------*/
extern bool ble_connection_status;
extern uint32_t update_send_buffer(uint8_t * p_send_array, uint16_t len);
extern uint8_t  send_array[64];

//SPI
extern uint8_t spi_tx_buf[];
extern uint8_t spi_rx_buf[];
extern void spi_send();

//FLASH
extern uint32_t flash_buff[];
extern const uint16_t flash_buff_size;
extern uint8_t wait_flash_timer;

/*-------------------- FUNCTIONS----------------*/
uint8_t ble_command_parser(uint8_t *data,uint16_t len);
void ble_answer_ok(uint16_t package_id);
void ble_answer_fail(uint16_t package_id,uint16_t error );

//void add_history(uint8_t change_num,uint8_t change_type);

//void load_params(void);

//--------------- ПАРАМЕТРЫ РАБОТЫ УСТРОЙСТВА ---------------------
struct  struct_prog              programm;
struct  struct_impulse_programm  new_prog;


struct  struct_status_device_now  status_device_now;
struct  struct_application        app;

//--------------- ПАРАМЕТРЫ РАБОТЫ УСТРОЙСТВА ---------------------

bool subscription_prog_value=false;

struct history {
  uint32_t datetime_offset;
  uint8_t  level;
  uint8_t  level_change_type;
 } ;

#define RAM_HISTORY_SIZE 256
struct history ram_history[RAM_HISTORY_SIZE];
uint16_t ram_history_write=0;
uint16_t ram_history_send=0;

bool send_history = false;


//#define  BTN_DEACTIVE_TIME 20 //20//*0,1=2sec
//uint8_t  btn_deactive_timer = 0; //2sec
//uint8_t  btn_tim_update     = 0;

uint8_t  btn_left_click     = 0;
uint8_t  btn_left_timer     = 0;

uint8_t  btn_right_click    = 0;
uint8_t  btn_right_timer    = 0;

uint8_t  led_blink_timer    = 0;

uint8_t  ledb_timer    = 0;

#define LED_G_BLINK   LED_G_ON;ledb_timer=1;

uint8_t delay_100ms=0;

extern uint8_t advestering;
extern bool    device_work;

//last_ble_cmd
uint8_t     cmd     = 0;
uint8_t     cmd_len = 0;

bool stm32_ping   = false;
bool stm32_answer = false;
bool stm32_sleep_pin_status=0;


void dev_init() {
  //LED 
  nrf_gpio_cfg_output(PIN_LED); 
  //STM32_SLEEP
  nrf_gpio_cfg_output(PIN_STM_SLEEP); 

  //LED_Config_RGB
  nrf_gpio_cfg_output(PIN_R); //R
  nrf_gpio_cfg_output(PIN_G); //G
  nrf_gpio_cfg_output(PIN_B); //B

  LED_R_OFF;
  LED_G_OFF;
  LED_B_OFF;

  STM32_SLEEP_ON;

  send_history=false;
}//void dev_init()

void Load_Default_Programm(void) 
{
// STM8L params //312,5us*32=10ms \ ~3us - max_perion 60us
//LOAD_PROGRAMM ---------------------------------- //
  programm.prog_id          = 0x0001;  //номер шаблона
  programm.count_limit      = 5;       //ограничено по времени
  
  programm.imp_voltage      = 19;      //19V
  programm.imp_t_pause      = 5;       //312,5us*16 = 5ms время между импульсами одного канала 
  programm.imp_t_min        = 10;      //  /10=1us+-1
  programm.imp_t_max        = 600;     //  /10=60us+-1
  programm.prog_time        = 1200;    //sec
  programm.prog_num         = 20;      //число шагов
  programm.prog_time_num    = 1000;    //sec, время перехода к конечному уровню
  programm.prog_button_mode = 1; //1-смартфон+кнопки(начинает сразу+кнопки)
                                 //2-со смартфоном(ждет смартфона)
                                 //3-смартфон+кнопки(начинает сразу+кнопки)

  programm.count_timer      = programm.count_limit*programm.prog_time;

  app.quote = 100*20*60; //sec
}//void dev_default_prog(void)

//100ms

void m_timeout_100ms_handler(void * p_context)  {  
   if (led_blink_timer!=0) {led_blink_timer--; if (led_blink_timer==0) {LED_OFF;}}

   //if (ledr_timer!=0) {ledr_timer--;if (ledr_timer==0) {LED_R_OFF};}
   //if (ledg_timer!=0) {ledg_timer--;if (ledg_timer==0) {LED_G_OFF};}

   if (ledb_timer!=0) {ledb_timer--;if (ledb_timer==0) {LED_B_OFF};}

   if (btn_left_timer!=0)  {
        if (btn_left_timer==BTN_LONG_TIME) { //*100=3sec
           btn_left_click=2; //long
           btn_left_timer++;
        } else {
          if (btn_left_timer<BTN_LONG_TIME) {
            btn_left_timer++;
          }
        }
   }

   if (btn_right_timer!=0) {btn_right_timer++;}

   if ((ble_connection_status)&&(send_history)) {
   //if (ble_connection_status) {
     if (ram_history_send!=ram_history_write) {
        uint16_t ram_h=ram_history_send;
        //send_hystory
        memset(&send_array,0x00,64);

        send_array[0]=(uint8_t)(app.package_send_counter);    //package_id
        send_array[1]=(uint8_t)(app.package_send_counter>>8); //package_id
   
        send_array[2]=2;//cmd
        send_array[3]=13;//len

        if (++ram_h>=RAM_HISTORY_SIZE) {ram_h=0;}
        if (ram_h==ram_history_write) {
            send_array[4]=1; //Is Last Data Packet 1-last
         } else {
             send_array[4]=0;
         }

        send_array[5]=0x55;//Program Launch ID //Номер запуска программы (инкрементируется устройством)
        send_array[6]=1;//N //Количество блоков данных (в текущем пакете) 

        send_array[7]=(uint8_t)(ram_history[ram_history_send].datetime_offset);     //Low
        send_array[8]=(uint8_t)(ram_history[ram_history_send].datetime_offset>>8);  //m_l
        send_array[9]=(uint8_t)(ram_history[ram_history_send].datetime_offset>>16); //m_H
        send_array[10]=(uint8_t)(ram_history[ram_history_send].datetime_offset>>24); //High
        
        send_array[11]=ram_history[ram_history_send].level;//N-level //Номер уровня, на который произошел переход

        send_array[12]=ram_history[ram_history_send].level_change_type;//Тип изменения уровня //1 – автоматический (плавный) //2 – ручной (резкий)
        
        ram_history_send++;

        app.send_error=update_send_buffer(send_array,send_array[3]);
        app.package_send_counter++;

     }
   }

   if (wait_flash_timer!=0) {wait_flash_timer--;}
   if (delay_100ms!=0)  {delay_100ms--;}
 
}//static void m_timeout_100ms_handler(void * p_context)

void delay100(uint8_t delay) {
  delay_100ms=delay;
  while (delay_100ms!=0) {__WFE();}
}

void m_timeout_1sec_handler(void * p_context) {

  //NRF_WDT->RR[0] = WDT_RR_RR_Reload;  //Reload watchdog register 0

  //1sec ----- START ---------
  if (ble_connection_status) 
  {
     nrf_gpio_pin_toggle(PIN_B); //
  } else 
  {   
    if (advestering) {
       nrf_gpio_pin_write(PIN_B,0); //LED_B ON
       ledb_timer=1;
     }
  }

  if (device_work)
  {
      if (batt_lvl_in_milli_volts>BATTERY_NORMAL) {
          //nrf_gpio_pin_toggle(PIN_G); //
          if (nrf_gpio_pin_out_read(PIN_B)) {
            nrf_gpio_pin_clear(PIN_G);
          } else {
            nrf_gpio_pin_set(PIN_G);
          }
          LED_R_OFF;
      } else {
          if (batt_lvl_in_milli_volts>BATTERY_LOW) {
            LED_G_OFF;
            //nrf_gpio_pin_toggle(PIN_R); //
            if (nrf_gpio_pin_out_read(PIN_B)) {
              nrf_gpio_pin_clear(PIN_R);
            } else {
              nrf_gpio_pin_set(PIN_R);
            }
          } else {
             //LOW_BATTERY POWER_OFF
            LED_R_ON;  
          }
      }//if (status_device_now.battery_voltage>3800) {

      if (status_device_now.device_status==1) {
        if (app.quote!=0) {
          app.quote_used++;
          app.quote--;
          if (app.quote==0) {
              Stm32_Stop_Devive();
              status_device_now.device_status=0;
          } //if (app.quote==0) {
        }

        if (programm.count_timer!=0) {
          programm.count_timer--;
          if (programm.count_timer==0) {
             Stm32_Stop_Devive();
             status_device_now.device_status=0;
          }//if (programm.count_timer==0) {
        }
      }//if (status_device_now.device_status==1) {

  if (status_device_now.hv_voltage!=0) {
     LED_ON;
   } else {
     LED_OFF;
   }
   
  }//if (device_work) {


   //if (subscription_prog_value) {
   if (ble_connection_status) {
    
     PROGRAM_VALUES(0x1234,5);
     app.send_error=update_send_buffer(send_array,send_array[3]);
     app.package_send_counter++;
   }

//   if (send_error!=0) 
//   {
//     send_error=update_send_buffer(send_array,send_array[3]);
//     package_send_counter++;
//   }
  app.datetime_offset++;

}//void m_timeout_1sec_handler(void)


void PROGRAM_VALUES(uint16_t pckg, uint8_t cmd) {
  if (pckg==0) {
    pckg=0x1234;
  }
  send_array[0]=(uint8_t)((pckg));    //package_id
  send_array[1]=(uint8_t)((pckg)<<8); //package_id
  send_array[2]=cmd;//cmd
  send_array[3]=20;//len

  send_array[4]=(uint8_t)(programm.prog_id);                       //Идентификатор  программы 
  send_array[5]=(uint8_t)(programm.prog_id>>8);                    //Идентификатор  программы
  
  send_array[6]=(uint8_t)(status_device_now.prog_impulse_us);      //Текущее T импульса 
  send_array[7]=(uint8_t)(status_device_now.prog_impulse_us>>8);   //Текущее T импульса 

  send_array[8]=(uint8_t)(status_device_now.prog_time);            //Текущее время выполнения программы, с
  send_array[9]=(uint8_t)(status_device_now.prog_time>>8);         //Текущее время выполнения программы, с

  send_array[10]=status_device_now.prog_num_now;                    //Текущий уровень

  send_array[11]=(uint8_t)(status_device_now.prog_time_num);        //Время  ДО перехода к конечному уровню работы, с
  send_array[12]=(uint8_t)(status_device_now.prog_time_num>>8);     //Время  ДО перехода к конечному уровню работы, с

  send_array[13]=(uint8_t)(status_device_now.battery_percent);      // 0-100% = Заряд аккумулятора,В
  send_array[14]=(uint8_t)(status_device_now.hv_voltage*10);        // 190/10 = Высокое напряжение,В

  //  Состояния выполнения текущей программы
  send_array[15]=(uint8_t)(status_device_now.device_status);        //STOP=0//RUN=1//PAUSE=2///SLEEP=3


  send_array[16]=(uint8_t)(status_device_now.hv_current);           //0-255 насколько сильное прижатие
  send_array[17]=(uint8_t)(status_device_now.hv_current_status);    // 0-100 прижатие прибора к языку

  send_array[18]=(uint8_t)(status_device_now.prog_time);          //programm.prog_time     = 1200; //sec
  send_array[19]=(uint8_t)(status_device_now.prog_time>>8);       //programm.prog_time     = 1200; //sec
  
}

uint8_t btn_counter=0;
void dev_main_loop() {
 
}//void main_loop()

uint8_t  ble_command_parser(uint8_t *data,uint16_t len) {
   ret_code_t  err_code;
   uint8_t     i       = 0;
   bool        answer  = false;


   if (len<4) {
     ble_answer_fail(app.package_id,0x0002); //неизвестная команда 2
     return 0;
   }
   app.package_id   = (data[1]<<8)+data[0];
   cmd              =  data[2];
   cmd_len          =  data[3];

   //CONNECT REQUEST
   //0A-00-01-18-BB-04-1F-02-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-01
   if (cmd==1) //&&(cmd_len==24)) 
   {
     //read
     app.datetime_offset = (data[7]<<24)+(data[6]<<16)+(data[5]<<8)+data[4];
     memcpy(&app.app_id_last,&app.app_id,APP_ID_SIZE);
     for (i=16;i<24;i++) {
        app.app_id[i-16]=data[i];
     }

     //answer
     answer=true;
     memset(&send_array,0x00,64);
     send_array[0]=(uint8_t)((app.package_id));    //package_id
     send_array[1]=(uint8_t)((app.package_id)>>8); //package_id
     send_array[2]=cmd;//cmd
     send_array[3]=0x27;//len

     send_array[4]=0;//4-7 Initial Quote
     
     for (i=0;i<16;i++) {
        send_array[18+i]=app.app_id[i];
     }//Идентификатор последнего управляющего приложения (0 – нет).

     send_array[33]=(uint8_t)((programm.prog_id));    //Program ID
     send_array[34]=(uint8_t)((programm.prog_id>>8)); //Program ID
      
     send_array[36]= 0x00; //Оставшаяся квота повторений программы ???

     //Состояния выполнения текущей программы
     //STOP=0//RUN=1//PAUSE=2///SLEEP=3
     send_array[37]=status_device_now.device_status;
     

     if (ram_history_send!=ram_history_write) {
       send_array[38]=0x01; //Имеется ли накопленная история
     } else {
       send_array[38]=0x00; //Имеется ли накопленная история
     }
     
     app.send_error=update_send_buffer(send_array,send_array[3]);
     app.package_send_counter++;

     //ble_answer_ok(app.package_id);

   } //cmd==1  //CONNECT REQUEST

   //ADD QUOTE REQUEST
   //04-00- 20- 0A- 01-00- 1E-00-00-00
   if ((cmd==0x20)&&(cmd_len==10)) {
     app.quote_id  = (data[5]<<8) + data[4];
     app.quote    += (data[9]<<24)+(data[8]<<16)+(data[7]<<8)+data[6];
     
     //answer
     answer=true;
     memset(&send_array,0x00,64);
     send_array[0]=(uint8_t)(app.package_id);    //package_id
     send_array[1]=(uint8_t)(app.package_id>>8); //package_id
     
     send_array[2]=cmd;//cmd
     send_array[3]=10;//len

     send_array[4]=(uint8_t)(app.quote_id);    //low
     send_array[5]=(uint8_t)(app.quote_id>>8); //high

     send_array[6]=(uint8_t)(app.quote);      //Low
     send_array[7]=(uint8_t)(app.quote>>8);   //m_l
     send_array[8]=(uint8_t)(app.quote>>16);  //m_H
     send_array[9]=(uint8_t)(app.quote>>24);  //High

     app.send_error=update_send_buffer(send_array,send_array[3]);
     app.package_send_counter++;
   }//cmd==32 //ADD QUOTE REQUEST

   //SYNC COMPLETE REQUEST
   //08-00-03-05-00
   if ((cmd==3)&&(cmd_len==5)) {
      answer=true;
      //Уведомление устройства об окончании синхронизации:
      if (data[4]==1) {
         //Oчистить всю накопленную историю
         ram_history_send=ram_history_write=0;
      }
      ble_answer_ok(app.package_id);
   } //cmd==3 //SYNC COMPLETE REQUEST

   //HISTORY DATA REQUEST
   if ((cmd==2)&&(cmd_len==4)) {
     answer=true;
     send_history=true;

   }//cmd==2 //HISTORY DATA REQUEST

  
   //GET PROGRAM VALUES REQUEST
   //5C-00-05-04
   if ((cmd==5)&&(cmd_len==4)) {
    uint16_t tim1_us;
    answer=true;
    memset(&send_array,0x00,64);

    PROGRAM_VALUES(app.package_id, cmd);

    app.send_error=update_send_buffer(send_array,send_array[3]);
    app.package_send_counter++;
   }//cmd==5 //HISTORY DATA REQUEST

   //GET PROGRAM VALUES subscription
   // 1A-00-65-05-01
   if ((cmd==0x65)&&(cmd_len==6)) {
     answer=true;
     memset(&send_array,0x00,64);
     if (send_array[4]==1) {
       subscription_prog_value=true;
     } else {
       subscription_prog_value=false;
     }
     answer=true;
     ble_answer_ok(app.package_id);
   }//cmd==6

    //PING REQUEST
    //22-00-0F-08-79-91-2E-02
   if ((cmd==15)&&(cmd_len==8)) {
    answer=true;
    memset(&send_array,0x00,64);
    send_array[0]=(uint8_t)(app.package_id);    //package_id
    send_array[1]=(uint8_t)(app.package_id>>8); //package_id

    send_array[2]=cmd;//cmd
    send_array[3]=8;//len

    send_array[4]=(uint8_t)(app.datetime_offset);     //Low
    send_array[5]=(uint8_t)(app.datetime_offset>>8);  //m_l
    send_array[6]=(uint8_t)(app.datetime_offset>>16); //m_H
    send_array[7]=(uint8_t)(app.datetime_offset>>24); //High

    app.send_error=update_send_buffer(send_array,send_array[3]);
    app.package_send_counter++;
   }//cmd==15 //PING REQUEST

    //SET PROGRAM REQUEST
   if ((cmd==8)&&(cmd_len==20)) {
     
     uint16_t error=0;
     answer=true;

     //stop_programm
     new_prog.prog_id          = (data[5]<<8) + data[4];
     new_prog.count_limit      = data[6];                  //????
     new_prog.imp_voltage      = data[7];                  //16\19v
     new_prog.imp_t_pause      = (data[9]<<8) + data[8];   //T паузы, (1 — 120) *10 млс +-0,1 
     new_prog.imp_t_min        = (data[11]<<8) + data[10]; //T импульса минимальное (1 уровень),  мкс 2.5*10
     new_prog.imp_t_max        = (data[13]<<8) + data[12]; //T импульса максимальное (X уровень), мкс 120*10
     new_prog.prog_time        = (data[15]<<8) + data[14]; //Общее время выполнения программы, с
     new_prog.prog_num         = data[16];                 //Конечный уровень
     new_prog.prog_time_num    = (data[18]<<8) + data[17]; //Время перехода к конечному уровню работы
     new_prog.prog_button_mode = data[19];                 //Режим работы программы:

     new_prog.count_timer      = new_prog.count_limit*new_prog.prog_time; //значение количества разрешенных использований, но не более 90 

     //CONTROL_PARAMS   !!!!!!!!!!!!!!!!!
     if ((new_prog.imp_voltage!=16)     || (new_prog.imp_voltage!=19))         {error=0x0801;} //2049
     if ((new_prog.imp_t_pause>1200)    || (new_prog.imp_t_pause<1))           {error=0x0802;} //2050
     if ((new_prog.imp_t_min>1200)      || (new_prog.imp_t_min<10))            {error=0x0803;} //2051
     if ((new_prog.imp_t_max>1200)      || (new_prog.imp_t_max<10))            {error=0x0804;} //2052
     if (new_prog.imp_t_min>new_prog.imp_t_max)                                {error=0x0805;} //2053
     if ((new_prog.prog_time>41*60)     || (new_prog.prog_time<120))           {error=0x0806;} //2054 //sec
     if ((new_prog.prog_num>40)         || (new_prog.prog_num==0))             {error=0x0807;} //2055 //число шагов
     if ((new_prog.prog_time_num>41*60) && (new_prog.prog_time_num>120))       {error=0x0808;} //2056 //sec
     if (new_prog.prog_time_num>new_prog.prog_time)                            {error=0x0809;} //2057
     if ((new_prog.prog_button_mode>4)  || (new_prog.prog_button_mode==0))     {error=0x080a;} //2058
     if ((new_prog.imp_t_pause*1000/16)>=(new_prog.imp_t_max))                 {error=0x080b;} //2059 //16*ICP<IBP
     if (new_prog.count_timer>90*1200)                                         {error=0x080c;} //2060
     
     error=0;
     if (error==0) {
       memcpy(&programm,&new_prog,sizeof(new_prog));
       Stm32_Send_Params();
       //Nrf52_Update_Params();
       //Save_To_Flash

       ////wait_answer
       //uint8_t count=10;
       //while ((stm32_answer==false) && (count>0)) {
       //   delay100(1);
       //   count--;
       //}
       //answer=true;

       //if (stm32_answer==true) {
       //   ble_answer_ok(app.package_id);
       //} else {
       //   error=0x0010;//dec=16 
       //   ble_answer_fail(app.package_id,error);
       //}

       answer=true;
       ble_answer_ok(app.package_id);
     } else {
       ble_answer_fail(app.package_id,error);
      }
   }//cmd==8


   //GET PROGRAM DEVICE REQUEST
   if ((cmd==0x09)&&(cmd_len==4)) {
    answer=true;
    memset(&send_array,0x00,64);
    send_array[0]=(uint8_t)(app.package_id);    //package_id
    send_array[1]=(uint8_t)(app.package_id>>8); //package_id

    send_array[2]=cmd;//cmd
    send_array[3]=20;//len

    //PROGRAMM
    send_array[4]=(uint8_t)(programm.prog_id);            //номер шаблона
    send_array[5]=(uint8_t)(programm.prog_id>>8);         //номер шаблона
    
    //send_array[6]=programm.count_limit;                             //Program Launch Limit
    if (programm.prog_time!=0) {
       send_array[6]=(uint8_t)(programm.count_timer/programm.prog_time); //Program Launch Limit
    } else {
       send_array[6]=0;
    }
    
    send_array[7]=(uint8_t)programm.imp_voltage;          // 16\19V
    
    send_array[8]=(uint8_t)(programm.imp_t_pause*10);     //programm.imp_t_pause   = 2;    //ms время между импульсами одного канала
    send_array[9]=(uint8_t)(programm.imp_t_pause*10>>8);  //programm.imp_t_pause   = 2;    //ms время между импульсами одного канала

    send_array[10]=(uint8_t)((uint16_t)(programm.imp_t_min*10));     //programm.imp_t_min     = 2.5;    //us
    send_array[11]=(uint8_t)((uint16_t)(programm.imp_t_min*10)>>8);  //programm.imp_t_min     = 2.5;    //us

    send_array[12]=(uint8_t)(programm.imp_t_max*10);       //programm.imp_t_max     = 20;   //us
    send_array[13]=(uint8_t)(programm.imp_t_max*10>>8);    //programm.imp_t_max     = 20;   //us
 
    send_array[14]=(uint8_t)(programm.prog_time);          //programm.prog_time     = 1200; //sec
    send_array[15]=(uint8_t)(programm.prog_time>>8);       //programm.prog_time     = 1200; //sec

    send_array[16]=(uint8_t)programm.prog_num;             //programm.prog_num      = 10;   //число шагов
    
    send_array[17]=(uint8_t)(programm.prog_time_num);      //programm.prog_time_num = 1000; //sec, време перехода к конечному уровню
    send_array[18]=(uint8_t)(programm.prog_time_num>>8);   //programm.prog_time_num = 1000; //sec, време перехода к конечному уровню
 
    send_array[19]=(uint8_t)programm.prog_button_mode;     //programm.prog_button_mode = 1; //по кнопкам
    app.send_error=update_send_buffer(send_array,send_array[3]);
    app.package_send_counter++;
    
   }//cmd==2 //HISTORY DATA REQUEST}


   //CHANGE PROGRAM LEVEL REQUEST
   //0A-00-13-05-FF
   if ((cmd==19)&&(cmd_len==5)) {
     
     if (data[4]==0x01) {
       //+1 level
       btn_right_click=1;
     } else {
       //-1 level
       btn_left_click=1;
     }
     //answer
     answer=true;
     memset(&send_array,0x00,64);
     send_array[0]=(uint8_t)(app.package_id);    //package_id
     send_array[1]=(uint8_t)(app.package_id>>8); //package_id
     
     send_array[2]=cmd;//cmd
     send_array[3]=cmd_len;//len
     
     if (status_device_now.device_status!=0) {
        send_array[4]=status_device_now.prog_num_now;
     } else {
        send_array[4]=0; //устройство неактивно
     }

     app.send_error=update_send_buffer(send_array,send_array[3]);
     app.package_send_counter++;
   } //cmd==19 //CHANGE PROGRAM LEVEL REQUEST

   //START PROGRAM REQUEST
   //16 (0x10)
   if ((cmd==0x10)&&(cmd_len==4)) {
     answer=true;

     Stm32_Start_Devive();
     status_device_now.device_status=1;

     ble_answer_ok(app.package_id);
   }

    //STOP PROGRAM REQUEST
   //17 (0x11)
   if ((cmd==0x11)&&(cmd_len==4)) {
     answer=true;

     Stm32_Stop_Devive();
     status_device_now.device_status=0;

     ble_answer_ok(app.package_id);
   }

  
   if (answer==false) {
     ble_answer_fail(app.package_id,0x0001); //неизвестная команда
   }
}//void ble_command_parser(uint8_t *data,uint16_t len) {



void ble_answer_ok(uint16_t package_id) {
  //answer OK
    memset(&send_array,0x00,64);
    send_array[0]=(uint8_t)(app.package_id);    //package_id
    send_array[1]=(uint8_t)(app.package_id>>8); //package_id
   
    send_array[2]=255;//cmd
    send_array[3]=4;//len

    app.send_error=update_send_buffer(send_array,send_array[3]);
    app.package_send_counter++;
}//void ble_answer_ok(uint16_t package_id) {

void ble_answer_fail(uint16_t package_id,uint16_t error ) {
  //answer OK
    app.last_error = error;
    memset(&send_array,0x00,64);
    send_array[0]=(uint8_t)(app.package_id);    //package_id
    send_array[1]=(uint8_t)(app.package_id>>8); //package_id
   
    send_array[2]=254;//cmd
    send_array[3]=6;//len
    
    send_array[4]=(uint8_t)(error);//err
    send_array[5]=(uint8_t)(error>>8);//err


    app.send_error=update_send_buffer(send_array,send_array[3]);
    app.package_send_counter++;
}//void ble_answer_ok(uint16_t package_id) {

void Stm32_Send_BtnClick(uint8_t btn,uint8_t btn_status) {
  spi_tx_buf[0]=0x04; //len
  spi_tx_buf[1]=0x11; //cmd
  spi_tx_buf[2]=btn; //btn_num
  spi_tx_buf[3]=btn_status; //btn_status 0,1,2
  spi_send();
}

void Stm32_Send_Params(void) {
    spi_tx_buf[0]=17;// len
    spi_tx_buf[1]=0x31;// cmd

    spi_tx_buf[2]=(uint8_t)(programm.prog_id);          //prog_id
    spi_tx_buf[3]=(uint8_t)(programm.prog_id>>8);       //prog_id
    
    spi_tx_buf[4]=(uint8_t)(programm.imp_voltage);       //HV_value
    
    spi_tx_buf[5]=(uint8_t)(programm.imp_t_pause);       //T паузы, 1 — 120 млс +-0,1
    spi_tx_buf[6]=(uint8_t)(programm.imp_t_pause>>8);    //T паузы, 1 — 120 млс +-0,1
    
    spi_tx_buf[7]=(uint8_t)(programm.imp_t_min);         //T импульса минимальное (1 уровень), мкс 2.5
    spi_tx_buf[8]=(uint8_t)(programm.imp_t_min>>8);      //T импульса минимальное (1 уровень), мкс 2.5
    
    spi_tx_buf[9]= (uint8_t)(programm.imp_t_max);        //T импульса максимальное (X уровень), мкс 120
    spi_tx_buf[10]=(uint8_t)(programm.imp_t_max>>8);     //T импульса максимальное (X уровень), мкс 120
    
    spi_tx_buf[11]=(uint8_t)(programm.prog_time);        //Общее время выполнения программы, с
    spi_tx_buf[12]=(uint8_t)(programm.prog_time>>8);     //Общее время выполнения программы, с
    
    spi_tx_buf[13]=(uint8_t)(programm.prog_num);         //Число уровней
    
    spi_tx_buf[14]=(uint8_t)(programm.prog_time_num);    //Время перехода к конечному уровню работы
    spi_tx_buf[15]=(uint8_t)(programm.prog_time_num>>8); //Время перехода к конечному уровню работы
    
    spi_tx_buf[16]=(uint8_t)(programm.prog_button_mode);  //Режим работы программы:

    stm32_answer=false;

    spi_send();
}//void Send_Params_to_STM32(void) {

void Stm32_Start_Devive() {
  spi_tx_buf[0]=0x02; //len
  spi_tx_buf[1]=0x33; //cmd
  spi_tx_buf[2]=0x01; //

  status_device_now.device_status=1;
  spi_send();
}

void Stm32_Stop_Devive() {
  spi_tx_buf[0]=0x02; //len
  spi_tx_buf[1]=0x34; //cmd
  spi_tx_buf[2]=0x01; //

  status_device_now.device_status=0;
  spi_send();

}

void Stm32_Pause_Devive() {
  spi_tx_buf[0]=0x02; //len
  spi_tx_buf[1]=0x35; //cmd
  spi_tx_buf[2]=0x01; //

  status_device_now.device_status=3;
  spi_send();
}

void Stm32_Sleep_On(void) {
  spi_tx_buf[0]=0x02; //len
  spi_tx_buf[1]=0x02; //cmd
  spi_tx_buf[2]=0x01; //
  spi_send();
}

void Stm32_Sleep_Off(void) {
  spi_tx_buf[0]=0x02; //len
  spi_tx_buf[1]=0x01; //cmd
  spi_tx_buf[2]=0x01; //START/
  spi_send();
}

void Stm32_Ping(void) {
  spi_tx_buf[0]=0x02; //len
  spi_tx_buf[1]=0x03; //cmd
  spi_tx_buf[2]=0x55; //
  stm32_ping=false;
  spi_send();
}



//#define RAM_HISTORY_SIZE 256
//struct history ram_history[RAM_HISTORY_SIZE];
//uint16_t ram_history_write=0;
//uint16_t ram_history_send=0;

void spi_parce(void) {
  uint8_t len,cmd;
  uint32_t eeprom_read;
  
  //spi_rx_buf[0]=0xff;
  len=spi_rx_buf[1];
  cmd=spi_rx_buf[2];

  switch (cmd) {
     case 0x01:
        //Время вышло, импульсы кончились
        status_device_now.device_status=0;
        status_device_now.prog_time    =0;
        break;
     case 0x02: 
       //prog_num_now - номер уровня, на который произошел переход
       //change_type  - 1 - auto; 2 - btn_click 
       ram_history[ram_history_write].datetime_offset=app.datetime_offset;
       ram_history[ram_history_write].level=spi_rx_buf[3];
       ram_history[ram_history_write].level_change_type=spi_rx_buf[4];
       if (++ram_history_write>=RAM_HISTORY_SIZE) {ram_history_write=0;}
       break;
     case 0x03:
       stm32_ping=true;
       break;
//  case 0x05: //EEPROM READ
//     eeprom_read=(spi_rx_buf[6]<<24)+(spi_rx_buf[5]<<16)+(spi_rx_buf[4]<<8)+spi_rx_buf[3];
//     break;
  case 0x0a:
     status_device_now.device_status    =  spi_rx_buf[3]; 
     status_device_now.prog_impulse_us  = (spi_rx_buf[5]<<8) + spi_rx_buf[4];  //Текущее T импульса 
     status_device_now.prog_time        = (spi_rx_buf[7]<<8) + spi_rx_buf[6];  //Текущее время выполнения программы, с     
     status_device_now.prog_num_now     =  spi_rx_buf[8];                      //Текущий уровень
     status_device_now.prog_time_num    = (spi_rx_buf[10]<<8) + spi_rx_buf[9]; //Время  ДО перехода к конечному уровню
     status_device_now.hv_voltage       =  spi_rx_buf[11];                     // 0\16\19 vdc
     status_device_now.hv_current       =  spi_rx_buf[12];                     // hv_current 0-255
     status_device_now.hv_current_status = spi_rx_buf[13];                     // hv_current 0-1
     break;
   case 0x31:
     stm32_answer=true;
     break;
  }//switch (cmd) {


}//void spi_parce(void) {


void Save_Paramms(void) {
  //uint16_t i=0;
  
  flash_buff[0]= 0; //KEY_WRITE\ERASE
  flash_buff[1]= flash_buff[1]++;
  flash_buff[2]= 0x55;

  flash_buff[3]= app.datetime_offset;//datetime_offset //время в секундах

  flash_buff[4]= (app.app_id[0]<<24) +(app.app_id[1]<<16) +(app.app_id[2]<<8) +app.app_id[3];;//app_id[16] 
  flash_buff[5]= (app.app_id[4]<<24) +(app.app_id[5]<<16) +(app.app_id[6]<<8) +app.app_id[7];;//app_id[16] 
  flash_buff[6]= (app.app_id[8]<<24) +(app.app_id[9]<<16) +(app.app_id[10]<<8)+app.app_id[11];;//app_id[16] 
  flash_buff[7]= (app.app_id[12]<<24)+(app.app_id[13]<<16)+(app.app_id[14]<<8)+app.app_id[15];;//app_id[16] 

  flash_buff[8]= (app.app_id_last[0]<<24)  +(app.app_id_last[1]<<16) +(app.app_id_last[2]<<8) +app.app_id_last[3];;//app_id[16] 
  flash_buff[9]= (app.app_id_last[4]<<24)  +(app.app_id_last[5]<<16) +(app.app_id_last[6]<<8) +app.app_id_last[7];;//app_id[16] 
  flash_buff[10]= (app.app_id_last[8]<<24) +(app.app_id_last[9]<<16) +(app.app_id_last[10]<<8)+app.app_id_last[11];;//app_id[16] 
  flash_buff[11]= (app.app_id_last[12]<<24)+(app.app_id_last[13]<<16)+(app.app_id_last[14]<<8)+app.app_id_last[15];;//app_id[16] 

  flash_buff[20] = app.quote;          //размер квоты, сек
  flash_buff[21] = app.quote_id;       //Идентификатор(инкрементируемый на сервере) добавляемой квоты
  flash_buff[22] = app.quote_last_id;  //идентификатор последней добавленной квоты
  flash_buff[23] = app.quote_used;     //размер за все время использованной квоты, сек
  flash_buff[24] = app.quote_add;      //размер за все время добавленной квоты, сек

  flash_buff[25] = app.prog_launch_id; //Номер запуска программы (инкрементируется устройством)

  flash_buff[30] = programm.prog_id;          // номер шаблона
  flash_buff[31] = programm.count_limit;      // ограничено по времени
  flash_buff[32] = programm.imp_voltage;      // 16\19V
  flash_buff[33] = programm.imp_t_pause;      //ms время между импульсами одного канала
  flash_buff[34] = programm.imp_t_min;        //  /10=us
  flash_buff[35] = programm.imp_t_max;        //  /10=us
  flash_buff[36] = programm.prog_time ;       //sec
  flash_buff[37] = programm.prog_num;         //число шагов
  flash_buff[38] = programm.prog_time_num;    //sec, время перехода к конечному уровню
  flash_buff[39] = programm.prog_button_mode; //1 - по кнопкам (начинает работать сразу)
//                                            //2 - со смартфоном(ждет смартфона)
//                                            //3 - смартфон+кнопки(начинает сразу+кнопки)
  flash_buff[40] = programm.count_timer;       //count_limit по времени


//-------------------TEST---------------------
//  for (i=0;i<flash_buff_size;i++) {
//    flash_buff[i]=datetime_offset+i;
//  }
}


bool Load_Paramms(void) { 
  uint32_t flash_key=0;

  flash_key= flash_buff[0]; //KEY_WRITE\ERASE
  app.flash_write_counter= flash_buff[1];
  //flash_buff[2]= 0x55;

  if (flash_key==0xffffffff) {return false;} //flash_clear

  app.datetime_offset = flash_buff[3];//datetime_offset //время в секундах


//app_id
  //flash_buff[4]= (app_id[0]<<24) +(app_id[1]<<16) +(app_id[2]<<8) +app_id[3];;//app_id[16] 
  app.app_id[0] = (uint8_t)(flash_buff[4]<<24);
  app.app_id[1] = (uint8_t)(flash_buff[4]<<16);
  app.app_id[2] = (uint8_t)(flash_buff[4]<<8);
  app.app_id[3] = (uint8_t)(flash_buff[4]);


  //flash_buff[5]= (app_id[4]<<24) +(app_id[5]<<16) +(app_id[6]<<8) +app_id[7];;//app_id[16] 
  app.app_id[4] = (uint8_t)(flash_buff[5]<<24);
  app.app_id[5] = (uint8_t)(flash_buff[5]<<16);
  app.app_id[6] = (uint8_t)(flash_buff[5]<<8);
  app.app_id[7] = (uint8_t)(flash_buff[5]);

//  flash_buff[6]= (app_id[8]<<24) +(app_id[9]<<16) +(app_id[10]<<8)+app_id[11];;//app_id[16] 
  app.app_id[8]  = (uint8_t)(flash_buff[6]<<24);
  app.app_id[9]  = (uint8_t)(flash_buff[6]<<16);
  app.app_id[10] = (uint8_t)(flash_buff[6]<<8);
  app.app_id[11] = (uint8_t)(flash_buff[6]);
//  flash_buff[7]= (app_id[12]<<24)+(app_id[13]<<16)+(app_id[14]<<8)+app_id[15];;//app_id[16] 
  app.app_id[12] = (uint8_t)(flash_buff[7]<<24);
  app.app_id[13] = (uint8_t)(flash_buff[7]<<16);
  app.app_id[14] = (uint8_t)(flash_buff[7]<<8);
  app.app_id[15] = (uint8_t)(flash_buff[7]);
//

//app_id_last
//  flash_buff[8]= (app.app_id_last[0]<<24) +(app.app_id_last[1]<<16) +(app.app_id_last[2]<<8) +app.app_id_last[3];;//app_id[16] 
  app.app_id_last[0] = (uint8_t)(flash_buff[8]<<24);
  app.app_id_last[1] = (uint8_t)(flash_buff[8]<<16);
  app.app_id_last[2] = (uint8_t)(flash_buff[8]<<8);
  app.app_id_last[3] = (uint8_t)(flash_buff[8]);
//  flash_buff[9]= (app.app_id_last[4]<<24) +(app.app_id_last[5]<<16) +(app.app_id_last[6]<<8) +app.app_id_last[7];;//app_id[16] 
  app.app_id_last[4] = (uint8_t)(flash_buff[9]<<24);
  app.app_id_last[5] = (uint8_t)(flash_buff[9]<<16);
  app.app_id_last[6] = (uint8_t)(flash_buff[9]<<8);
  app.app_id_last[7] = (uint8_t)(flash_buff[9]);
//  flash_buff[10]= (app.app_id_last[8]<<24) +(app.app_id_last[9]<<16) +(app.app_id_last[10]<<8)+app.app_id_last[11];;//app_id[16] 
  app.app_id_last[8] = (uint8_t)(flash_buff[10]<<24);
  app.app_id_last[9] = (uint8_t)(flash_buff[10]<<16);
  app.app_id_last[10] = (uint8_t)(flash_buff[10]<<8);
  app.app_id_last[11] = (uint8_t)(flash_buff[10]);
//  flash_buff[11]= (app.app_id_last[12]<<24)+(app.app_id_last[13]<<16)+(app.app_id_last[14]<<8)+app.app_id_last[15];;//app_id[16] 
  app.app_id_last[12] = (uint8_t)(flash_buff[11]<<24);
  app.app_id_last[13] = (uint8_t)(flash_buff[11]<<16);
  app.app_id_last[14] = (uint8_t)(flash_buff[11]<<8);
  app.app_id_last[15] = (uint8_t)(flash_buff[11]);

  app.quote         = flash_buff[20];  //размер квоты, сек
  app.quote_id      = flash_buff[21];  //Идентификатор(инкрементируемый на сервере) добавляемой квоты
  app.quote_last_id = flash_buff[22];  //идентификатор последней добавленной квоты
  app.quote_used    = flash_buff[23];  //размер за все время использованной квоты, сек
  app.quote_add     = flash_buff[24];  //размер за все время добавленной квоты, сек

  app.prog_launch_id = flash_buff[25]; //Номер запуска программы (инкрементируется устройством)

  programm.prog_id       = (uint16_t)flash_buff[30];    //номер шаблона
  programm.count_limit   = (uint16_t)flash_buff[31];    //ограничено по времени
  programm.imp_voltage   = (uint16_t)flash_buff[32];    //19V
  programm.imp_t_pause   = (uint16_t)flash_buff[33];    //ms время между импульсами одного канала
  programm.imp_t_min     = (uint16_t)flash_buff[34];    //  /10=us
  programm.imp_t_max     = (uint16_t)flash_buff[35];    // /10=us
  programm.prog_time     = (uint16_t)flash_buff[36];    //sec
  programm.prog_num      = (uint8_t)flash_buff[37];     //число шагов
  programm.prog_time_num = (uint16_t)flash_buff[38];    //sec, время перехода к конечному уровню
  programm.prog_button_mode = (uint8_t)flash_buff[39];  //1 - по кнопкам (начинает работать сразу)
                                                        //2 - со смартфоном(ждет смартфона)
                                                        //3 - смартфон+кнопки(начинает сразу+кнопки)

  programm.count_timer   = (uint32_t)flash_buff[40];     //count_limit по времени
 
  return true;
}//void Load_Paramms(void) {