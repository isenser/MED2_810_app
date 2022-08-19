
#define PIN_LED 12        //
#define PIN_R 30          //R
#define PIN_B 28          //G
#define PIN_G 25          //B
#define PIN_STM_SLEEP 15  // 

#define LED_R_OFF   nrf_gpio_pin_write(PIN_R,1); //R_OFF
#define LED_R_ON   nrf_gpio_pin_write(PIN_R,0);  //LED_R_ON

#define LED_B_OFF   nrf_gpio_pin_write(PIN_B,1); //G_OFF
#define LED_B_ON   nrf_gpio_pin_write(PIN_B,0);  //LED_G_ON

#define LED_G_OFF   nrf_gpio_pin_write(PIN_G,1); //B_OFF
#define LED_G_ON   nrf_gpio_pin_write(PIN_G,0);  //LED_B_ON

#define LEDs_ON LED_R_ON;LED_G_ON;LED_B_ON;

#define LED_ON nrf_gpio_pin_write(PIN_LED,1);
#define  LED_OFF nrf_gpio_pin_write(PIN_LED,0);

extern bool stm32_sleep_pin_status;
#define STM32_SLEEP_ON  nrf_gpio_pin_write(PIN_STM_SLEEP,1); stm32_sleep_pin_status=1;
#define STM32_SLEEP_OFF nrf_gpio_pin_write(PIN_STM_SLEEP,0); stm32_sleep_pin_status=0;

//BTN1 
#define BTN_LEFT_PIN  0
#define BTN_RIGHT_PIN 1

//BATTERY
extern uint16_t batt_lvl_in_milli_volts;
#define BATTERY_FULL   4200
#define BATTERY_CHARGE 4000
#define BATTERY_NORMAL 3600
#define BATTERY_LOW    3400
#define BATTERY_CUT    3200 //DEVICE_OFF SLEEP_MODE
//#define BATTERY_MIN    3000

//MIN_VOLTAGE 2.75В //https://www.robiton.ru/article/1782

#define BTN_LONG_TIME 30 //*100ms


//SPI
#define SPI_PIN_SLAVE 16

struct struct_impulse_programm {
  uint16_t prog_id;          //номер шаблона
  uint16_t count_limit;      //
  uint32_t count_timer;      //=count_limit*prog_time
  uint16_t imp_voltage;      //19V
  uint16_t imp_t_pause;      //ms время между импульсами одного канала
  uint16_t imp_t_min;        //*10=us
  uint16_t imp_t_max;        //*10=us
  uint16_t prog_time;        //sec
  uint8_t  prog_num;         //число шагов
  uint16_t prog_time_num;    //sec, време перехода к конечному уровню
  uint8_t  prog_button_mode; //по кнопкам
  };

struct struct_prog {
  uint16_t prog_id;          //номер шаблона
  uint16_t count_limit;      //
  uint32_t count_timer;      //=count_limit*prog_time
  uint16_t imp_voltage;      //19V
  uint16_t imp_t_pause;      //ms время между импульсами одного канала
  uint16_t imp_t_min;        //*10=us
  uint16_t imp_t_max;        //*10=us
  uint16_t prog_time;        //сек, время сеанса
  uint8_t  prog_num;         //число шагов
  uint16_t prog_time_num;    //sec, време перехода к конечному уровню
  uint8_t  prog_button_mode; //по кнопкам
 };

struct struct_status_device_now {   
 uint16_t  count_limit_now;
 uint16_t  count_timer;       //=count_limit*prog_time
 uint16_t  prog_impulse_us;
 uint8_t   prog_num_now;
 uint16_t  prog_time;         //сек, время сеанса
 uint16_t  prog_time_num;     //sec, време перехода к конечному уровню
 uint8_t   device_status;     //STOP=0//RUN=1//PAUSE=2///SLEEP=3
 uint8_t   btn_mode;          //enable\disable
 uint8_t   battery_voltage;   //30-42vdc
 uint8_t   battery_percent;   //0-100%
 uint8_t   hv_voltage;        //0\16\19 vdc
 uint8_t   hv_current;        //xx-xxx mA
 uint8_t   hv_current_status; //0-100%

} ;


#define APP_ID_SIZE 16
struct struct_application {
  
  const uint8_t app_id_size;

  uint32_t datetime_offset          ; //время в секундах
  uint8_t  app_id[APP_ID_SIZE]      ; //uint32_t  app_id[4]      = {0};
  uint8_t  app_id_last[APP_ID_SIZE] ; //uint32_t  app_id_last[4] = {0};
  uint16_t package_id               ;
  uint16_t send_error               ; //последняя ошибка при отправки BLE пакета
  uint16_t last_error               ; //последняя ошибка 
  uint16_t package_send_counter     ;

  uint32_t quote                    ; //размер квоты, сек
  uint16_t quote_id                 ; //Идентификатор(инкрементируемый на сервере) добавляемой квоты
  uint32_t quote_last_id            ; //идентификатор последней добавленной квоты
  uint32_t quote_used               ; //размер за все время использованной квоты, сек
  uint32_t quote_add                ; //размер за все время добавленной квоты, сек

  uint8_t  prog_launch_id           ; //Номер запуска программы (инкрементируется устройством)

  uint32_t flash_write_counter      ; //счетчик записи во flash
};


void dev_init();
void Load_Default_Programm(void);
void TIMER1_IRQHandler();
void TIMER2_IRQHandler();
void m_timeout_handler(void) ;
//void m_timeout_100ms_handler(void * p_context);

void delay100(uint8_t delay);

void PROGRAM_VALUES(uint16_t pckg, uint8_t cmd);
void dev_main_loop();

void Stm32_Send_BtnClick(uint8_t btn,uint8_t btn_status);
void Stm32_Send_Params(void);

void Stm32_Start_Devive(void);
void Stm32_Stop_Devive(void);
void Stm32_Pause_Devive(void);

void Stm32_Sleep_On(void);
void Stm32_Sleep_Off(void);

void Stm32_Ping(void);

void spi_parce(void);

void Save_Paramms(void);
bool Load_Paramms(void);

