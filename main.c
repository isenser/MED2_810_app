#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
//// <o> NRF_SDH_CLOCK_LF_SRC  - SoftDevice clock source.
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"

//#define NRFX_SAADC_ENABLED 1
#include "nrf_drv_saadc.h"
#include "nrf_saadc.h"


//#include "app_uart.h"
//#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_drv_wdt.h"

#include "nrf_drv_gpiote.h"

#include "nrf_drv_spi.h"

//#if defined (UART_PRESENT)
//#include "nrf_uart.h"
//#endif
//#if defined (UARTE_PRESENT)
//#include "nrf_uarte.h"
//#endif

//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
//#include "nrf_log_default_backends.h"

#define DEBUG_MODE   1

#include "my_ble_nus.h"
#include "dev.h"
#include "prog.h"
#include "flash.h"

//DEVICE SDK_CONFIG
//0 NRF52 - DK
//1 Nrf52810 //pca10040e
//2 Nrf52811 //pca10056e

//#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)  

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "MED_BLE"                                  /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           0x02//BLE_UUID_TYPE_VENDOR_BEGIN           /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_DURATION                10000//3000                                  /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_0_625_MS)            //UNIT_1_25_MS /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_0_625_MS)            //UNIT_1_25_MS /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                64                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                64                                         /**< UART RX buffer size. */

//#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600         /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
//#define ADC_PRE_SCALING_COMPENSATION    6           /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define ADC_RES_10BIT                   1024 
static  nrf_saadc_value_t adc_buf[1];

//#define NRF_ATOMIC_USE_BUILD_IN 1
//typedef volatile uint32_t nrf_atomic_u32_t; 
uint16_t  batt_lvl_in_milli_volts=BATTERY_FULL;

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


//MAIN DEF
APP_TIMER_DEF(m_app_timer_id);
APP_TIMER_DEF(m_app_timer_1sec_id);
APP_TIMER_DEF(m_battery_timer_id);   /**< Battery measurement timer. */

//--------- MAIN_FUNC ---------------//
static void button_event_handler(uint8_t pin_no, uint8_t button_action);
uint32_t update_send_buffer(uint8_t * p_send_array, uint16_t len);
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context);
static void battery_level_meas_timeout_handler(void * p_context);

//---------- EXTERN --------------- //
extern void m_timeout_100ms_handler(void);
extern void m_timeout_1sec_handler(void);
extern void ble_command_parser(uint8_t *data,uint16_t len);


extern uint8_t  btn_left_click;
extern uint8_t  btn_left_timer;

extern uint8_t  btn_right_click;
extern uint8_t  btn_right_timer;

extern uint8_t  ledb_timer;
extern uint8_t  led_blink_timer;
extern bool stm32_ping;
extern bool stm32_answer;

//>>> to Phone
#define SEND_ARRAY_SIZE 64
uint8_t send_array[SEND_ARRAY_SIZE]={0};

//<<< from Phone
#define RECEIVE_ARRAY_SIZE 64
uint8_t receive_array[RECEIVE_ARRAY_SIZE]={0};

//SPI
#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done=true;  /**< Flag used to indicate that SPI instance completed the transfer. */
//static spi_crc_ok = false;

#define SPI_RX_TX_SIZE 32
uint8_t       spi_tx_buf[SPI_RX_TX_SIZE];      /**< TX buffer. */
uint8_t       spi_rx_buf[SPI_RX_TX_SIZE];      /**< RX buffer. */
const uint8_t m_length = SPI_RX_TX_SIZE;       /**< Transfer length. */

void spi_send();
void SleepMode(void);

extern uint8_t led_blink_timer;
extern struct  struct_prog    programm; 
extern struct  struct_status_device_now   status_device_now;

extern struct  struct_application        app;

bool sleeping              = true;
bool advestering           = false;
bool ble_connection_status = false;
bool device_work           = false;
bool save_to_flash         = false;
bool ble_send_ready        = false;


nrf_drv_wdt_channel_id m_channel_id;


//void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event);


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_app_timer_id, 
                                APP_TIMER_MODE_REPEATED, 
                                (app_timer_timeout_handler_t)m_timeout_100ms_handler); //NULL
    APP_ERROR_CHECK(err_code); 

    err_code = app_timer_create(&m_app_timer_1sec_id, 
                                APP_TIMER_MODE_REPEATED, 
                                (app_timer_timeout_handler_t)m_timeout_1sec_handler); //NULL
    APP_ERROR_CHECK(err_code);

    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                (app_timer_timeout_handler_t)battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

static void timer_100ms_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_app_timer_id, 1628, m_timeout_100ms_handler);//m_timeout_100ms_handler); //1638
    APP_ERROR_CHECK(err_code);
}

static void timer_100ms_stop(void)
{
    ret_code_t err_code;
    err_code = app_timer_stop(m_app_timer_id); 
    APP_ERROR_CHECK(err_code);
}

//1sec
/**@brief Function for starting timers.
 */
static void timer_1sec_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_app_timer_1sec_id, 16380, m_timeout_1sec_handler);  //16280
    APP_ERROR_CHECK(err_code);
    
//    err_code = app_timer_start(m_battery_timer_id, 16385, battery_level_meas_timeout_handler); //
//    APP_ERROR_CHECK(err_code);
}

static void timer_1sec_stop(void)
{  
  ret_code_t err_code;
  err_code = app_timer_stop(m_app_timer_1sec_id);
  APP_ERROR_CHECK(err_code);

//  err_code = app_timer_stop(m_battery_timer_id);
//  APP_ERROR_CHECK(err_code);
}

static void timer_1sec_adc_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_battery_timer_id, 3*16385, battery_level_meas_timeout_handler); //
    APP_ERROR_CHECK(err_code);
}

static void timer_1sec_adc_stop(void)
{  
  ret_code_t err_code;
  err_code = app_timer_stop(m_battery_timer_id);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

uint32_t update_send_buffer(uint8_t * p_send_array, uint16_t len)
{
  ret_code_t err_code;

  if ((ble_connection_status) && (ble_send_ready)) 
   {
      err_code = ble_nus_data_send(&m_nus, p_send_array, &len, m_conn_handle);
      if ((err_code != NRF_ERROR_INVALID_STATE) &&
      (err_code != NRF_ERROR_RESOURCES) &&
      (err_code != NRF_ERROR_NOT_FOUND))
      {
        APP_ERROR_CHECK(err_code);
      }
      return err_code;
  } else  {
     return 50;
  }
}//static void m_timeout_handler(void * p_context)

/**
 * @details This function will process the data 
 * received from the Nordic UART BLE Service and send
 *          it to the UART module.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
//static void nus_data_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t *data, uint8_t len)
{
   uint16_t len=0;

   if (p_evt->type == BLE_NUS_EVT_RX_DATA)
   {
     memset(&receive_array,0x00,RECEIVE_ARRAY_SIZE);
     if (p_evt->params.rx_data.length<RECEIVE_ARRAY_SIZE) {
        memcpy(receive_array,p_evt->params.rx_data.p_data,p_evt->params.rx_data.length);
        len=p_evt->params.rx_data.length;
     } else {
       memcpy(receive_array,p_evt->params.rx_data.p_data,RECEIVE_ARRAY_SIZE);
       len=RECEIVE_ARRAY_SIZE;
     }
     

     ble_command_parser(receive_array,len);
   }

   if(p_evt->type == BLE_NUS_EVT_TX_RDY)//Service is ready to accept new data to be transmitted.
   {
      ble_send_ready=true;
   }
 
    if(p_evt->type == BLE_NUS_EVT_COMM_STARTED)//Notification has been enabled
    {
      ble_send_ready=true;
    }


}
/**@snippet [Handling the data received over BLE] */



/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
    //ble_connection_status=false;
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;

    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;


    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    timer_100ms_stop();
    LED_OFF;
    LED_R_OFF;
    LED_G_OFF;
    LED_B_OFF;
    STM32_SLEEP_ON;
    sleeping=true;
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            /*< Idle; no connectable advertising is ongoing.*/
            //sleep_mode_enter();
            advestering=false;
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
uint32_t ble_evtn=0;
uint32_t ble_rssi=0;
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
    ble_evtn = p_ble_evt->header.evt_id;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            ble_connection_status=true;
            break;

        case BLE_GAP_EVT_PHY_UPDATE:
            //ble_connection_status=true;
            break;

        case BLE_GAP_EVT_RSSI_CHANGED:
                ble_rssi = p_ble_evt->evt.gap_evt.params.rssi_changed.rssi;
                break;

        case BLE_GAP_EVT_DISCONNECTED:
            ble_connection_status=false; // 1- diconnect
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            //NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_ADV_SET_TERMINATED:
            if (p_ble_evt->evt.gap_evt.params.adv_set_terminated.reason == BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_TIMEOUT)
            {
              //sleep_mode_enter();
              advestering=false;
              ble_connection_status=false;
                // Go to system-off mode (this function will not return; wakeup will cause a reset).
                //err_code = sd_power_system_off();
                //APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            ble_connection_status=false;
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            ble_connection_status=false;
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}//static void ble_stack_init(void)


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        //NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;
    //init.config.ble_adv_on_disconnect_disabled = true;      //ADD THIS LINE TO PREVENT ADVERTISING ON THE DISCONNECTED EVENT


    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


// Interrupt handler
//E:\YandexDisk\DATASHEETS\_NORDIC\nRF5SDK17009d13099\nRF5_SDK_17.0.0_9d13099\modules\nrfx\drivers\src\nrfx_gpiote.c
//nrfx_gpiote.c nrfx_gpiote_irq_handler

//#define BTN_LONG_TIME 15 //*100ms dev.h
void GPIOTE_IRQHandler(){

   if (NRF_GPIOTE->EVENTS_IN[0] != 0) { //BTN LEFT Int
      NRF_GPIOTE->EVENTS_IN[0] = 0;
      if (nrf_gpio_pin_read(BTN_LEFT_PIN)==0) {
          //BTN_LEFT_CLICK
          btn_left_timer=1;
          #if !defined (DEBUG_MODE)
             nrf_gpio_pin_write(12,1);
          #endif
          if (sleeping) {
             sleeping=false;
             ledb_timer    = 0;
             led_blink_timer=0;
             timer_100ms_start();
          }
      }//if (nrf_gpio_pin_read(BTN_LEFT_PIN)==0) {

      if (nrf_gpio_pin_read(BTN_LEFT_PIN)==1) {
         //BTN_LEFT_UNCLICK
         #if !defined (DEBUG_MODE) 
             nrf_gpio_pin_write(12,0);
         #endif
        
         if (btn_left_timer<BTN_LONG_TIME) { //*100=3sec
           btn_left_click=1; //short
           if (device_work==false) 
           {
              timer_100ms_stop();
              sleeping=true;
              btn_left_click=0;
           }
         }
         btn_left_timer=0;
      }//if (nrf_gpio_pin_read(BTN_LEFT_PIN)==1) {
    }//if(NRF_GPIOTE->EVENTS_IN[0] != 0)

    if (device_work) {
      if (NRF_GPIOTE->EVENTS_IN[1] != 0) {//BTN RIGTH
         NRF_GPIOTE->EVENTS_IN[1] = 0;
         btn_right_click=nrf_gpio_pin_read(BTN_RIGHT_PIN);
         if (nrf_gpio_pin_read(BTN_RIGHT_PIN)==0) {
            //BTN_RIGHT_LO
            btn_right_timer=1;
         }
         if (nrf_gpio_pin_read(BTN_RIGHT_PIN)==1) {
           //BTN_RIGHT_HI
           if (btn_right_timer>BTN_LONG_TIME) {
             btn_right_click=2;
           } else {
             btn_right_click=1;
           }
           btn_right_timer=0;
         }
      }//if(NRF_GPIOTE->EVENTS_IN[1] != 0)
     } else {
        if (NRF_GPIOTE->EVENTS_IN[1] != 0) {//BTN RIGTH
           NRF_GPIOTE->EVENTS_IN[1] = 0;
           btn_right_click=0;
           btn_right_timer=0;
        }

     }// else if (device_work) {
   

   if (NRF_GPIOTE->EVENTS_IN[2] != 0) { //SPI_SLAVE_ANSWER
      //SPI_ANSWER
      NRF_GPIOTE->EVENTS_IN[2] = 0;
      memset(spi_tx_buf, 0, SPI_RX_TX_SIZE);
      spi_send();
    }

}//void GPIOTE_IRQHandler(){

/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
  //BTN_INTERRUPT
  nrf_gpio_cfg_input(BTN_LEFT_PIN,NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(BTN_RIGHT_PIN,NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(SPI_PIN_SLAVE,NRF_GPIO_PIN_PULLDOWN);

  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Set;
  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk; //Set GPIOTE interrupt register on channel 0
  nrf_gpiote_event_configure(0, BTN_LEFT_PIN, GPIOTE_CONFIG_POLARITY_Toggle); //NRF_GPIOTE_POLARITY_TOGGLE
  nrf_gpiote_event_enable(0);
  nrf_gpiote_event_clear(BTN_LEFT_PIN);
  nrf_gpiote_int_enable(BTN_LEFT_PIN);
    
  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN1_Set;
  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN1_Msk; //Set GPIOTE interrupt register on channel 0
  nrf_gpiote_event_configure(1, BTN_RIGHT_PIN, GPIOTE_CONFIG_POLARITY_Toggle); //NRF_GPIOTE_POLARITY_TOGGLE
  nrf_gpiote_event_enable(1);
  nrf_gpiote_event_clear(BTN_RIGHT_PIN);
  nrf_gpiote_int_enable(BTN_RIGHT_PIN);

  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN2_Set;
  NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN2_Msk; //Set GPIOTE interrupt register on channel 0
  nrf_gpiote_event_configure(2, SPI_PIN_SLAVE, GPIOTE_CONFIG_POLARITY_LoToHi); //NRF_GPIOTE_POLARITY_TOGGLE
  nrf_gpiote_event_enable(2);
  nrf_gpiote_event_clear(SPI_PIN_SLAVE);
  nrf_gpiote_int_enable(SPI_PIN_SLAVE);

  NVIC_EnableIRQ(GPIOTE_IRQn); //Enable interrupts
  NVIC_SetPriority(GPIOTE_IRQn,APP_IRQ_PRIORITY_MID);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

//----------------------------- ADC ---------------------------------------------

float f_battery=0;
float k = 0.3;  // коэффициент фильтрации, 0.0-1.0
// бегущее среднее
float battery_expRunningAverage(float newVal) {
  static float filVal = 0;
  //init
  if (filVal==0) {
    filVal=newVal;
    return filVal;
  }
  //filter
  filVal += (newVal - filVal) * k;
  return filVal;
}

nrf_saadc_value_t adc_result;
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;

    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);

    NRF_WDT->RR[0] = WDT_RR_RR_Reload;  //Reload watchdog register 0
}

/**@brief Function for handling the ADC interrupt.
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
*/
//uint16_t adc_event_counter=0;
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
      //nrf_saadc_value_t adc_result;
      //uint8_t           percentage_batt_lvl;
      uint32_t          err_code;
      static uint8_t bt_counter=0;
      
      //adc_result = p_event->data.done.p_buffer[0];

      err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
      APP_ERROR_CHECK(err_code);

      if (bt_counter>5) {
        uint16_t batt=0;
        batt = (uint16_t)((uint32_t)(3810*adc_buf[0])/810);
        f_battery=battery_expRunningAverage(batt);
        if (f_battery<2000) {f_battery=2000;}
        batt_lvl_in_milli_volts=(uint16_t)(f_battery);
        status_device_now.battery_voltage = (uint8_t)(batt_lvl_in_milli_volts/100); //3920/10=39 (3,9v)
       } else {
          bt_counter++;
          batt_lvl_in_milli_volts=4200;
          status_device_now.battery_voltage = (uint8_t)(batt_lvl_in_milli_volts/100); //3920/10=39 (3,9v)
       }

      if (batt_lvl_in_milli_volts>=BATTERY_CUT) {
         status_device_now.battery_percent = (uint8_t)((batt_lvl_in_milli_volts-BATTERY_CUT)*100/(BATTERY_FULL-BATTERY_CUT));
         if (status_device_now.battery_percent>100) {status_device_now.battery_percent=100;}
       } else {
         status_device_now.battery_percent=0;
       }

//    percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);     
    }

    //adc_event_counter++;
}


/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void)
{
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);//P0.04/AIN2 NRF_SAADC_INPUT_VDD);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1); //can be used to start conversion in non-blocking mode.
    //err_code = nrf_drv_saadc_sample(); // sets the SAADC up for conversion, but does not trigger sampling.
    APP_ERROR_CHECK(err_code);
}


static void advertising_start(void)
{
   uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
   APP_ERROR_CHECK(err_code);
}

static void if_advestering(void) {
  if (advestering==false) {
   advestering=true;
   advertising_start();
  }
}//void if_advestering(void) {

static void ble_disconnect(void) 
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
      err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      if (err_code != NRF_ERROR_INVALID_STATE)
      {
          APP_ERROR_CHECK(err_code);
      }
    }
    while (ble_connection_status==true) { __WFE();}

    err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
    if (err_code != NRF_ERROR_INVALID_STATE) {
        APP_ERROR_CHECK(err_code);
    }
  
    advestering=false;

    sleep_mode_enter();
}//void ble_disconnect(void) 



void spi_init(void) {
  //SPI
    #define SPI_SS_PIN    10
    #define SPI_MISO_PIN  6
    #define SPI_MOSI_PIN  5
    #define SPI_SCK_PIN   9

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin    = SPI_SS_PIN;
    spi_config.miso_pin  = SPI_MISO_PIN;
    spi_config.mosi_pin  = SPI_MOSI_PIN;
    spi_config.sck_pin   = SPI_SCK_PIN;
    spi_config.frequency = NRF_DRV_SPI_FREQ_2M;
    nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL);
} 

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    uint8_t len; 
    if(p_event->type == NRF_DRV_SPI_EVENT_DONE) { 
       
       spi_xfer_done = true;
       //Transfer completed.
    
      if (spi_rx_buf[1] != 0) //LEN
      {
         spi_parce();
         //memset(spi_tx_buf, 0, m_length);
      }
    }
}//void spi_event_handler(

void spi_send() {//uint8_t *buff,uint8_t lenght) {
  uint8_t i;
  uint8_t len;
  uint8_t crc=0;

  memset(spi_rx_buf, 0, m_length);//spi_rx_buf=0
  spi_xfer_done = false;

  //ADD_CRC
  //len=spi_tx_buf[0]; //len
  //for (i=0;i<len;i++) {
  //  crc+=spi_tx_buf[i];
  //}
  //spi_tx_buf[len+1] = crc;
  //spi_crc_ok = false;
  
  nrf_drv_spi_transfer(&spi, spi_tx_buf, m_length, spi_rx_buf, m_length);

  //if (device_work) { 
  //  LED_ON;//nrf_gpio_pin_write(PIN_LED,0); //LED_B ON
  //  led_blink_timer=1;//*100ms
  // }

 // while (spi_xfer_done==false) { __WFE();}
}

//Supported tx_power values: -40dBm, -20dBm, -16dBm, -12dBm, -8dBm, -4dBm, 0dBm, +3dBm and +4dBm.
#define TX_POWER_LEVEL (-8)  /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */
static void tx_power_set(void)
{
    ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}

void wdt_init(void)
{
  //Configure Watchdog. a) Pause watchdog while the CPU is halted by the debugger.  b) Keep the watchdog running while the CPU is sleeping.
  NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);   
  NRF_WDT->CRV = 5*32768;             //ca 5 sek. timout
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;  //Enable reload register 0
  
  // Enable WDT interrupt:
  NVIC_EnableIRQ(WDT_IRQn);
  NRF_WDT->INTENSET = WDT_INTENSET_TIMEOUT_Msk;	
  NRF_WDT->TASKS_START = 1;//Start the Watchdog timer
}

void WDT_IRQHandler(void)
{     
  //The WDT interrupt handler will have around 2 32kHz clock cycles 
  //to execute before reset, so you can not actully see this LED blink with your eye.
  LEDs_ON;

  //NRF_WDT->RR[0] = WDT_RR_RR_Reload;  //Reload watchdog register 0
}

void Led_Blink(uint8_t count) {
  uint8_t i=0;

  timer_100ms_start();
  for (i=0;i<count;i++) {
    LED_R_ON;delay100(2);LED_R_OFF;delay100(5);
  }
  //sleep_mode_enter();
 }

uint32_t err_code;
uint8_t  dev_err;
/**@brief Application main function.
 */
int main(void)
{

    wdt_init();//Initialize watchdog

  // Initialize.
    timers_init();
    buttons_init();
    
    power_management_init();

  // Initialize BLE.
    ble_stack_init();

    gap_params_init();
    gatt_init();
    
    conn_params_init();
    services_init();

    advertising_init();
    
  //Initialize MAIN
    // Init_param_device
    dev_init();

    spi_init();

    //NRFX_SAADC_CONFIG_LP_MODE 1 //Enabling low power mode
    adc_configure();
    timer_1sec_adc_start();

    //WDT+TIM_1sec = 0,16ma   //-SPI - ADC - ADC_TIM1
    //SPI+TIM_1sec = 0,16ma   //

    Read_from_Flash_to_Buffer();
    if (Load_Paramms()==false) { 
      Load_Default_Programm();
    }

    //Load_Default_Programm();

    timer_1sec_start();

    sleep_mode_enter();

    // Enter main loop.
    for (;;)
    {
        //main_loop
        if (device_work) 
        {
          //dev_main_loop();
          if (btn_left_click==1) { //-1 level
            if (status_device_now.device_status!=0) {
               Stm32_Send_BtnClick(0x01,0x01);
            }
            btn_left_click=0;
            if_advestering();
          }

          if (btn_right_click==1) { //+1 level
            if (status_device_now.device_status!=0) {
               Stm32_Send_BtnClick(0x02,0x01);
            }
            btn_right_click=0;
            if_advestering();
          }
        }//if (device_work) 

        //if ((batt_lvl_in_milli_volts<=BATTERY_MIN)&&(batt_lvl_in_milli_volts!=0)) {
        if (batt_lvl_in_milli_volts<BATTERY_CUT) {
            //IF (19v) -> STOP_19V
            //SAVE_TO_FLASH
            Stm32_Stop_Devive();
            while (spi_xfer_done==false) { __WFE();}
            
            device_work=false;

            if (save_to_flash==false) {
              //UPDATE_BUFF
              Save_Paramms();
              timer_100ms_start();
              Write_Buffer_to_Flash();
              save_to_flash=true;
              
              NVIC_DisableIRQ(GPIOTE_IRQn); //DISABLE interrupts
              //STOPS_ALL_TIMERS
              timer_1sec_stop();
              timer_100ms_stop();
              
            }

            sleep_mode_enter();
            STM32_SLEEP_ON;
            ble_disconnect();

            //WAIT_for_charges
        }//if (batt_lvl_in_milli_volts<=BATTERY_CUT) {

        if (save_to_flash==true) {
            //if ((batt_lvl_in_milli_volts>BATTERY_NORMAL)&&(batt_lvl_in_milli_volts!=0)) {
            if (batt_lvl_in_milli_volts>BATTERY_NORMAL) {
             save_to_flash=false;
             NVIC_EnableIRQ(GPIOTE_IRQn); //Enable interrupts
             timer_1sec_start();
            }
        }


        if (btn_left_click==2) //LONG_CLICK 
        {
          if (device_work) {
           //SLEEP DEVICE
              SleepMode();
          } else 
          {
           //START_DEVICE
                 //#define DELAY1 
                 //uint8_t error=0;
                 dev_err=0;
                 //<=9 - battery errors
                 //>=10 - time errorrs

               //if ((batt_lvl_in_milli_volts<=BATTERY_NORMAL)&&(batt_lvl_in_milli_volts!=0)) {
               if (batt_lvl_in_milli_volts<=BATTERY_LOW) {
                  //LOW_BATTERY
                  //BLINK and Sleep
                  Led_Blink(2);
                  sleep_mode_enter();
                  dev_err=2;
               } 

               if (dev_err==0) {
                 //send_ping_to_STM32
                  STM32_SLEEP_OFF;
                  timer_100ms_start();
                  delay100(2); //200ms
                  Stm32_Ping();
                  while (spi_xfer_done==false) { __WFE();}
                  
                  delay100(2); //200ms
                  Stm32_Ping();
                  while (spi_xfer_done==false) { __WFE();}
                  

                  //wait_answer
                  uint8_t count=15;//10;
                  while ((stm32_ping==false) && (count>0)) {
                     delay100(1);
                     count--;
                  }
                  if (stm32_ping==false) {
                     Led_Blink(4);
                     sleep_mode_enter();
                     dev_err=1; //STM32 Connect FAIL
                  }
               }//if (error==0) {

               if (dev_err==0) {
                 if (programm.count_timer==0) 
                 {
                    Led_Blink(5);
                    dev_err=10;
                 } 

                 if (app.quote==0)
                 {
                    Led_Blink(6);
                    dev_err=11;
                 }
               } else {
                  //Led_Blink(5);
               }

              if ((dev_err==0) || (dev_err>=10)) {
                  if_advestering();

                  STM32_SLEEP_OFF;
                  delay100(1); //200ms

                  //send_params_to_stm32
                  Stm32_Send_Params();
                  while (spi_xfer_done==false) { __WFE();}

                  delay100(1);
                  Stm32_Start_Devive();
                  while (spi_xfer_done==false) { __WFE();}

                  ////1-по кнопкам (начинает работать сразу)
                  ////2-со смартфоном(ждет смартфона)
                  ////3-смартфон+кнопки(начинает сразу+кнопки)
                  //if (error==0) 
                  //{
                  //  if (programm.prog_button_mode!=2) {
                  //    Stm32_Start_Devive();
                  //  } else {
                  //    Stm32_Pause_Devive();
                  //  }
                  //} else {
                  //    Stm32_Stop_Devive();
                  //}
                  device_work=true;
              }//if (error==0) {
            }//START_DEVICE   
           btn_left_click=0;
        }//if (btn_left_click==2) 


        //POWER_DOWN
        if (spi_xfer_done) 
        {
          nrf_pwr_mgmt_run();
        }//if (spi_xfer_done)

    }//while (1)
}//main

extern bool send_history;
extern bool subscription_prog_value;
void SleepMode(void) {
 //SLEEP DEVICE
    //stop_impulse
    if (status_device_now.device_status!=0) {
      Stm32_Stop_Devive();
      while (spi_xfer_done==false) { __WFE();}
    }
    device_work=false;

    ble_send_ready  = false;
    send_history    = false;
    subscription_prog_value=false;

    sleep_mode_enter();
    STM32_SLEEP_ON;
    ble_disconnect();
}