#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "flash.h"

#define FLASH_WAIT_READY { \
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {__WFE();}; \
}

//------------------- FLASH ---------------------//
//#define PAGE_START 39 //160kB max page=47 (0x0002F000)
//#define PAGE_START_ADDR 0x0027000

#define PAGE_1 45 //160kB max
#define PAGE_1_ADDR 0x002D000

#define PAGE_2 46 //160kB max
#define PAGE_2_ADDR 0x002E000


#define FLASH_BUFF_SIZE 1024 //0x00001000
uint32_t flash_buff[FLASH_BUFF_SIZE]={0};
const uint16_t flash_buff_size=FLASH_BUFF_SIZE;

uint8_t wait_flash_timer=0;

//read_word = (*(__IO uint32_t*)PAGE_START_ADDR);
//result=sd_flash_page_erase(PAGE_START);
//sd_flash_write(PAGE_START_ADDR,  flash_buff ,64);
//----------------- FLASH END---------------------//

uint32_t Flash_Read(uint32_t addr) {
  return  (*(__IO uint32_t*)addr);
}

//static void flash_page_erase(uint32_t * page_address)
//{
//    // Turn on flash erase enable and wait until the NVMC is ready:
//    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
//    FLASH_WAIT_READY;
//
//    // Erase page:
//    NRF_NVMC->ERASEPAGE = (uint32_t)page_address;
//    FLASH_WAIT_READY;
//
//    // Turn off flash erase enable and wait until the NVMC is ready:
//    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
//    FLASH_WAIT_READY;
//}

//Flash Storage Example

void Flash_Write_Page(uint32_t *buff,  uint16_t buff_size)
{
    uint32_t f_addr = PAGE_1_ADDR;// 0x0007f000;
    uint32_t *addr = (uint32_t *)f_addr;
    //uint32_t tempa[4] = {0x123456aa,0xaa669977,0xaa559977,0x55559977};
    uint32_t *address = (uint32_t *)PAGE_1_ADDR;
    uint16_t i=0;
    uint32_t retval=0;

    sd_flash_page_erase(PAGE_1);
    wait_flash_timer=2;
    while (wait_flash_timer!=0) {__WFE();}

    retval=sd_flash_write(addr,  flash_buff , 64);
    if (retval!=NRF_SUCCESS) {
        
    }
    wait_flash_timer=2;
    while (wait_flash_timer!=0) {__WFE();}

///---------------------------------------------------------------------
//    // Turn on flash write enable and wait until the NVMC is ready:
//    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
//    FLASH_WAIT_READY;
//
//     // Erase page:
//    NRF_NVMC->ERASEPAGE = (uint32_t)PAGE_1_ADDR;
//    FLASH_WAIT_READY;
//
//    for (i=0;i<4;i++) {
//      *(addr+i) = tempa[i];
//      FLASH_WAIT_READY;
//      buff[i] = (uint32_t)(addr+i); 
//    }
//
////    for (i=0;i<buff_size;i++) {
////      *(addr+i) = buff[i];
////      FLASH_WAIT_READY;
////      buff[i] = (uint32_t)(addr+i); 
////    }
//
//
//   // Turn off flash write enable and wait until the NVMC is ready:
//   NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
//   FLASH_WAIT_READY;
}

void Flash_Read_Page(uint32_t *buff,  uint16_t buff_size) {
  uint16_t page1=45;
  uint32_t page_status=0;
  uint16_t i=0;
  uint32_t page_addr=PAGE_1_ADDR;
  uint32_t addr=0;

//----------------------------------------------------//
  //memcpy(buff,(uint32_t*)page_addr,buff_size);
  //FLASH_WAIT_READY;

  for (i=0;i<buff_size;i++) {
    addr=page_addr+4*i; //0x002D000 sizeof(uint32_t)
    buff[i]=(*(__IO uint32_t*)addr);
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {__WFE();}
  }


}//void Flash_Read_Page(uint32_t *buff,  uint16_t buff_size) {

void Read_from_Flash_to_Buffer(void) {
  Flash_Read_Page(flash_buff,flash_buff_size);
}

void Write_Buffer_to_Flash(void) {
  Flash_Write_Page(flash_buff,flash_buff_size);
}



/** @} */