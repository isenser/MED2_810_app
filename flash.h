

uint32_t Flash_Read(uint32_t addr);
uint32_t Flash_Erase_Page(uint16_t page);
void Flash_Write_Page(uint32_t *buff,  uint16_t buff_size);
void Flash_Read_Page(uint32_t *buff,  uint16_t buff_size);
void Read_from_Flash_to_Buffer(void);
void Write_Buffer_to_Flash(void);