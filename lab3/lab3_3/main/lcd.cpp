/*!
 * @file DFRobot_RGBLCD1602.cpp
 * @brief DFRobot_RGBLCD1602 class infrastructure, the implementation of basic methods
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @maintainer [yangfeng](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2021-09-24
 * @url https://github.com/DFRobot/DFRobot_RGBLCD1602
 */

 #include <stdio.h>
 #include <string.h>
 
 
 #include "lcd.h"

 #define delayMicroseconds vTaskDelay
 #define delay vTaskDelay

#define I2C_MASTER_SCL_IO           8       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           10       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          400000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
 
 const uint8_t color_define[4][3] = 
 {
     {255, 255, 255},            // white
     {255, 0, 0},                // red
     {0, 255, 0},                // green
     {0, 0, 255},                // blue
 };
 
 /*******************************public*******************************/
 LCDDisplayManager::LCDDisplayManager(uint8_t RGBAddr,uint8_t lcdCols,uint8_t lcdRows, uint8_t lcdAddr)
 {
   _lcdAddr = lcdAddr;
   _RGBAddr = RGBAddr;
   _cols = lcdCols;
   _rows = lcdRows;
 }

 void LCDDisplayManager::i2c_master_init()
{
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = I2C_MASTER_NUM;
    bus_config.sda_io_num = (gpio_num_t) I2C_MASTER_SDA_IO;
    bus_config.scl_io_num = (gpio_num_t) I2C_MASTER_SCL_IO;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &_bus_handle));

    i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = _lcdAddr;
    dev_config.scl_speed_hz = I2C_MASTER_FREQ_HZ;
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(_bus_handle, &dev_config, &_dev_handle));

    i2c_device_config_t rgb_config = {};
    rgb_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    rgb_config.device_address = _RGBAddr;
    rgb_config.scl_speed_hz = I2C_MASTER_FREQ_HZ;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(_bus_handle, &rgb_config, &_rgb_handle));
}
 
 void LCDDisplayManager::init()
 {
   i2c_master_init();
   if(_RGBAddr == (0x60)){
     REG_RED   =      0x04;
     REG_GREEN =      0x03;
     REG_BLUE  =      0x02;
     REG_ONLY  =      0x02 ;
   } else if(_RGBAddr == (0x60>>1)){
     REG_RED      =   0x06 ;       // pwm2
     REG_GREEN    =   0x07 ;       // pwm1
     REG_BLUE     =   0x08 ;       // pwm0
     REG_ONLY     =   0x08 ;
   } else if(_RGBAddr == (0x6B)){
     REG_RED      =   0x06 ;       // pwm2
     REG_GREEN    =   0x05 ;       // pwm1
     REG_BLUE     =   0x04 ;       // pwm0
     REG_ONLY     =   0x04 ; 
   } else if(_RGBAddr == (0x2D)){
     REG_RED      =   0x01 ;       // pwm2
     REG_GREEN    =   0x02 ;       // pwm1
     REG_BLUE     =   0x03 ;       // pwm0
     REG_ONLY     =   0x01 ; 
   }
   _showFunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
   begin(_rows);
 }
 
 void LCDDisplayManager::clear()
 {
     command(LCD_CLEARDISPLAY);        // clear display, set cursor position to zero
     delayMicroseconds(2000);          // this command takes a long time!
 }
 
 void LCDDisplayManager::home()
 {
     command(LCD_RETURNHOME);        // set cursor position to zero
     delayMicroseconds(2000);        // this command takes a long time!
 }
 
 void LCDDisplayManager::noDisplay()
 {
     _showControl &= ~LCD_DISPLAYON;
     command(LCD_DISPLAYCONTROL | _showControl);
 }
 
 void LCDDisplayManager::display() {
     _showControl |= LCD_DISPLAYON;
     command(LCD_DISPLAYCONTROL | _showControl);
 }
 
 void LCDDisplayManager::stopBlink()
 {
     _showControl &= ~LCD_BLINKON;
     command(LCD_DISPLAYCONTROL | _showControl);
 }
 void LCDDisplayManager::blink()
 {
     _showControl |= LCD_BLINKON;
     command(LCD_DISPLAYCONTROL | _showControl);
 }
 
 void LCDDisplayManager::noCursor()
 {
     _showControl &= ~LCD_CURSORON;
     command(LCD_DISPLAYCONTROL | _showControl);
 }
 
 void LCDDisplayManager::cursor() {
     _showControl |= LCD_CURSORON;
     command(LCD_DISPLAYCONTROL | _showControl);
 }
 
 void LCDDisplayManager::scrollDisplayLeft(void)
 {
     command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
 }
 
 void LCDDisplayManager::scrollDisplayRight(void)
 {
     command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
 }
 
 void LCDDisplayManager::leftToRight(void)
 {
     _showMode |= LCD_ENTRYLEFT;
     command(LCD_ENTRYMODESET | _showMode);
 }
 
 void LCDDisplayManager::rightToLeft(void)
 {
     _showMode &= ~LCD_ENTRYLEFT;
     command(LCD_ENTRYMODESET | _showMode);
 }
 
 void LCDDisplayManager::noAutoscroll(void)
 {
     _showMode &= ~LCD_ENTRYSHIFTINCREMENT;
     command(LCD_ENTRYMODESET | _showMode);
 }
 
 void LCDDisplayManager::autoscroll(void)
 {
     _showMode |= LCD_ENTRYSHIFTINCREMENT;
     command(LCD_ENTRYMODESET | _showMode);
 }
 
 void LCDDisplayManager::customSymbol(uint8_t location, uint8_t charmap[])
 {
 
     location &= 0x7; // we only have 8 locations 0-7
     command(LCD_SETCGRAMADDR | (location << 3));
     
     
     uint8_t data[9];
     data[0] = 0x40;
     for(int i=0; i<8; i++)
     {
         data[i+1] = charmap[i];
     }
     send(data, 9);
 }
 
 void LCDDisplayManager::setCursor(uint8_t col, uint8_t row)
 {
 
     col = (row == 0 ? col|0x80 : col|0xc0);
     uint8_t data[3] = {0x80, col};
 
     send(data, 2);
 
 }
 
 void LCDDisplayManager::setRGB(uint8_t r, uint8_t g, uint8_t b)
 {
   uint16_t temp_r,temp_g,temp_b;
   if(_RGBAddr == 0x60>>1){
     temp_r = (uint16_t)r*192/255;
     temp_g = (uint16_t)g*192/255;
     temp_b = (uint16_t)b*192/255;
     setReg(REG_RED, temp_r);
     setReg(REG_GREEN, temp_g);
     setReg(REG_BLUE, temp_b);
   } else{
     setReg(REG_RED, r);
     setReg(REG_GREEN, g);
     setReg(REG_BLUE, b);
     if(_RGBAddr == 0x6B){
       setReg(0x07, 0xFF);
     }
   }
 
 }
 
 void LCDDisplayManager::setColor(uint8_t color)
 {
     if(color > 3)return ;
     setRGB(color_define[color][0], color_define[color][1], color_define[color][2]);
 }
 
 
 inline size_t LCDDisplayManager::write(uint8_t value)
 {
 
     uint8_t data[3] = {0x40, value};
     send(data, 2);
     return 1; // assume sucess
 }
 
 inline void LCDDisplayManager::command(uint8_t value)
 {
     uint8_t data[3] = {0x80, value};
     send(data, 2);
 }
 
 
 
 void LCDDisplayManager::setBacklight(bool mode){
     if(mode){
         setColorWhite();		// turn backlight on
     }else{
         closeBacklight();		// turn backlight off
     }
 }
 
 /*******************************private*******************************/
 void LCDDisplayManager::begin( uint8_t rows, uint8_t charSize) 
 {
     if (rows > 1) {
         _showFunction |= LCD_2LINE;
     }
     _numLines = rows;
     _currLine = 0;
     ///< for some 1 line displays you can select a 10 pixel high font
     if ((charSize != 0) && (rows == 1)) {
         _showFunction |= LCD_5x10DOTS;
     }
 
     ///< SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
     ///< according to datasheet, we need at least 40ms after power rises above 2.7V
     ///< before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
     delay(50);
 
     ///< this is according to the hitachi HD44780 datasheet
     ///< page 45 figure 23
 
     ///< Send function set command sequence
     command(LCD_FUNCTIONSET | _showFunction);
     delay(5);  // wait more than 4.1ms
     
     ///< second try
     command(LCD_FUNCTIONSET | _showFunction);
     delay(5);
 
     ///< third go
     command(LCD_FUNCTIONSET | _showFunction);
 
     ///< turn the display on with no cursor or blinking default
     _showControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
     display();
 
     ///< clear it off
     clear();
 
     ///< Initialize to default text direction (for romance languages)
     _showMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
     ///< set the entry mode
     command(LCD_ENTRYMODESET | _showMode);
     
     if(_RGBAddr == (0xc0>>1)){
       ///< backlight init
       setReg(REG_MODE1, 0);
       ///< set LEDs controllable by both PWM and GRPPWM registers
       setReg(REG_OUTPUT, 0xFF);
       ///< set MODE2 values
       ///< 0010 0000 -> 0x20  (DMBLNK to 1, ie blinky mode)
       setReg(REG_MODE2, 0x20);
     }else if(_RGBAddr == (0x60>>1)){
        setReg(0x01, 0x00);
        setReg(0x02, 0xfF);
        setReg(0x04, 0x15);
     }else if(_RGBAddr==0x6B){
         setReg(0x2F, 0x00);
         setReg(0x00, 0x20);
         setReg(0x01, 0x00);
         setReg(0x02, 0x01);
         setReg(0x03, 4);
     }
     setColorWhite();
 }
 
 void LCDDisplayManager::send(uint8_t *data, uint8_t len)
 {
    i2c_master_transmit(_dev_handle, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    /*
     _pWire->beginTransmission(_lcdAddr);        // transmit to device #4
     for(int i=0; i<len; i++) {
         _pWire->write(data[i]);
         delayMicroseconds(100);
     }
     _pWire->endTransmission();                     // stop transmitting*/
 }
 
 void LCDDisplayManager::setReg(uint8_t addr, uint8_t data)
 {
    uint8_t tosend[2] = {addr, data};
    i2c_master_transmit(_rgb_handle, tosend, sizeof(tosend), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    /*
     _pWire->beginTransmission(_RGBAddr); // transmit to device #4
     _pWire->write(addr);
     _pWire->write(data);
     _pWire->endTransmission();    // stop transmitting */
 }

 void LCDDisplayManager::printstr(char * str){
    int i = 0;
    while(str[i] != '\0'){
        write((uint8_t)str[i]);
        i++;
    }
 }

 