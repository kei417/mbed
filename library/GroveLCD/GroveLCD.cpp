#include "mbed.h"
#include "GroveLCD.h"
#include <stdio.h>
#include <string.h>
#include <cstdarg>

char _displayfunction;
char _displaycontrol;

// コンストラクタ
GroveLCD::GroveLCD(I2C& _i2c)
    : i2c(_i2c)
{
    this->init();
}

// ディスプレイＯＮ
void GroveLCD::displayOn() 
{
    _displaycontrol |= LCD_DISPLAYON;
    this->sendCommand(LCD_DISPLAYCONTROL | _displaycontrol);
}

// ディスプレイクリア
void GroveLCD::clear()
{
    this->sendCommand(LCD_CLEARDISPLAY);        
    wait_us(2000);         
}

// 現在のカーソル位置からディスプレイに文字列を表示します。
void GroveLCD::print(char *str)
{   
    char data[2];
    data[0] = 0x40;
    while(*str) {
        data[1] = *str;
        i2c.write(LCD_ADDRESS, data, 2);
        str++;
    }
}

void GroveLCD::printf(const char* format, ... )
{
    std::va_list args;
    va_start(args, format);

    char dummy_buf[1];
    int len = vsnprintf(dummy_buf, sizeof(dummy_buf), format, args);
    char *temp = new char[len + 1];
    vsprintf(temp, format, args);
    print(temp);
    delete[] temp;

    va_end(args);
}


// カーソルを移動します
void GroveLCD::locate(char col, char row)
{
    if(row == 0) {
        col = col | 0x80;
    } else {   
        col = col | 0xc0;
    }
 
    char data[2];
    data[0] = 0x80;
    data[1] = col;
    i2c.write(LCD_ADDRESS, data, 2);
}

// コマンドを送信します
void GroveLCD::sendCommand(char value)
{
    char data[2] = {0x80, value};
    i2c.write(LCD_ADDRESS, data, 2);
}

// ＬＣＤを初期化します
void GroveLCD::init() 
{   
    // 2行 5x10ドット
   _displayfunction |= LCD_2LINE;
   _displayfunction |= LCD_5x10DOTS;
 
   // Wait for more than 30 ms after power rises above 4.5V per the data sheet
    wait_ms(50);


    // Send first function set command. Wait longer that 39 us per the data sheet
    sendCommand(LCD_FUNCTIONSET | _displayfunction);
    wait_us(45);  
    
    // turn the display on
    displayOn();

    // clear the display
    clear();
}

