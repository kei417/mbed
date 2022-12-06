#include "mbed.h"

// I2C addresses
#define LCD_ADDRESS     (0x7c)

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_DISPLAYCONTROL 0x08
#define LCD_FUNCTIONSET 0x20

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00

// flag for entry mode
#define LCD_ENTRYLEFT 0x02

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_2LINE 0x08
#define LCD_5x10DOTS 0x04

/**
 * Grove LCD ライブラリ
 */    
class GroveLCD {
public:
    
    /**
     * コンストラクタ
     *   @param i2c [in] i2Cの参照を指定
     */
    GroveLCD(I2C& i2c); 
    
    /**
     * LCDをクリアします
     */
    void clear();
    
    /**
     * 現在のカーソル位置から文字列を出力します。 
     */
    void print(char *str);
    
    void printf(const char* format, ... );
    
    /**
     * カーソルを移動します
     * @param col [in] カラム
     * @param row [in] 行
     */
    void locate(char col, char row);
    
private:
    
    // デバイスの初期化
    void init();   
    
    //Turn on display
    void displayOn();
    
    //Send command to display
    void sendCommand(char value);
    
    // MBED I2Cへの参照
    I2C& i2c;
};