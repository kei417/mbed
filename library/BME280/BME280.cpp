/*****************************************************************************
 *                                                                           *
 * BOSCH BME208 Temperature Pressure Humidity Sensor Driver / I2C            *
 *                                                                           *
 * (c) 2015 Jens Schneider                                                   *
 * mailto:jens.schneider@kaust.edu.sa                                        *
 * Visual Computing Center (VCC)                                             *
 * King Abdullah University of Science and Technology (KAUST)                *
 * 4700 Thuwal, Kingdom of Saudi Arabia                                      *
 *                                                                           *
 * PERMISSION GRANTED TO USE IN NON-COMMERCIAL AND EDUCATIONAL PROJECTS      *
 * NO RESPONSIBILITY ASSUMED FOR DAMAGE OR LOSS OF ANY HARDWARE OR SOFTWARE  *
 *                                                                           *
 * version 0.1                                                               *
 * last change 07.Nov.2015                                                   *
 *                                                                           *
 *****************************************************************************/
#include"BME280.h"

const uint8_t BME280::REG_HUM_LSB           = 0xFE;
const uint8_t BME280::REG_HUM_MSB           = 0xFD;
const uint8_t BME280::REG_TEMP_XLSB         = 0xFC;
const uint8_t BME280::REG_TEMP_LSB          = 0xFB;
const uint8_t BME280::REG_TEMP_MSB          = 0xFA;
const uint8_t BME280::REG_PRESS_XLSB        = 0xF9;
const uint8_t BME280::REG_PRESS_LSB         = 0xF8;
const uint8_t BME280::REG_PRESS_MSB         = 0xF7;

const uint8_t BME280::REG_CONFIG            = 0xF5;
const uint8_t BME280::REG_CTRL_MEAS         = 0xF4;
const uint8_t BME280::REG_STATUS            = 0xF3;
const uint8_t BME280::REG_CTRL_HUM          = 0xF2;
const uint8_t BME280::REG_CALIB26_41_BASE   = 0xE1;
const uint8_t BME280::REG_RESET             = 0xE0;
const uint8_t BME280::REG_ID                = 0xD0;
const uint8_t BME280::REG_CALIB00_25_BASE   = 0x88;

const uint8_t BME280::VAL_CALIB26_41_SIZE   = 0x10;
const uint8_t BME280::VAL_CALIB00_25_SIZE   = 0x1A;
const uint8_t BME280::VAL_CHIP_ID           = 0x60;
const uint8_t BME280::VAL_RESET             = 0xB6;

const uint8_t BME280::STATUS_IDLE           = 0x00;
const uint8_t BME280::STATUS_MEASURING      = 0x08;
const uint8_t BME280::STATUS_UPDATING       = 0x01;
const uint8_t BME280::STATUS_ERROR          = 0xFF;

const std::string BME280::m_name = std::string("Bosch BME280");

BME280::BME280(void) : m_pI2C(NULL), m_address(0x00), m_fine_temp(0x1F3E6), m_mode(MODE_SLEEP),m_bOk(false) {
    // initialized m_fine_temp to 25C
}

BME280::~BME280(void) {
    done();
}

bool BME280::init(I2C& i2c, uint8_t address) {
    m_bOk = false;
    m_pI2C = &i2c;
    m_address = address<<1;
    m_fine_temp = 0x1F3E6;
    if (!reset()) return false;    
    if (read_chip_id()!=0x60) return false;    
    if (!read_calibration()) return false;    
    m_bOk = true;
    return true;
}

const bool& BME280::is_ok(void) const {
    return m_bOk;
}

bool BME280::done(void) {
    stop();
    m_pI2C = NULL;
    m_address = 0x00;
    m_mode = MODE_SLEEP;
    return true;
}


bool BME280::start( BME280::sampling_t hum, 
                    BME280::sampling_t temp, 
                    BME280::sampling_t press,
                    BME280::standby_t standby,
                    BME280::filter_t filter,
                    BME280::mode_t mode) {
    // 1. Reset
    if (!reset()) return false;
    
    // 2. Write CONFIG
    uint8_t val_cfg = uint8_t(standby)<<5;
    val_cfg|=uint8_t(filter)<<2;
    if (!write8(REG_CONFIG,val_cfg)) return false;    
    
    // 3. Write CTRL_HUM
    if (!write8(REG_CTRL_HUM,uint8_t(hum))) return false;

    // 4. Write CTRL_MEAS
    uint8_t val_meas = uint8_t(temp)<<5;
    val_meas |= uint8_t(press)<<2;
    val_meas |= uint8_t(mode);
    if (!write8(REG_CTRL_MEAS,val_meas)) return false;
    
    m_mode = mode;
    m_bOk  = true;
    
    return true;
}

bool BME280::stop(void) {
    return reset();
}

bool BME280::get(float& temperature, float& pressure, float& humidity) {
    int32_t T;
    uint32_t P, H;
    if (!get(T,P,H)) return false;
    temperature = float(T)*0.01f;           // in degree C
    pressure    = 0.01f*(float(P)/256.0f);  // in hPa / mbar
    humidity    = float(H)/1024.0f;         // in %
    return true;
}

bool BME280::get(int32_t& temperature, uint32_t& pressure, uint32_t& humidity) {
    if (m_mode!=MODE_AUTO) {
        // Trigger forced conversion by updating CTRL_MEAS register
        uint8_t val = 0x00;
        if (!read8(REG_CTRL_MEAS,val)) return false;
        val = (val&0xFC)|uint8_t(MODE_FORCED);
        if (!write8(REG_CTRL_MEAS,val)) return false;
        // Wait for measurement to finish
        do {
            val = read_status();
            if ((val&STATUS_MEASURING)!=0) wait_ms(1);
        }  while(val&STATUS_MEASURING);
    }
    int32_t rawT, rawP, rawH;
    if (!raw_data(rawT,rawP,rawH)) return false;
    temperature = compensate_T(rawT);
    pressure = compensate_P(rawP);
    humidity = compensate_H(rawH);
    return true;
}

const std::string& BME280::name(void) const {
    return m_name;
}

bool BME280::raw_data(int32_t& rawT, int32_t& rawP, int32_t& rawH) {
    if (m_pI2C==NULL) return false;
    uint8_t data[8];
    if (!burst_read(data,8,REG_PRESS_MSB)) {
        rawT = rawP = rawH = 0;
        return false;
    }
    rawP = (int32_t(data[0])<<12)|(int32_t(data[1])<<4)|(int32_t(data[2])>>4);
    rawT = (int32_t(data[3])<<12)|(int32_t(data[4])<<4)|(int32_t(data[5])>>4);
    rawH = (int32_t(data[6])<<8)|int32_t(data[7]);
    return true;
}
        

int16_t BME280::conv_s16(const uint8_t* data, int lo, int hi) const {
    return int16_t(uint16_t(data[lo])|(uint16_t(data[hi])<<8));
}

uint16_t BME280::conv_u16(const uint8_t* data, int lo, int hi) const {
    return uint16_t(data[lo])|(uint16_t(data[hi])<<8);
}

bool BME280::read_calibration(void) {
    if (m_pI2C==NULL) return false;
    return read_calibration_lo() && read_calibration_hi();
}

bool BME280::read_calibration_lo(void) {
    if (m_pI2C==NULL) return false;
    uint8_t data[VAL_CALIB00_25_SIZE];
    if (!burst_read(data,VAL_CALIB00_25_SIZE,REG_CALIB00_25_BASE)) return false;
    m_calib.dig_T1 = conv_u16(data,0,1);    // 0x88:89
    m_calib.dig_T2 = conv_s16(data,2,3);    // 0x8A:8B
    m_calib.dig_T3 = conv_s16(data,4,5);    // 0x8C:8D
    m_calib.dig_P1 = conv_u16(data,6,7);    // 0x8E:8F
    m_calib.dig_P2 = conv_s16(data,8,9);    // 0x90:91
    m_calib.dig_P3 = conv_s16(data,10,11);  // 0x92:93
    m_calib.dig_P4 = conv_s16(data,12,13);  // 0x94:95
    m_calib.dig_P5 = conv_s16(data,14,15);  // 0x96:97
    m_calib.dig_P6 = conv_s16(data,16,17);  // 0x98:99
    m_calib.dig_P7 = conv_s16(data,18,19);  // 0x9A:9B
    m_calib.dig_P8 = conv_s16(data,20,21);  // 0x9C:9D
    m_calib.dig_P9 = conv_s16(data,22,23);  // 0x9E:9F
                                            // 0xA0
    m_calib.dig_H1 = data[25];              // 0xA1
    return true;
}

bool BME280::read_calibration_hi(void) {
    if (m_pI2C==NULL) return false;
    uint8_t data[VAL_CALIB26_41_SIZE];
    if (!burst_read(data,VAL_CALIB26_41_SIZE,REG_CALIB26_41_BASE)) return false;
    m_calib.dig_H2 = conv_s16(data,0,1);                                            // 0xE1:E2
    m_calib.dig_H3 = data[2];                                                       // 0xE3
    m_calib.dig_H4 = int16_t((uint16_t(data[3])<<8)|uint16_t((data[4]&0xF)<<4))>>4; // 0xE4:E5[3:0]
    m_calib.dig_H5 = int16_t((uint16_t(data[5])<<8)|uint16_t((data[4]&0xF0))>>4);   // 0xE5[7:4]:E6
    m_calib.dig_H6 = int8_t(data[6]);                                               // 0xE7
    return true;
}

int32_t BME280::compensate_T(int32_t raw_T) {
    int32_t var1, var2, T;
    var1 = ((((raw_T>>3) - ((int32_t)m_calib.dig_T1<<1))) * ((int32_t)m_calib.dig_T2)) >> 11;    
    var2 = (((((raw_T>>4) - ((int32_t)m_calib.dig_T1)) * ((raw_T>>4) - ((int32_t)m_calib.dig_T1))) >> 12) * ((int32_t)m_calib.dig_T3)) >> 14;
    m_fine_temp = var1 + var2;
    T = (m_fine_temp * 5 + 128) >> 8;
    return T;
}

uint32_t BME280::compensate_P(int32_t raw_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)m_fine_temp) - 128000;
    var2 = var1 * var1 * (int64_t)m_calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)m_calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)m_calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)m_calib.dig_P3)>>8) + ((var1 * (int64_t)m_calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)m_calib.dig_P1)>>33;
    if (var1 == 0) return false; // avoid division by zero
    p = 1048576-raw_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)m_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)m_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)m_calib.dig_P7)<<4);
    return (uint32_t)p;
}

uint32_t BME280::compensate_H(int32_t raw_H) {
    int32_t v_x1_u32r;
    v_x1_u32r = (m_fine_temp - ((int32_t)76800));
    v_x1_u32r = (((((raw_H << 14) - (((int32_t)m_calib.dig_H4) << 20) - (((int32_t)m_calib.dig_H5) * v_x1_u32r)) +
                ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)m_calib.dig_H6)) >> 10) * (((v_x1_u32r * 
                ((int32_t)m_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                ((int32_t)m_calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)m_calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r>>12);
}

// ======================================== LOW LEVEL API ========================================
bool BME280::reset(void) {
    m_bOk = false;
    if (write8(REG_RESET,VAL_RESET)) {
        wait_ms(2);
        m_mode = MODE_SLEEP;
        return true;
    }
    return false;
}

uint8_t BME280::read_chip_id(void) {
    uint8_t result;
    if (read8(REG_ID,result)) return result;
    else return 0x00;
}

uint8_t BME280::read_status(void) {
    uint8_t result;
    if (read8(REG_STATUS,result)) return result&0x9;
    else return STATUS_ERROR;
}

bool BME280::read8(uint8_t reg, uint8_t& val) {
    if (m_pI2C==NULL) return false;
    uint8_t ok = 0x00;
    val = 0x00;
    m_pI2C->start();
        ok = (ok<<1)|m_pI2C->write(m_address);
        ok = (ok<<1)|m_pI2C->write(reg);
        m_pI2C->start();
        ok = (ok<<1)|m_pI2C->write(m_address|1);
        val = m_pI2C->read(0);
    m_pI2C->stop();
    return ok==0x7;
}

bool BME280::write8(uint8_t reg, uint8_t val) {
    if (m_pI2C==NULL) return false;
    uint8_t ok = 0x00;
    m_pI2C->start();
        ok = (ok<<1)|m_pI2C->write(m_address);
        ok = (ok<<1)|m_pI2C->write(reg);
        ok = (ok<<1)|m_pI2C->write(val);
    m_pI2C->stop();
    return ok==0x7;
}

const BME280::calib_t& BME280::calib(void) const {
    return m_calib;
}

bool BME280::burst_read(uint8_t* data, uint8_t nBytes, uint8_t reg) {
    if (m_pI2C==NULL) return false;
    if (nBytes==0) return false;
    if (data==NULL) return false;
    uint8_t ok = 0x00;
    m_pI2C->start();
        ok = (ok<<1)|m_pI2C->write(m_address);
        ok = (ok<<1)|m_pI2C->write(reg);
        m_pI2C->start();
        ok = (ok<<1)|m_pI2C->write(m_address|1);
        for (int i=0; i<nBytes-1; i++) data[i] = m_pI2C->read(1);
        data[nBytes-1] = m_pI2C->read(0);
    m_pI2C->stop();
    if (ok!=0x7) {
        memset(data,0,nBytes);
        return false;
    }
    return true;
}