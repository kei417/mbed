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
#ifndef __BME280_H__
#define __BME280_H__

#include"mbed.h"
#include<string>

class BME280 {
public:
    /// Oversampling settings for Pressure, Temperature, Humidity
    enum sampling_t {
        SAMPLING_0x         = 0x00,                 ///< skipped, output set to 0x80000
        SAMPLING_1x         = 0x01,                 ///< oversampling x1
        SAMPLING_2x         = 0x02,                 ///< oversampling x2
        SAMPLING_4x         = 0x03,                 ///< oversampling x4
        SAMPLING_8x         = 0x04,                 ///< oversampling x8
        SAMPLING_16x        = 0x05                  ///< oversampling x16
    };
    /// Standby times for auto mode, ignored in forced mode
    enum standby_t {
        STANDBY_0_5         = 0x0,                  ///<    0.5ms standby between autonomous measurements
        STANDBY_62_5        = 0x1,                  ///<   62.5ms standby time
        STANDBY_125         = 0x2,                  ///<  125.0ms standby time
        STANDBY_250         = 0x3,                  ///<  250.0ms standby time
        STANDBY_500         = 0x4,                  ///<  500.0ms standby time
        STANDBY_1000        = 0x5,                  ///< 1000.0ms standby time
        STANDBY_10          = 0x6,                  ///<   10.0ms standby time
        STANDBY_20          = 0x7                   ///<   20.0ms standby time
    };
    /// Filter settings
    enum filter_t {
        FILTER_OFF          = 0x0,                  ///< no IIR filtering, immediately reaches 75% of step response
        FILTER_2            = 0x1,                  ///< 75% of step response reached after 2 samples
        FILTER_4            = 0x2,                  ///< 75% of step response reached after 5 samples
        FILTER_8            = 0x3,                  ///< 75% of step response reached after 11 samples
        FILTER_16           = 0x4                   ///< 75% of step response reached after 22 samples
    };
    /// Sensor mode
    enum mode_t {
        MODE_SLEEP          = 0x00,                 ///< sleep mode, can be achieved by calling reset()
        MODE_FORCED         = 0x01,                 ///< forced mode, conversion only upon request
        MODE_AUTO           = 0x03                  ///< continuous measurement mode
    };
    /// Registers
    static const uint8_t    REG_HUM_LSB;            ///< Humidity, lsb register
    static const uint8_t    REG_HUM_MSB;            ///< Humidity, msb register
    static const uint8_t    REG_TEMP_XLSB;          ///< Temperature, xlsb register (4bit)
    static const uint8_t    REG_TEMP_LSB;           ///< Temperature, lsb register
    static const uint8_t    REG_TEMP_MSB;           ///< Temperature, msb register
    static const uint8_t    REG_PRESS_XLSB;         ///< Pressure, xlsb register (4bit)
    static const uint8_t    REG_PRESS_LSB;          ///< Pressure, lsb register
    static const uint8_t    REG_PRESS_MSB;          ///< Pressure, msb register
    static const uint8_t    REG_CONFIG;             ///< Config register
    static const uint8_t    REG_CTRL_MEAS;          ///< Control Measurement register
    static const uint8_t    REG_STATUS;             ///< Status register
    static const uint8_t    REG_CTRL_HUM;           ///< Control Humidity register
    static const uint8_t    REG_CALIB26_41_BASE;    ///< Base of higher calibration data registers
    static const uint8_t    REG_RESET;              ///< Soft Reset register
    static const uint8_t    REG_ID;                 ///< Chip ID register
    static const uint8_t    REG_CALIB00_25_BASE;    ///< Base of lower calibration data registers
    /// Values
    static const uint8_t    VAL_CALIB26_41_SIZE;    ///< Amount of higher calibration data
    static const uint8_t    VAL_CALIB00_25_SIZE;    ///< Amount of lower calibration data
    static const uint8_t    VAL_CHIP_ID;            ///< Chip ID
    static const uint8_t    VAL_RESET;              ///< Reset command
    static const uint8_t    STATUS_IDLE;            ///< Status value for Idle
    static const uint8_t    STATUS_MEASURING;       ///< Status value for Measurment in progress
    static const uint8_t    STATUS_UPDATING;        ///< Status value for Updating calibration registers from NVM
    static const uint8_t    STATUS_ERROR;           ///< Status value for Error
    
protected:

    /// Calibration data structure
    struct calib_t {
        uint16_t    dig_T1;     
        int16_t     dig_T2;
        int16_t     dig_T3;
        uint16_t    dig_P1;
        int16_t     dig_P2;
        int16_t     dig_P3;
        int16_t     dig_P4;
        int16_t     dig_P5;
        int16_t     dig_P6;
        int16_t     dig_P7;
        int16_t     dig_P8;
        int16_t     dig_P9;
        uint8_t     dig_H1;
        int16_t     dig_H2;
        uint8_t     dig_H3;
        int16_t     dig_H4;
        int16_t     dig_H5;
        int8_t      dig_H6;
    } m_calib;

public:

                        BME280(void);                               ///< Default constructor
                        ~BME280(void);                              ///< Destructor
    bool                init(I2C& i2c, uint8_t address=0x76);       ///< Initialize sensor and read out calibration
    bool                start(  sampling_t hum = SAMPLING_8x,       ///< Start measurement with provided configuration
                                sampling_t temp = SAMPLING_8x, 
                                sampling_t press = SAMPLING_8x,
                                standby_t standby = STANDBY_500,
                                filter_t filter = FILTER_8,
                                mode_t mode = MODE_AUTO);
    bool                stop(void);                                 ///< Stop measurement and put sensor in sleep mode
    bool                done(void);                                 ///< "Detach" sensor from I2C bus and put in sleep mode
    bool                get(float& T, float& P, float& H);          ///< read float data: [T]=1C    [P]=1 hPa    [H]=%
    bool                get(int32_t& T,uint32_t& P,uint32_t& H);    ///< read int data:   [T]=0.01C [P]=1/256 Pa [H]=1/1024 %
    const bool&         is_ok(void) const;                          ///< true iff sensor initialized properly
    const std::string&  name(void) const;                           ///< returns sensor name
    
    /// LOW LEVEL API
    uint8_t             read_chip_id(void);                         ///< returns the chip id on success, 0x00 on failure
    bool                reset(void);                                ///< reset the sensor and put to sleep mode
    uint8_t             read_status(void);                          ///< read status, returns 0xFF on failure
    bool                write8(uint8_t reg,uint8_t val);            ///< write 8bit register, returns true on success
    bool                read8(uint8_t reg,uint8_t& val);            ///< read 8bit register, returns true on success
    const calib_t&      calib(void) const;                          ///< access to calibration data
    bool                burst_read(uint8_t* data, uint8_t nBytes,
                                   uint8_t reg);                    ///< burst read of data
    
    I2C*                m_pI2C;                                     ///< pointer to I2C bus
    uint8_t             m_address;                                  ///< slave address (with write flag)
    int32_t             m_fine_temp;                                ///< last temperature for thermo-compensation
    mode_t              m_mode;                                     ///< sensor mode
    bool                m_bOk;                                      ///< sensor initialized ok?
    
    bool                read_calibration(void);                     ///< read lower and higher part of calibration data
    bool                read_calibration_lo(void);                  ///< read lower part of calibration data
    bool                read_calibration_hi(void);                  ///< read higher part of calibration data
    int16_t             conv_s16(const uint8_t* data,               ///< convert 2 bytes of calibration data to int16
                                 int lo, int hi) const;
    uint16_t            conv_u16(const uint8_t* data,               ///< convert 2 bytes of calibration data to uint16
                                 int lo, int hi) const;
    bool                raw_data(int32_t& T,int32_t& P,int32_t& H); ///< read the raw sensor data
    
    /// converts a raw temperature measurement into degree C, resolution 0.01C
    /// updates m_fine_temp for later pressure and humidity compensation
    int32_t             compensate_T(int32_t raw_T);
    
    /// converts a raw pressure measurement into PA, resolution 1/256 Pa / 1/256000 hPa
    /// uses m_fine_temp for correction
    uint32_t            compensate_P(int32_t raw_P);
    
    // converts a raw humidity measurement into %rel.H., resolution 1/1024 % rh
    // uses m_fine_temp for correction
    uint32_t            compensate_H(int32_t raw_H);
    static const std::string m_name;
private:
    // No copy constructor for you!
    BME280(const BME280& other);
};

#endif
