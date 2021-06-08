#ifndef _LIS3DH_H
#define _LIS3DH_H

#include "SPI_16bit.h"

#define SPI_PRESCALER 16

/**
 * LIS3DH REGISTERS
 */
#define WHO_AM_I 0x0F
#define CTRL_REG0 0x1E
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define STATUS_REG 0x27
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0X2C
#define OUT_Z_H 0X2D

#define LIS3DH_ID 63

namespace custom_libraries{

struct Raw_values{
    int16_t x_axis;
    int16_t y_axis;
    int16_t z_axis;
};

struct Angle_values{
    int16_t x_axis;
    int16_t y_axis;
    bool x_clockwise;
    bool y_clockwise;
};

class LIS3DH : public _SPI_16{
    private:
        GPIO_TypeDef *CS_PORT;
        uint8_t CS_PIN;
        int16_t X_AXIS_RAW = 0;
        int16_t Y_AXIS_RAW = 0;
        int16_t Z_AXIS_RAW = 0;
        int16_t X_AXIS_ANGLE = 0;
        int16_t Y_AXIS_ANGLE = 0;
        bool x_clockwise = false;
        bool x_anticlockwise = false;
        bool y_clockwise = false;
        bool y_anticlockwise = false;   
    private:
    public:
    public:
        LIS3DH(SPI_TypeDef *SPI,
                GPIO_TypeDef *GPIO,
                uint8_t SCK_PIN,
                uint8_t MOSI_PIN,
                uint8_t MISO_PIN,
                GPIO_TypeDef *CS_PORT,
                uint8_t CS_PIN);
        void set_cs_pin(void);
        void reset_cs_pin(void);
        bool initialize(void);
        Raw_values read_raw_values(void);
        Angle_values read_angles(void);
        ~LIS3DH();

};

}





#endif //_LIS3DH_H