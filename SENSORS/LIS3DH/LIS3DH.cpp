#include "LIS3DH.h"

namespace custom_libraries{

LIS3DH::LIS3DH(SPI_TypeDef *SPI,
                GPIO_TypeDef *GPIO,
                uint8_t SCK_PIN,
                uint8_t MOSI_PIN,
                uint8_t MISO_PIN,
                GPIO_TypeDef *CS_PORT,
                uint8_t CS_PIN): _SPI_16(SPI,
                                            GPIO,
                                            SCK_PIN,
                                            MOSI_PIN,
                                            MISO_PIN,
                                            SPI_PRESCALER,
                                            true,
                                            true,
                                            false),
                                            CS_PORT(CS_PORT),
                                            CS_PIN(CS_PIN){
    //SET RESET, CHIP SELECT AND DC PIN DIRECTION (OUTPUT)                 
    if(CS_PORT == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    if(CS_PORT == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    if(CS_PORT == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    if(CS_PORT == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    if(CS_PORT == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    if(CS_PORT == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    if(CS_PORT == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    if(CS_PORT == GPIOH) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    if(CS_PORT == GPIOI) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;

    //CONFIGURE AS GENERAL PURPOSE OUTPUT 
    this->CS_PORT->MODER |= (1 << (this->CS_PIN*2));
    this->CS_PORT->MODER &= ~(1 << ((this->CS_PIN*2)+1));

    /**
   * Configure Control register 0
   * 1. Initialize the register as indicated in datasheet (This is needed for the sensor to function optimally)
   */
    reset_cs_pin();
    write((CTRL_REG0 << 8)| 0x10);
    set_cs_pin();

    /**
     * configure control register 1
     * 1. Set Normal mode. 
     * 2. Set 100Hz sampling rate
     */
    reset_cs_pin();
    write((CTRL_REG1 << 8)| 0x57);
    set_cs_pin();

    /**
     * Configure Control Register 4
     * 1. Clear HR bit to set Normal Mode
     * 2. Spi Interface mode selection (Set to 4-wire).
     * 3. Full scale selected to +-2g
     */
    reset_cs_pin();
    write((CTRL_REG4 << 8)| 0x00);
    set_cs_pin();

}

void LIS3DH::set_cs_pin(){
  this->CS_PORT->ODR |= (1 << this->CS_PIN);
}

void LIS3DH::reset_cs_pin(){
  this->CS_PORT->ODR &= ~(1 << this->CS_PIN);
}

bool LIS3DH::initialize(){
    //Read from the WHO_AM_I register
    reset_cs_pin();
    uint16_t whoami = read(((0x80 | WHO_AM_I) << 8));
    set_cs_pin();
    whoami &=  ~(0xFF << 8);
    if(whoami == LIS3DH_ID) return true;

    return false;
}

Raw_values LIS3DH::read_raw_values(void){
    Raw_values raw_values;
    /**
   * Read the sensor Status Register
   */
    reset_cs_pin();
    uint16_t status_register = read(((0x80 | STATUS_REG) << 8));
    set_cs_pin();
    status_register &=  ~(0xFF << 8);
    /**
     * Check to confirm new data is available from the sensor
     */
    if(status_register & (1 <<  3)){
        /**
        *  read Raw values for the Y-AXIS
        */
        reset_cs_pin();
        Y_AXIS_RAW = read(((0x80 | OUT_Y_L) << 8)); //Reads the low order bits
        set_cs_pin();
        Y_AXIS_RAW &=  ~(0xFF << 8);

        reset_cs_pin();
        uint16_t Y_AXIS_H = read(((0x80 | OUT_Y_H) << 8)); //Reads the high order bits
        set_cs_pin();
        uint16_t temp_Y_AXIS_H = Y_AXIS_H;
        Y_AXIS_H &=  ~(0xFF80);

        Y_AXIS_RAW |= (Y_AXIS_H << 8);
        if(temp_Y_AXIS_H & (1 << 7)){
        Y_AXIS_RAW = 0-Y_AXIS_RAW;
        }

        /**
         * read Raw data from the X-AXIS
         **/
        reset_cs_pin();
        X_AXIS_RAW = read(((0x80 | OUT_X_L) << 8)); //Reads the low order bits
        set_cs_pin();
        X_AXIS_RAW &=  ~(0xFF << 8);

        reset_cs_pin();
        uint16_t X_AXIS_H = read(((0x80 | OUT_X_H) << 8)); //Reads high order bits
        set_cs_pin();
        uint16_t temp_X_AXIS_H = X_AXIS_H;
        X_AXIS_H &=  ~(0xFF80);

        X_AXIS_RAW |= (X_AXIS_H << 8);
        //Determines the sign of the value
        if(temp_X_AXIS_H & (1 << 7)){ 
        X_AXIS_RAW = 0-X_AXIS_RAW;
        }

        /**
         * read Raw data from the Z-AXIS
         **/
        reset_cs_pin();
        Z_AXIS_RAW = read(((0x80 | OUT_Z_L) << 8)); //Reads low order bits
        set_cs_pin();
        Z_AXIS_RAW &=  ~(0xFF << 8);

        reset_cs_pin();
        uint16_t Z_AXIS_H = read(((0x80 | OUT_Z_H) << 8)); //Reads high order bits
        set_cs_pin();
        uint16_t temp_Z_AXIS_H = Z_AXIS_H;
        Z_AXIS_H &=  ~(0xFF80);

        Z_AXIS_RAW |= (Z_AXIS_H << 8);
        if(temp_Z_AXIS_H & (1 << 7)){
        Z_AXIS_RAW = 0-Z_AXIS_RAW;
        }
    }

    raw_values.x_axis = X_AXIS_RAW;
    raw_values.y_axis = Y_AXIS_RAW;
    raw_values.z_axis = Z_AXIS_RAW;
    
    return raw_values;
}

Angle_values LIS3DH::read_angles(void){
    Angle_values angle_values;

    Raw_values raw_values;
    raw_values = read_raw_values();

    Y_AXIS_ANGLE  = (raw_values.y_axis*90)/17195;
    X_AXIS_ANGLE = (raw_values.x_axis*90)/17195;

    if(X_AXIS_ANGLE > 0){
        x_clockwise = true;
        x_anticlockwise = false;
    } 
    if(X_AXIS_ANGLE < 0){
        X_AXIS_ANGLE = (180 + X_AXIS_ANGLE);
        x_anticlockwise = true;
        x_clockwise = false;
    }

    if(Y_AXIS_ANGLE > 0){
        y_clockwise = true;
        y_anticlockwise = false;
    } 
    if(Y_AXIS_ANGLE < 0){
        Y_AXIS_ANGLE = (180 + Y_AXIS_ANGLE);
        y_anticlockwise = true;
        y_clockwise = false;
    }

    angle_values.x_axis = X_AXIS_ANGLE;
    angle_values.y_axis = Y_AXIS_ANGLE;
    if(x_clockwise) angle_values.x_clockwise = true;
    if(x_anticlockwise) angle_values.x_clockwise = false;
    if(y_clockwise) angle_values.y_clockwise = true;
    if(y_anticlockwise) angle_values.y_clockwise = false;

    return angle_values;
}

LIS3DH::~LIS3DH(){

}

}
                                                