#include "SPI_16bit.h"

namespace custom_libraries{

    _SPI_16::_SPI_16(SPI_TypeDef *SPI_,
                GPIO_TypeDef *GPIO,
                uint8_t SCK_PIN,
                uint8_t MOSI_PIN,
                uint8_t MISO_PIN,
                uint8_t SPI_PRESCALER,
                bool CPHA,
                bool CPOL,
                bool LSBFIRST):SPI_(SPI_),
                                SCK_PIN(SCK_PIN),
                                MOSI_PIN(MOSI_PIN),
                                MISO_PIN(MISO_PIN),
                                SPI_PRESCALER(SPI_PRESCALER),
                                CPHA(CPHA),
                                CPOL(CPOL),
                                LSBFIRST(LSBFIRST){

        //ENABLE SPI RCC
        if(SPI_ == SPI1) RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; 
        if(SPI_ == SPI2) RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; 
        if(SPI_ == SPI3) RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;                          

        //ENABLE GPIO RCC
        if(GPIO == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        if(GPIO == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        if(GPIO == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
        if(GPIO == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
        if(GPIO == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
        if(GPIO == GPIOF) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
        if(GPIO == GPIOG) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
        if(GPIO == GPIOH) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
        if(GPIO == GPIOI) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;

        //***************CONFIGURE SCK PIN*********************
        //CONFIGURE PIN TO ALTERNATE FUNCTION PUSH PULL
        GPIO->MODER &= ~(1<<(SCK_PIN*2));
        GPIO->MODER &= ~(1<<((SCK_PIN*2)+1));

        GPIO->MODER &= ~(1<<(SCK_PIN*2));
        GPIO->MODER |= (1<<((SCK_PIN*2)+1));

        //SET SPEED TO MAXIMUM (VERY HIGH SPEED)
        GPIO->OSPEEDR &= ~(1<<(SCK_PIN*2));
        GPIO->OSPEEDR &= ~(1<<((SCK_PIN*2)+1));

        //SET SPEED TO MAXIMUM (VERY HIGH SPEED)
        GPIO->OSPEEDR |= (1<<(SCK_PIN*2));
        GPIO->OSPEEDR |= (1<<((SCK_PIN*2)+1));

        //ENABLE SPECIFIC ALTERNATE FUNCTION
        if(SCK_PIN < 8){
            GPIO->AFR[0] |= (5<<(SCK_PIN*4));
        }
        else{
            GPIO->AFR[1] |= (5<<((SCK_PIN-8)*4));
        }
        
        //******************CONFIGURE MOSI PIN*******************
         //CONFIGURE PIN TO ALTERNATE FUNCTION PUSH PULL
        GPIO->MODER &= ~(1<<(MOSI_PIN*2));
        GPIO->MODER &= ~(1<<((MOSI_PIN*2)+1));

        GPIO->MODER &= ~(1<<(MOSI_PIN*2));
        GPIO->MODER |= (1<<((MOSI_PIN*2)+1));

        //SET SPEED TO MAXIMUM (VERY HIGH SPEED)
        GPIO->OSPEEDR &= ~(1<<(MOSI_PIN*2));
        GPIO->OSPEEDR &= ~(1<<((MOSI_PIN*2)+1));

        //SET SPEED TO MAXIMUM (VERY HIGH SPEED)
        GPIO->OSPEEDR |= (1<<(MOSI_PIN*2));
        GPIO->OSPEEDR |= (1<<((MOSI_PIN*2)+1));

        //ENABLE SPECIFIC ALTERNATE FUNCTION
        if(MOSI_PIN < 8){
            GPIO->AFR[0] |= (5<<(MOSI_PIN*4));
        }
        else{
            GPIO->AFR[1] |= (5<<((MOSI_PIN-8)*4));
        }     

        //********************CONFIGURE MISO PIN********************
        //SET TO INPUT STATE ALTERNATE FUNCTION
        GPIO->MODER &= ~(1<<(MISO_PIN*2));
        GPIO->MODER |= (1<<((MISO_PIN*2)+1));
        //ENABLE SPECIFIC ALTERNATE FUNCTION
        if(MISO_PIN < 8){
            GPIO->AFR[0] |= (5<<(MISO_PIN*4));
        }
        else{
            GPIO->AFR[1] |= (5<<((MISO_PIN-8)*4));
        }   

        //*****************CONFIGURE ACTUAL SPI**********************
        //DISABLE THE SPI FOR CONFIGURATION
        SPI_->CR1 &= ~SPI_CR1_SPE;

        //SET THE SPI CLOCK PRESCALER
        if(this->SPI_PRESCALER == 2) SPI_->CR1 &= ~(SPI_CR1_BR);
        if(this->SPI_PRESCALER == 4) SPI_->CR1 |= SPI_CR1_BR_0;
        if(this->SPI_PRESCALER == 8) SPI_->CR1 |= SPI_CR1_BR_1;
        if(this->SPI_PRESCALER == 16) SPI_->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1;
        if(this->SPI_PRESCALER == 32) SPI_->CR1 |= SPI_CR1_BR_2;
        if(this->SPI_PRESCALER == 64) SPI_->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_0;
        if(this->SPI_PRESCALER == 128) SPI_->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1;
        if(this->SPI_PRESCALER == 256) SPI_->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;

       //SET 16 BIT MODE
       SPI_->CR1 |= SPI_CR1_DFF;

        //SET SPI TO MASTER MODE
        SPI_->CR1 |= SPI_CR1_MSTR;

        //ENABLE SS PIN SOFTWARE MODE
        SPI_->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM;

        //SET THE CLOCK PHASE
        if(this->CPHA){
            SPI_->CR1 |= SPI_CR1_CPHA;
        }
        else{
            SPI_->CR1 &= ~SPI_CR1_CPHA;
        }

        //SET THE CLOCK POLARITY
        if(this->CPOL){
            SPI_->CR1 |= SPI_CR1_CPOL;
        }
        else{
            SPI_->CR1 &= ~SPI_CR1_CPOL;
        }

        //SET WHETHER LSBFIRST OR MSBFIRST
        if(this->LSBFIRST){
            SPI_->CR1 |= SPI_CR1_LSBFIRST;
        }
        else{
            SPI_->CR1 &= ~SPI_CR1_LSBFIRST;
        }

     
        //ENABLE SLAVE SELECT OUTPUT
        SPI_->CR2 |= SPI_CR2_SSOE;

        //ENABLE ACTUAL SPI
        SPI_->CR1 |= SPI_CR1_SPE;

        
     }

     void _SPI_16::write(uint16_t data){
        SPI_->DR = data;
        while(!(SPI_->SR & SPI_SR_TXE)){}
        while(SPI_->SR & SPI_SR_BSY){}
     }

     uint16_t _SPI_16::read(uint16_t junk){
        (void)SPI_->DR;
        SPI_->DR = junk;
        while(!(SPI_->SR & SPI_SR_RXNE)){}
        while(SPI_->SR & SPI_SR_BSY){}

        return SPI_->DR;
    }

    _SPI_16::~_SPI_16(){


    }





}