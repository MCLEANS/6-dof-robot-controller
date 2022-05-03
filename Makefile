
# list of source files
#CXX SOURCE_FILES
CXX_SOURCES += main.cpp
CXX_SOURCES += SYS_CLOCK/clockconfig.cpp
CXX_SOURCES += SERVO/PWM.cpp
CXX_SOURCES += SERVO/MG996R.cpp
CXX_SOURCES += LIS3DH/SPI_16bit.cpp 
CXX_SOURCES += LIS3DH/LIS3DH.cpp 
CXX_SOURCES += USART/USART.cpp
CXX_SOURCES += GPIO/GPIO.cpp
CXX_SOURCES += ADC/ADC.cpp
CXX_SOURCES += TIMER/Timerconfiguration.cpp
CXX_SOURCES += EXTI/EXTI.cpp 

CXX_OBJS += main.o
CXX_OBJS += clockconfig.o
CXX_OBJS += PWM.o 
CXX_OBJS += MG996R.o
CXX_OBJS += SPI_16bit.o 
CXX_OBJS += LIS3DH.o 
CXX_OBJS += USART.o
CXX_OBJS += GPIO.o 
CXX_OBJS += ADC.o
CXX_OBJS += Timerconfiguration.o
CXX_OBJS += EXTI.o


#C SOURCE FILES
SOURCES  += ./dependencies/system_stm32f4xx.c
#SOURCES += ./dependencies/stm32f4xx_it.c
SOURCES += ./startup/startup_stm32f40_41xxx.s
#FreeRTOS dependencies
SOURCES += ./FreeRTOS/Source/croutine.c
SOURCES += ./FreeRTOS/Source/event_groups.c
SOURCES += ./FreeRTOS/Source/list.c 
SOURCES += ./FreeRTOS/Source/queue.c 
SOURCES += ./FreeRTOS/Source/stream_buffer.c 
SOURCES += ./FreeRTOS/Source/tasks.c 
SOURCES += ./FreeRTOS/Source/timers.c 
SOURCES += ./FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
SOURCES += ./FreeRTOS/Source/portable/MemMang/heap_4.c

CC_OBJS += system_stm32f4xx.o
#CC_OBJS += stm32f4xx_it.o
CC_OBJS += startup_stm32f40_41xxx.o
CC_OBJS += croutine.o
CC_OBJS += event_groups.o
CC_OBJS += list.o
CC_OBJS += queue.o
CC_OBJS += stream_buffer.o
CC_OBJS += tasks.o
CC_OBJS += timers.o
CC_OBJS += port.o
CC_OBJS += heap_4.o



# name for output binary files
PROJECT = main

# compiler, objcopy (should be in PATH)
CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy

# path to st-flash (or should be specified in PATH)
ST_FLASH = st-flash

# specify compiler flags
CFLAGS  = -g -O2 -Wall -c 
CFLAGS += -T  STM32F417IG_FLASH.ld -specs=nano.specs -specs=nosys.specs
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS +=   -mfpu=fpv4-sp-d16 -mfloat-abi=soft
CFLAGS += -DSTM32F40_41xxx
CFLAGS += -I ./Includes
CFLAGS += -I .
CFLAGS += -I ./FreeRTOS/Source/portable/GCC/ARM_CM4F
CFLAGS += -I ./FreeRTOS/Source/include/
CFLAGS += -I ./EXTI 
CFLAGS += -I ./TIMER
CFLAGS += -I ./LIS3DH
CFLAGS += -I ./GPIO
CFLAGS += -I ./SERVO
CFLAGS += -I ./SYS_CLOCK
CFLAGS += -I ./USART 
CFLAGS += -I ./ADC

LD_FLAGS = -mlittle-endian -mthumb
LD_FLAGS += -DSTM32F40_41xxx -T  STM32F417IG_FLASH.ld
LD_FLAGS +=  -mfpu=fpv4-sp-d16 -specs=nano.specs -specs=nosys.specs
LD_FLAGS += -Wl,--gc-sections


all: $(PROJECT).elf

# compile
$(PROJECT).elf: $(CXX_SOURCES)
	$(CC) $(CFLAGS) $(SOURCES) 
	$(CXX) $(CFLAGS) $(CXX_SOURCES) 
	$(CXX) $(LD_FLAGS) $(CC_OBJS) $(CXX_OBJS) -o $@ 
	$(OBJCOPY) -O ihex $(PROJECT).elf $(PROJECT).hex
	$(OBJCOPY) -O binary $(PROJECT).elf $(PROJECT).bin

# remove binary files
clean:
	rm -f *.o *.elf *.hex *.bin

# flash
flash:
	sudo $(ST_FLASH) write $(PROJECT).bin 0x8000000
