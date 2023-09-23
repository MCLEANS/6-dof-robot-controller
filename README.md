# stm32f4-build-FreeRTOS
The build requires the following:
- [GNU Tools for ARM Embedded Processors toolchain](https://launchpad.net/gcc-arm-embedded) (compiler, objcopy)
- [STLINK software](https://github.com/texane/stlink) (uploading code to stm32f4)
- [STM32 Standard Peripheral Libraries](http://www.st.com/content/st_com/en/products/embedded-software/mcus-embedded-software/stm32-embedded-software/stm32-standard-peripheral-libraries/stsw-stm32054.html)


## Usage
`arm-none-eabi-gcc` and `arm-none-eabi-objcopy` should be in `${PATH}`, `${ST_FLASH}` contains a path to `st-flash`.

## Build

> generate build files
```
cmake -S . -B cmake_build/release
```

> build
```
cmake --build cmake_build/release
```
> upload the code to stm32f4 (requires root privileges)
```
st-flash write cmake_build/release/6dofRobotController.bin 0x8000000
```

