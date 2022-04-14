# G5 Motor Controller Firmware

Code repository for the Goose V Motor Controller Logic Board.

# Installation

```bash
git clone https://github.com/waterloop/MC_STM32.git
cd MC_STM32
git submodule update --init --recursive
```

# Build

Start by building the `WLoopCan` and `WLoopUtil` submodules and then the MC files

``` bash
cd /path/to/MC_STM32
cd WLoopCAN
make motor_controller
cd WLoopUtil
make motor_controller
cd ..
make
```
