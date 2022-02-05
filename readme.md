# G5 Motor Controller Firmware

Code repository for the Goose V Motor Controller Logic Board.

# Installation

```bash
git clone https://github.com/waterloop/MC_STM32.git
cd MC_STM32
git submodule --init --recursive --remote
```

# Build

Start by building the `WLoopCan` submodule and then the MC files

``` bash
cd /path/to/BMW_SW_G5
cd WLoopCAN
make motor_controller
cd ..
make
```
