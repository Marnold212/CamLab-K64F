# CamLab-K64F
Code for performing data acquisition with the FRDM-K64F microcontroller

## Installing mbed-CLI 1

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install mbed command line tools for compiling and flashing mbed device 

## Installing mbed-CLI 1

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install mbed command line tools for compiling and flashing mbed device 

```bash
pip install mbed-cli
pip install mbed-greentea
pip install mercurial
pip install mbed-ls
pip install mbed-os-tools
```

## Basic mbed-CLI Usage 

Look at the documentation for [mbed-CLI 1](https://os.mbed.com/docs/mbed-os/v6.15/build-tools/install-and-set-up.html) for further details 

```python
# lists available commands 
mbed 

# set the path for the GCC Toolchain - this path depends on your installation 
mbed config --global GCC_ARM_PATH "C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2020-q4-major\bin"

# returns list of connected devices 
mbed detect

# returns current compilation config 
mbed config --list

# set local toolchain to GCC_ARM (Use -G flag for global config)
mbed -t GCC_ARM

# set local device to either detect or k64f (Use -G flag for global config)
mbed -m detect
mbed -m k64f

# imports example project 
mbed import https://github.com/ARMmbed/mbed-os-example-blinky 

# imports mbed-os library
mbed import mbed-os 

# Compile an mbed project (-t and -m not required if aready set within project)
# -f flag automatically flashes .bin file if there is a suitable device connected 
# --sterm option opens a terminal in the command line with default settings (9600)
mbed compile -t GCC_ARM -m k64f -f --sterm 
```
