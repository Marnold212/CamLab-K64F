# CamLab-K64F
Code for performing data acquisition with the FRDM-K64F microcontroller

## Windows Toolchain 

Breif description of tools required for this project  

- **Python 3.9** was the version I used - Can use a Virtual Environment
  - Note that on powershell I needed to enable scripts using command "Set-ExecutionPolicy Unrestricted -Scope Process" to allow the activate script to work 
- **mbed-CLI 1** since mbed-CLI2 doesn't seem to have the same feature set 
- **GNU Arm Embedded Toolchain** version depends on which mbed-CLI version is used can be found on mbed-CLI documentation
  - Note that path must be specified in mbed config 
  - Name slightly for different versions - beware if changing path names 


## Installing mbed-CLI 1

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install mbed command line tools for compiling and flashing mbed device 

```bash
pip install mbed-cli
pip install mbed-greentea
pip install mercurial
pip install mbed-ls
pip install mbed-tools
```

## Basic mbed-CLI Usage 

mbed-CLI runs in command-line such as a vs-code terminal or windows CMD. Look at the documentation for [mbed-CLI 1](https://os.mbed.com/docs/mbed-os/v6.15/build-tools/install-and-set-up.html) for further details. Note that to my knowledge pyOCD does not work on Raspberry Pi architechture so tools such as flashing board or debugging don't work.

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
