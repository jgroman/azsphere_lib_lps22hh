# Azure Sphere custom wrapper for LPS22HH sensor driver
This wrapper allows use of [STM system independent LPS22HH driver](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lps22hh_STdC) on Azure Sphere devices and kits. Currently the wrapper supports only Avnet Azure Sphere Starter Kit with onboard LPS22HH sensor connected via LSM6DSO Sensor Hub.

## Importing
1. Clone this repository to a local directory.
2. Add VS Project lib_lps22hh.vcxproj to your Solution.
3. Add Reference lib_lps22hh to your Project References.
4. Include lib_lps22hh.h to your source files.

## Usage
Refer to included example project lib_lps22hh_example for library usage demonstration.
