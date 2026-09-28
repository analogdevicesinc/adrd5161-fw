# ADRD5161 BMS firmware

Check out https://analogdevicesinc.github.io/system-level/solutions/reference-designs/adrd5161-01z

Implements state machine firmware for battery monitoring system, with CANopen CiA 419 over 500 kbaud CAN.

Operation states:
- Normal State - power delivered to external system
- Charging State - power delivered to ext. system + battery charging
- Shutdown State - power to external system is cut
- Configure State - set charging parameters 

Roadmap:
- [ ] Simpler motor tuning procedure
- [ ] Support RamDebug via SDO block transfer, to enable automated tuning

## Flashing

Use a [MAX32625PICO programmer](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/max32625pico.html) which has been [configured for MAX32690EVKIT](https://github.com/analogdevicesinc/max32625pico-firmware-images) (compatible with the MAX32662).

Software Requirements:

- MaximSDK installed
- ZephyrSDK installed
- Zephyr Workspace configured

# Replace /MaximSDK/ with the path to MSDK
$ west flash --openocd-search /MaximSDK/Tools/OpenOCD/scripts/ --openocd /MaximSDK/Tools/OpenOCD/openocd

## UART Console

By default, the firmware exposes a UART shell. You may use the default Zephyr shell commands (such as `kernel reboot`). Otherwise, the UART console displays the operation state, and battery measurements, such as cell voltage, remaining capacity, state of charge.

## Project structure

Board definition:

- `boards/adi/adrd5161`: board definition for both revA and revB (default). Especially relevant files:
    - `adrd5161_defconfig`: default kconfig enabling all board functionality
    - `adrd5161.dts`: base device tree

Out-of-tree drivers:
- `drivers`: Driver implementation
    - `fuel_gauge/max17320`
    - `charger/max77961`
    - `sensor/max31827`
- `dts/bindings`: Devicetree bindings
    - `fuel_gauge/adi,max17320.yaml`
    - `charger/adi,max77961.yaml`
    - `sensor/adi,max31827.yaml`

Application implementation:
- `app`
    - `src`
        - `button_manager/button_manager.c/h`: Logic for handling on-board push buttons behaviour
        - `drivers/adp5589.c/h`: ADP5589 GPIO expander driver implementation
        - `bms_fw.c` : main state machine implementation
    - `objdict`: CANOpenNode object dictionary, EDS, XDD
    - `prj.conf`: Additional kconfig settings
