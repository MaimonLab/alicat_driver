import serial.tools.list_ports
from typing import List

"""list_ports 

Script to list all devices connected to a com port. 
Can be run with:
- python3 list_ports.py 
- ros2 run alicat_driver list_ports
"""


def list_ports_serial() -> List:
    """Returns list of port with manufacturer and serial"""

    port_data = list(serial.tools.list_ports.comports())

    resultPorts = []
    for com_port in port_data:
        port_dict = {
            "port": com_port.device,
            "manufacturer": com_port.manufacturer,
            "serial": com_port.serial_number,
        }
        if com_port.device:
            resultPorts.append(port_dict)

    return resultPorts


def main():
    listed_ports = list_ports_serial()

    print(f"\nPorts found (port, manufacturer, serial)")
    for port_dict in listed_ports:
        print(
            f"  - {port_dict['port']}, {port_dict['manufacturer']}, {port_dict['serial']}"
        )
    print("")


if __name__ == "__main__":
    main()
