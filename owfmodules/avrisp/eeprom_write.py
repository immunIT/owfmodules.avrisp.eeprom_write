import struct
import time
import hexformat

from io import BytesIO
from tqdm import tqdm

from octowire_framework.module.AModule import AModule
from octowire.gpio import GPIO
from octowire.spi import SPI
from owfmodules.avrisp.device_id import DeviceID


class EepromWrite(AModule):
    def __init__(self, owf_config):
        super(EepromWrite, self).__init__(owf_config)
        self.meta.update({
            'name': 'AVR write eeprom memory',
            'version': '1.0.0',
            'description': 'Module to write the eeprom memory of an AVR device using the ISP protocol.',
            'author': 'Jordan Ovrè <ghecko78@gmail.com> / Paul Duncan <eresse@dooba.io>'
        })
        self.options = {
            "spi_bus": {"Value": "", "Required": True, "Type": "int",
                        "Description": "The octowire SPI bus (0=SPI0 or 1=SPI1)", "Default": 0},
            "reset_line": {"Value": "", "Required": True, "Type": "int",
                           "Description": "The octowire GPIO used as the Reset line", "Default": 0},
            "spi_baudrate": {"Value": "", "Required": True, "Type": "int",
                             "Description": "set SPI baudrate (1000000 = 1MHz) maximum = 50MHz", "Default": 1000000},
            "firmware": {"Value": "", "Required": True, "Type": "file_r",
                         "Description": "The firmware to write into the eeprom memory.\n"
                                        "Allowed file type: IntelHex or Raw binary.", "Default": ""},
            "erase": {"Value": "", "Required": True, "Type": "bool",
                      "Description": "Erase the eeprom memory before writing", "Default": False},
            "verify": {"Value": "", "Required": False, "Type": "bool",
                       "Description": "Verify the firmware after the write process", "Default": False},
        }
        self.dependencies.append("owfmodules.avrisp.device_id>=1.0.0")

    def get_device_id(self, spi_bus, reset_line, spi_baudrate):
        device_id_module = DeviceID(owf_config=self.config)
        # Set DeviceID module options
        device_id_module.options["spi_bus"]["Value"] = spi_bus
        device_id_module.options["reset_line"]["Value"] = reset_line
        device_id_module.options["spi_baudrate"]["Value"] = spi_baudrate
        device_id_module.owf_serial = self.owf_serial
        device_id = device_id_module.run(return_value=True)
        return device_id

    def erase(self, spi_interface, reset, device):
        write_cmd = b'\xC0'
        enable_mem_access_cmd = b'\xac\x53\x00\x00'

        # Drive reset low
        reset.status = 0

        self.logger.handle("Enable Memory Access...", self.logger.INFO)
        # Drive reset low
        reset.status = 0
        # Enable Memory Access
        spi_interface.transmit(enable_mem_access_cmd)
        time.sleep(0.5)

        # Fill the eeprom with 0xFF
        self.logger.handle("Erasing the eeprom memory (Write 0xFF)...", self.logger.INFO)
        for addr in tqdm(range(0, int(device["eeprom_size"], 16), 1), desc="Erase", ascii=" #", unit_scale=True,
                         bar_format="{desc} : {percentage:3.0f}%[{bar}] {n_fmt}/{total_fmt} bytes "
                                    "[elapsed: {elapsed} left: {remaining}]"):
            spi_interface.transmit(write_cmd + struct.pack(">H", addr) + b'\xFF')
            # Wait until byte write on the eeprom
            if not self.wait_poll_eeprom(spi_interface, 0xFF, addr):
                self.logger.handle("\nWriting at byte address '{}' take too much time, exiting..".format(addr),
                                   self.logger.ERROR)
                return False

        # Drive reset high
        reset.status = 1
        self.logger.handle("Eeprom memory successfully erased.", self.logger.SUCCESS)
        return True

    def verify(self, spi_interface, eeprom_size, firmware):
        read_cmd = b'\xA0'
        dump = BytesIO()

        # Read eeprom loop
        for read_addr in tqdm(range(0, eeprom_size), desc="Read", unit_divisor=1024, ascii=" #", unit_scale=True,
                              bar_format="{desc} : {percentage:3.0f}%[{bar}] {n_fmt}/{total_fmt} Bytes "
                                         "[elapsed: {elapsed} left: {remaining}]"):
            # Read byte
            spi_interface.transmit(read_cmd + struct.pack(">H", read_addr))
            dump.write(spi_interface.receive(1))

        self.logger.handle("Verifying...", self.logger.INFO)
        for index, byte in enumerate(firmware):
            if byte != dump.getvalue()[index]:
                self.logger.handle("verification error, first mismatch at byte 0x{:04x}"
                                   "\n\t\t0x{:04x} != 0x{:04x}".format(index, byte, dump.getvalue()[index]),
                                   self.logger.ERROR)
                break
        else:
            self.logger.handle("{} bytes of eeprom successfully verified".format(len(firmware)), self.logger.SUCCESS)
        dump.close()

    def loading_firmware(self, firmware_file):
        try:
            with open(firmware_file, 'r') as file:
                ihex_firmware = hexformat.intelhex.IntelHex.fromihexfh(file)
                self.logger.handle("IntelHex format detected..", self.logger.INFO)
                firmware = ihex_firmware.get(address=0x00, size=ihex_firmware.usedsize())
        except (UnicodeDecodeError, hexformat.base.DecodeError, ValueError):
            self.logger.handle("Raw binary format detected..", self.logger.INFO)
            with open(firmware_file, 'rb') as file:
                firmware = bytearray(file.read())
        return firmware

    @staticmethod
    def wait_poll_eeprom(spi_interface, byte, byte_addr):
        read_cmd = b'\xA0'

        # 10s timeout
        timeout = time.time() + 10

        while True:
            # Send read cmd
            spi_interface.transmit(read_cmd + struct.pack(">H", byte_addr))
            # Receive the byte and compare it
            read_byte = spi_interface.receive(1)[0]
            if read_byte == byte:
                return True
            if time.time() > timeout:
                return False

    def program_eeprom(self, spi_interface, reset, device, firmware):
        write_cmd = b'\xC0'
        enable_mem_access_cmd = b'\xac\x53\x00\x00'
        verify = self.options["verify"]["Value"]

        # Drive reset low
        reset.status = 0
        # Enable Memory Access
        self.logger.handle("Enable Memory Access...", self.logger.INFO)
        spi_interface.transmit(enable_mem_access_cmd)
        time.sleep(0.5)

        for addr in tqdm(range(0, len(firmware), 1), desc="Program", ascii=" #", unit_scale=True,
                         bar_format="{desc} : {percentage:3.0f}%[{bar}] {n_fmt}/{total_fmt} bytes "
                                    "[elapsed: {elapsed} left: {remaining}]"):
            # Send write cmd
            spi_interface.transmit(write_cmd + struct.pack(">H", addr) + bytes([firmware[addr]]))
            # Wait until byte write on the eeprom
            if not self.wait_poll_eeprom(spi_interface, firmware[addr], addr):
                self.logger.handle("\nWriting at byte address '{}' take too much time, exiting..".format(addr),
                                   self.logger.ERROR)
                return

        self.logger.handle("Successfully write {} byte(s) to eeprom memory.".format(len(firmware)), self.logger.SUCCESS)

        if verify:
            self.logger.handle("Start verifying eeprom memory against {}".format(self.options["firmware"]["Value"]))
            self.verify(spi_interface, int(device["eeprom_size"], 16), firmware)

        # Drive reset high
        reset.status = 1

    def process(self):
        spi_bus = self.options["spi_bus"]["Value"]
        reset_line = self.options["reset_line"]["Value"]
        spi_baudrate = self.options["spi_baudrate"]["Value"]

        device = self.get_device_id(spi_bus, reset_line, spi_baudrate)
        if device is None:
            return

        spi_interface = SPI(serial_instance=self.owf_serial, bus_id=spi_bus)
        reset = GPIO(serial_instance=self.owf_serial, gpio_pin=reset_line)

        # Configure SPI with default phase and polarity
        spi_interface.configure(baudrate=spi_baudrate)
        # Configure GPIO as output
        reset.direction = GPIO.OUTPUT

        # Active Reset is low
        reset.status = 1

        # Erase the target chip
        if not self.erase(spi_interface, reset, device):
            return 

        # Loading firmware
        firmware = self.loading_firmware(self.options["firmware"]["Value"])
        if len(firmware) > int(device["eeprom_size"], 16):
            self.logger.handle("The firmware size is larger than the eeprom size, exiting..", self.logger.ERROR)
            return

        # Program the device eeprom
        self.program_eeprom(spi_interface, reset, device, firmware)

    def run(self):
        """
        Main function.
        Write the eeprom memory of an AVR device.
        :return: Nothing.
        """
        # If detect_octowire is True then Detect and connect to the Octowire hardware. Else, connect to the Octowire
        # using the parameters that were configured. It sets the self.owf_serial variable if the hardware is found.
        self.connect()
        if not self.owf_serial:
            return
        try:
            self.process()
        except ValueError as err:
            self.logger.handle(err, self.logger.ERROR)
        except Exception as err:
            self.logger.handle("{}: {}".format(type(err).__name__, err), self.logger.ERROR)
