# -*- coding: utf-8 -*-

# Octowire Framework
# Copyright (c) ImmunIT - Jordan Ovrè / Paul Duncan
# License: Apache 2.0
# Paul Duncan / Eresse <pduncan@immunit.ch>
# Jordan Ovrè / Ghecko <jovre@immunit.ch>

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
            'name': 'AVR EEPROM memory write',
            'version': '1.0.1',
            'description': 'Write the EEPROM memory of AVR microcontrollers',
            'author': 'Jordan Ovrè / Ghecko <jovre@immunit.ch>, Paul Duncan / Eresse <pduncan@immunit.ch>'
        })
        self.options = {
            "spi_bus": {"Value": "", "Required": True, "Type": "int",
                        "Description": "SPI bus (0=SPI0 or 1=SPI1)", "Default": 0},
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
        self.advanced_options.update({
            "start_address": {"Value": "0x00", "Required": False, "Type": "hex",
                              "Description": "For raw binary only. The starting address\nto write the firmware. "
                                             "Hex format (0x1FC00).",
                              "Default": 0},
        })
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
        self.logger.handle("Enabling Memory Access...", self.logger.INFO)

        # Drive reset low
        reset.status = 0
        # Enable Memory Access
        spi_interface.transmit(enable_mem_access_cmd)
        time.sleep(0.5)

        # Fill the eeprom with 0xFF
        self.logger.handle("Erasing the EEPROM memory (Write 0xFF)...", self.logger.INFO)
        for addr in tqdm(range(0, int(device["eeprom_size"], 16), 1), desc="Erasing", ascii=" #", unit_scale=True,
                         bar_format="{desc} : {percentage:3.0f}%[{bar}] {n_fmt}/{total_fmt} bytes "
                                    "[elapsed: {elapsed} left: {remaining}]"):
            spi_interface.transmit(write_cmd + struct.pack(">H", addr) + b'\xFF')
            # Wait until byte write on the eeprom
            if not self.wait_poll_eeprom(spi_interface, 0xFF, addr):
                self.logger.handle("\nWriting at byte address '{}' took too long, exiting..".format(addr),
                                   self.logger.ERROR)
                return False

        # Drive reset high
        reset.status = 1
        self.logger.handle("EEPROM memory successfully erased.", self.logger.SUCCESS)
        return True

    def verify(self, spi_interface, chunk_size, start_address, chunk):
        read_cmd = b'\xA0'
        dump = BytesIO()
        address = start_address

        # Read eeprom loop
        for _ in tqdm(range(0, chunk_size), desc="Reading", unit_divisor=1024, ascii=" #", unit_scale=True,
                      bar_format="{desc} : {percentage:3.0f}%[{bar}] {n_fmt}/{total_fmt} Bytes "
                                 "[elapsed: {elapsed} left: {remaining}]"):
            # Read byte
            spi_interface.transmit(read_cmd + struct.pack(">H", address))
            dump.write(spi_interface.receive(1))
            address = address + 1

        self.logger.handle("Verifying...", self.logger.INFO)
        for index, byte in enumerate(chunk):
            if byte != dump.getvalue()[index]:
                self.logger.handle("verification error, first mismatch at byte 0x{:04x}"
                                   "\n\t\t0x{:04x} != 0x{:04x}".format(index, byte, dump.getvalue()[index]),
                                   self.logger.ERROR)
                dump.close()
                return False
        else:
            self.logger.handle("{} bytes of EEPROM successfully verified".format(len(chunk)), self.logger.SUCCESS)
            dump.close()
            return True

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

    def program_eeprom(self, spi_interface, reset, chunk, address, chunk_nb=None, chunks=None):
        write_cmd = b'\xC0'
        enable_mem_access_cmd = b'\xac\x53\x00\x00'
        verify = self.options["verify"]["Value"]
        start_address = address
        hex_address = "0x{:04x}".format(start_address)

        # Drive reset low
        reset.status = 0
        # Enable Memory Access
        self.logger.handle("Enabling Memory Access...", self.logger.INFO)
        spi_interface.transmit(enable_mem_access_cmd)
        time.sleep(0.5)

        if chunk_nb is not None and chunks is not None:
            self.logger.handle(f"Writing chunk {chunk_nb}/{chunks} (start address: {hex_address})", self.logger.INFO)
        else:
            self.logger.handle(f"Writing firmware (start address: {hex_address})", self.logger.INFO)

        for index in tqdm(range(0, len(chunk), 1), desc="Programming", ascii=" #", unit_scale=True,
                          bar_format="{desc} : {percentage:3.0f}%[{bar}] {n_fmt}/{total_fmt} bytes "
                                     "[elapsed: {elapsed} left: {remaining}]"):
            # Send write cmd
            spi_interface.transmit(write_cmd + struct.pack(">H", address) + bytes([chunk[index]]))
            # Wait until byte write on the eeprom
            if not self.wait_poll_eeprom(spi_interface, chunk[index], address):
                self.logger.handle("\nWriting at byte address '{}' took too long, exiting..".format(address),
                                   self.logger.ERROR)
                return
            # Increment the Eeprom address
            address = address + 1

        self.logger.handle("Successfully wrote {} byte(s) to EEPROM memory.".format(len(chunk)), self.logger.SUCCESS)

        if verify:
            hex_address = "0x{:04x}".format(start_address)
            if chunk_nb is not None and chunks is not None:
                self.logger.handle(f"Starting chunk verification {chunk_nb}/{chunks} (start address: {hex_address})")
            else:
                self.logger.handle(f"Starting EEPROM memory verification against {self.options['firmware']['Value']} "
                                   f"(start address: {hex_address})")
            self.verify(spi_interface, len(chunk), start_address, chunk)

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

        # Reset is active-low
        reset.status = 1

        # Erase the target chip
        if self.options["erase"]["Value"]:
            if not self.erase(spi_interface, reset, device):
                return

        # Load the firmware and call the write function with the needed arguments,
        # depending the firmware format (raw or ihex)
        try:
            with open(self.options["firmware"]["Value"], 'r') as file:
                ihex_firmware = hexformat.intelhex.IntelHex.fromihexfh(file)
                self.logger.handle("IntelHex format detected..", self.logger.INFO)
                # Program the device
                chunks = len(ihex_firmware.parts())
                chunk_nb = 1
                # For every part in the ihex file, write it to the correct address in the flash memory.
                for ihex_part in ihex_firmware.parts():
                    chunk_addr, chunk_len = ihex_part
                    chunk = ihex_firmware.get(address=chunk_addr, size=chunk_len)
                    self.program_eeprom(spi_interface, reset, chunk, chunk_addr, chunk_nb, chunks)
                    chunk_nb = chunk_nb + 1
        except (UnicodeDecodeError, hexformat.base.DecodeError, ValueError):
            self.logger.handle("Raw binary format detected..", self.logger.INFO)
            with open(self.options["firmware"]["Value"], 'rb') as file:
                firmware = bytearray(file.read())
                self.program_eeprom(spi_interface, reset, firmware, self.advanced_options["start_address"]["Value"])

    def run(self):
        """
        Main function.
        Write the EEPROM memory of AVR microcontrollers.
        :return: Nothing.
        """
        # If detect_octowire is True then detect and connect to the Octowire hardware. Else, connect to the Octowire
        # using the parameters that were configured. This sets the self.owf_serial variable if the hardware is found.
        self.connect()
        if not self.owf_serial:
            return
        try:
            self.process()
        except ValueError as err:
            self.logger.handle(err, self.logger.ERROR)
        except Exception as err:
            self.logger.handle("{}: {}".format(type(err).__name__, err), self.logger.ERROR)
