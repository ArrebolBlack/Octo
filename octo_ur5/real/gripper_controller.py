import os
import serial
import time


class GripperController:
    """Serial Modbus RTU controller for a servo gripper (range 0-1000)."""

    def __init__(self, com_port=None, rate=115200) -> None:
        com_port = com_port or os.environ.get("UR5_GRIPPER_PORT", "/dev/ttyUSB0")
        self.send_data = None
        self.last_send_value = None
        self.ser = serial.Serial(com_port, rate, timeout=1)
        self.init_gripper()

    def _crc16_bytes(self, data: bytes, polynomial=0xA001, start_value=0xFFFF) -> bytes:
        crc = start_value
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ polynomial
                else:
                    crc >>= 1
        return crc.to_bytes(2, 'little')

    def init_gripper(self) -> bytes:
        addr = bytes.fromhex('01')
        func = bytes.fromhex('06')
        reg = bytes.fromhex('01 00')
        data = bytes.fromhex('00 01')
        raw_data = addr + func + reg + data
        crc_value = self._crc16_bytes(raw_data)
        send_data = raw_data + crc_value
        self.ser.write(send_data)
        self.send_data = send_data
        self.last_send_value = 1000
        return send_data

    def set_gripper_position(self, value: int) -> bytes:
        value = max(0, min(value, 1000))
        addr = bytes.fromhex('01')
        func = bytes.fromhex('06')
        reg = bytes.fromhex('01 03')
        raw_data = addr + func + reg + value.to_bytes(2, 'big')
        crc_value = self._crc16_bytes(raw_data)
        send_data = raw_data + crc_value
        self.ser.write(send_data)
        self.send_data = send_data
        self.last_send_value = value
        return send_data


if __name__ == "__main__":
    com_port = os.environ.get("UR5_GRIPPER_PORT", "/dev/ttyUSB0")
    gc = GripperController(com_port)
    print(f"init: {gc.send_data.hex()}")
    time.sleep(3)
    res = gc.set_gripper_position(10)
    print(f"set: {res.hex()}")
