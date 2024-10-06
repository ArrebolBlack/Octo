import serial
import time
# import numpy as np


class GripperController:
    def __init__(self, com_port='/dev/ttyUSB0', rate=115200) -> None:
        self.send_data = None
        self.last_send_value = None
        self.com_port = com_port
        self.rate = rate
        self.ser = serial.Serial(com_port, rate, timeout=1)  # 使用适当的串口和波特率
        self.init_gripper()
        
    # crc16校验位生成函数
    def _crc16_bytes(self, data: bytes, polynomial=0xA001, start_value=0xFFFF) -> bytes:
        """
        Compute the CRC-16 (Modbus variant) of the given bytes and return as bytes.
        """
        crc = start_value
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ polynomial
                else:
                    crc >>= 1
        return crc.to_bytes(2, 'little')  # Modbus uses little-endian for CRC


    # 机械爪初始化
    def init_gripper(self) -> bytes:
        # 发送初始化指令
        # 指令前缀
        addr = bytes.fromhex('01')
        func = bytes.fromhex('06')
        reg = bytes.fromhex('01 00')
        data = bytes.fromhex('00 01')

        raw_data = addr + func + reg + data

        # 计算CRC
        crc_value = self._crc16_bytes(raw_data)

        # 发送数据
        send_data = raw_data + crc_value

        self.ser.write(send_data)
        # received_data = bytes(self.ser.read(8))
        # print(received_data.hex())

        self.send_data = send_data
        self.last_send_value = 1000
        return send_data


    # 机械爪位置设置函数，范围：0-1000
    def set_gripper_position(self, value: int) -> bytes:
        # if not (0 <= value <= 1000):
        #     print("Value out of range")
        #     return -1
        if value < 0:
            value = 0
        elif value > 1000:
            value = 1000
        # print(value)

        # 指令前缀
        addr = bytes.fromhex('01')
        func = bytes.fromhex('06')
        reg = bytes.fromhex('01 03')

        # 将整数值转换为2字节的字节串
        raw_data = addr + func + reg + value.to_bytes(2, 'big')

        # 计算CRC
        crc_value = self._crc16_bytes(raw_data)

        # 发送数据
        send_data = raw_data + crc_value

        self.ser.write(send_data)
        
        self.send_data = send_data
        self.last_send_value = value
        return send_data


if __name__ == "__main__":
    # 打开串口
    com_port = '/dev/ttyUSB0'
    rate = 115200

    gripper_controller = GripperController(com_port, rate)

    send_data = gripper_controller.send_data
    print(f"init: {gripper_controller.send_data.hex()}")

    time.sleep(3)

    set_position = 10  # 0-1000之间
    res_set = gripper_controller.set_gripper_position(set_position)

    print(f"res_set: {res_set.hex()}")
