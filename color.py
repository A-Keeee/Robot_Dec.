from pyb import UART
import struct
import time  # 用于延时
# 初始化 UART
# 根据您的硬件选择正确的 UART 端口和波特率
# 对于 OpenMV RT，请注释掉 UART(3) 并使用 UART(1)
uart = UART(1, 9600)   # OpenMV RT 注释掉这一行，用下一行 UART(1)
# uart = UART(1, 19200)  # OpenMV RT 用 UART(1)，对应 P4-TX P5-RX

def send_color(num):
    try:
        if num == 1:
            # 使用 struct 打包三个 int16，采用小端字节序
            packed_data = b'$RED!'
            uart.write(packed_data)
            print('发送数据:', packed_data)
        elif num == 2:
            # 使用 struct 打包三个 int16，采用小端字节序
            packed_data = b'$GREEN!'
            uart.write(packed_data)
            print('发送数据:', packed_data)
        elif num == 4:
            # 使用 struct 打包三个 int16，采用小端字节序
            packed_data = b'$BLUE!'
            uart.write(packed_data)
            print('发送数据:', packed_data)
    except Exception as e:
        print("发送数据时出错:", e)

def send_arm(get_drop,direction):
    try:
        if get_drop == 1:
            # 使用 struct 打包三个 int16，采用小端字节序
            packed_data = b'$ARMGET' + struct.pack('<h', direction) + b'!'
            uart.write(packed_data)
            print('发送数据:', packed_data)
        elif get_drop == -1:
            # 使用 struct 打包三个 int16，采用小端字节序
            packed_data = b'$ARMDROP' + struct.pack('<h', direction) + b'!'
            uart.write(packed_data)
            print('发送数据:', packed_data)
    except Exception as e:
        print("发送数据时出错:", e)

def send_draw(color_num):
    try:
        if color_num == 1:
            packed_data = b'$ARMDRAWBLUE!'
            uart.write(packed_data)
            print('发送数据:', packed_data)
        elif color_num == 2:
            packed_data = b'$ARMDRAWGREEN!'            
            uart.write(packed_data)
            print('发送数据:', packed_data)
        elif color_num == 3:
        # 使用 struct 打包三个 int16，采用小端字节序
            packed_data = b'$ARMDRAWRED!'            
            uart.write(packed_data)
            print('发送数据:', packed_data)
    except Exception as e:
        print("发送数据时出错:", e)


def read_data():
    byte = ''
    if uart.any():
        data = uart.read(1)  # 读取一个字节
        if data == b'$':  # 如果读取到了数据头
            data = b'$'
            while True:
                if uart.any():
                    byte = uart.read(1)
                    data += byte
                if byte == b'!':  # 如果读取到了数据尾
                    return data  # 返回有效数据
    return None


