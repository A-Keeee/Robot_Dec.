from pyb import UART
import struct
import time  # 用于延时
# 初始化 UART
# 根据您的硬件选择正确的 UART 端口和波特率
# 对于 OpenMV RT，请注释掉 UART(3) 并使用 UART(1)
uart = UART(1, 115200)   # OpenMV RT 注释掉这一行，用下一行 UART(1)
# uart = UART(1, 19200)  # OpenMV RT 用 UART(1)，对应 P4-TX P5-RX
def send_x(x):
    try:
        # 使用 struct 打包三个 int16，采用小端字节序
        packed_data = b'$ARMAIM' + struct.pack('<h', x) + b'!'
        uart.write(packed_data)
        print('发送数据:', packed_data)
    except Exception as e:
        print("发送数据时出错:", e)

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


# def read_data():
#     byte = ''
#     if uart.any():
#         data = uart.read(1)  # 读取一个字节
#         if data == b'$':  # 如果读取到了数据头
#             data = b'$'
#             while True:
#                 if uart.any():
#                     byte = uart.read(1)
#                     data += byte
#                 if byte == b'!':  # 如果读取到了数据尾
#                     return data  # 返回有效数据
#     return None
buffer = bytearray()  # 全局缓冲区，用于存储串口接收的数据
def read_data():
    global buffer
    while uart.any():
        buffer += uart.read(uart.any())  # 读取所有可用数据并追加到缓冲区

    # 检测数据包的起始和结束标志
    start = buffer.find(b'$')
    end = buffer.find(b'!')
    if start != -1 and end != -1 and end > start:  # 找到完整的数据包
        packet = buffer[start:end + 1]  # 提取完整的数据包
        buffer = buffer[end + 1:]  # 丢弃已处理的数据
        return packet  # 返回有效数据包
    elif len(buffer) > 64:  # 防止缓冲区溢出
        buffer = buffer[-64:]  # 仅保留最近的数据
    return None


