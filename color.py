from pyb import UART
import struct
import time  # 用于延时
# 初始化 UART
# 根据您的硬件选择正确的 UART 端口和波特率
# 对于 OpenMV RT，请注释掉 UART(3) 并使用 UART(1)
uart = UART(1, 9600)   # OpenMV RT 注释掉这一行，用下一行 UART(1)
# uart = UART(1, 19200)  # OpenMV RT 用 UART(1)，对应 P4-TX P5-RX

def send_speed(x_speed, y_speed, z_speed):
    """
    通过 UART 发送速度数据。

    数据格式:
    $GOLINE + x_speed (int16) + y_speed (int16) + z_speed (int16) + !

    参数:
    x_speed (int): X 轴速度，范围 -32768 到 32767
    y_speed (int): Y 轴速度，范围 -32768 到 32767
    z_speed (int): Z 轴速度，范围 -32768 到 32767
    """
    # 确保速度值在 int16 范围内
    x_speed = int(max(-32768, min(32767, x_speed)))
    y_speed = int(max(-32768, min(32767, y_speed)))
    z_speed = int(max(-32768, min(32767, z_speed)))
    uart.write("Hello World!\r")
#    try:
#        # 使用 struct 打包三个 int16，采用小端字节序
#        packed_data = b'$GOLINE' + struct.pack('<hhh', x_speed, y_speed, z_speed) + b'!'
#        uart.write(packed_data)
##        print('发送数据:', packed_data)
#    except Exception as e:
#        print("发送数据时出错:", e)

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



# 定义变量
vlf, vrf, vlb, vrb = 0, 0, 0, 0

def scale_down_if_greater_than_1000(a, b, c, d):
    """
    如果四个数的最大绝对值大于1000，则进行缩放。
    """
    max_val = max(abs(a), abs(b), abs(c), abs(d))

    if max_val > 1000:
        scale = 1000.0 / max_val
        a = int(a * scale)
        b = int(b * scale)
        c = int(c * scale)
        d = int(d * scale)

    return a, b, c, d

def chassis_control(vx, vy, wz):
    global vlf, vrf, vlb, vrb

    # 计算四个电机的速度
    vlf = vx + vy - wz
    vrf = -(vx - vy + wz)
    vlb = vx - vy - wz
    vrb = -(vx + vy + wz)

    # 缩放电机速度
    vlf, vrf, vlb, vrb = scale_down_if_greater_than_1000(vlf, vrf, vlb, vrb)

    # 发送控制命令
    send_motor_command(6, vlf)
    send_motor_command(7, vrf)
    send_motor_command(8, vlb)
    send_motor_command(9, vrb)

    time.sleep(0.002)  # 延时，防止指令过于密集

def send_motor_command(motor_id, speed):
    """
    发送电机控制指令。
    """
    # 确保速度是整数
    speed = int(speed)

    # 创建指令
    command = "#{:03d}P{:04d}T{:04d}!".format(motor_id, 1500 + speed, 0)
    # 发送指令
    uart.write(command.encode('utf-8'))
#    print("发送指令:", command)

## 测试代码
#if __name__ == "__main__":
#    # 设置速度
#    vx, vy, wz = 300, 400, 100
#    chassis_control(vx, vy, wz)
