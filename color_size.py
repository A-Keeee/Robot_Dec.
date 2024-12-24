# 导入所需的库函数
import sensor, image, time
from pyb import Pin

# 初始化 P0 引脚，设置为推挽输出并启用下拉电阻
pin0 = Pin('P0', Pin.OUT_PP, Pin.PULL_DOWN)

# 定义颜色阈值
red_threshold = (0, 100, 15, 51, -1, 34)     # 红色阈值
green_threshold = (0, 100, -33, -12, 7, 34) # 绿色阈值
blue_threshold = (15, 25, -10, 12, -34, -9) # 蓝色阈值

# 初始化摄像头设置
def init_setup():
    global sensor, clock  # 将 sensor 和 clock 设为全局变量
    sensor.reset()                         # 重置并初始化摄像头传感器
    sensor.set_pixformat(sensor.RGB565)    # 设置图像格式为 RGB565（彩色）
    sensor.set_framesize(sensor.QVGA)      # 设置分辨率为 QVGA（320x240）
    sensor.skip_frames(time=2000)          # 等待2秒，使设置生效
    sensor.set_auto_gain(False)            # 关闭自动增益，以避免颜色识别误差
    sensor.set_auto_whitebal(False)        # 关闭自动白平衡，以保持颜色一致性
    clock = time.clock()                   # 创建时钟对象，用于帧率控制

# 主循环函数
def main():
    while True:
        pin0.value(0)  # 将 P0 引脚设置为低电平，默认状态

        size = 0  # 初始化检测到的物体大小为0
        img = sensor.snapshot()  # 捕捉当前图像帧

        # 在图像中查找符合红色阈值的区域，步长为100
        blobs = img.find_blobs([red_threshold], x_stride=100, y_stride=100)
        b = []

        if len(blobs) == 1:  # 如果检测到一个红色区域
            b = blobs[0]
            pin0.value(1)  # 将 P0 引脚设置为高电平，表示检测到目标

        if b:
            img.draw_rectangle(b[0:4])    # 在图像中绘制检测到的区域的矩形框
            img.draw_cross(b[5], b[6])    # 在检测区域的中心点绘制十字标记
            Lm = (b[2] + b[3]) / 2        # 计算检测区域的平均直径（像素值）

            # 调试时使用：打印检测到的像素直径
            print('Lm =', Lm)

            w = K * b[2]  # 计算物体的实际宽度
            h = K * b[3]  # 计算物体的实际高度
            size = (w + h) / 2  # 计算物体的平均尺寸
            # print(size)  # 可选：打印物体尺寸

        print((pin0.value(), size))  # 输出 P0 引脚电平状态和检测到的物体尺寸
        time.sleep_ms(50)  # 延时50毫秒，控制检测频率

# 程序入口
if __name__ == '__main__':
    init_setup()  # 执行初始化设置
    main()        # 进入主循环
