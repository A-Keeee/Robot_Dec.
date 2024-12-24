# 导入所需的模块
import sensor, image, time
from pyb import LED
import car
from pid import PID
import math



def line_detect(img):
    #图像预处理
    img = img.to_grayscale()  # 转换为灰度图像
    img.gaussian(1)  # 应用高斯模糊，减少噪声
    img.binary([THRESHOLD], invert=True)  # 二值化
    img = img.erode(1)

#    histogram = img.get_histogram()
#    Thresholds = histogram.get_threshold()
#    img.binary([(Thresholds.value(), 255)])
    # 应用Canny边缘检测
#    edges = img.find_edges(image.EDGE_CANNY, threshold=(50, 80))



    # 初始化 move_x 和 move_turn
    move_x = -0  # 默认无效测量
    move_turn = -0
    line_none = 0  # 记录没有发现黑线段的次数

    # 分段切割，定位黑线偏移量（模仿初始代码的逻辑）
    regions = 6
    region_height = img.height() // regions
    midpoint = img.width() // 2  # 图像中心的x坐标

    line_points = []
    blobs_width = []
    for n in range(regions):
        roi = (0, n * region_height, img.width(), region_height)
        blobs = img.find_blobs([THRESHOLD], roi=roi, x_stride=5, y_stride=5, pixels_threshold=10)

        if blobs:
            # 找到面积最大的 blob
            blob = max(blobs, key=lambda b: b.pixels())
            # 绘制检测到的 blob 边框和中心
            img.draw_rectangle(blob.rect(), color=(255))  # 绘制白色边框
            img.draw_cross(blob.cx(), blob.cy(), color=(255))  # 绘制白色十字
            # 计算中心点偏移
            offset = blob.cx() - midpoint
            # 根据 blob 的宽度调整偏移量
            if 80 < blob.w() :
                offset *= 2
            line_points.append(offset)
            blobs_width.append(blob.w())
        else:
            line_points.append(-500000)
            blobs_width.append(0)
            line_none += 1




# ================== 主程序开始 ==================

# 定义用于检测暗色物体的灰度阈值
THRESHOLD = (0, 40, -23, 15, -57, 0)  # 灰度阈值范围 (min_gray, max_gray, ...)

# 初始化PID控制器，分别用于平移（左右移动）和转向（旋转角度）的控制
# PID参数需要根据实际情况进行调整
pid_x = PID(p=1.1, i=0.01, d=0.02, imax=100)      # 用于平移（左右移动）的PID
pid_turn = PID(p=2.8, i=0.0, d=0.2,imax=0,outmax = 140)           # 用于转向（旋转角度）的PID

# 打开LED灯，指示程序运行状态
LED(1).on()  # 红灯
LED(2).on()  # 绿灯
LED(3).on()  # 蓝灯

# 初始化摄像头传感器`
sensor.reset()  # 重置摄像头
sensor.set_vflip(True)  # 垂直翻转图像
sensor.set_hmirror(True)  # 水平镜像翻转图像
sensor.set_pixformat(sensor.RGB565)  # 设置像素格式为RGB565
sensor.set_framesize(sensor.HQVGA)  # 设置分辨率为QVGA (320*240)
# sensor.set_windowing([0,20,80,40])  # 可选：设置窗口区域进行图像截取
sensor.skip_frames(time=2000)  # 跳过前2000毫秒的帧，等待摄像头稳定
clock = time.clock()  # 初始化时钟，用于计算帧率

# 设置期望的移动和转向值
desired_move_x = 0      # 期望的 move_x 为0，即中心对齐
desired_move_turn = 0   # 期望的 move_turn 为0，即不旋转
z_correct = 0

# 主循环
while(True):
    clock.tick()  # 开始记录一帧的时间
    img = sensor.snapshot()  # 拍摄一帧图像
    line_detect(img)






