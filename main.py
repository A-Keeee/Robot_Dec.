# 导入所需的模块
import sensor, image, time
from pyb import LED
import car
import color
from pid import PID
import math

def color_detect(img):
    red = (0, 100, 22, 53, -9, 41)  # 红色阈值
    green = (0, 100, -55, -12, -9, 22) # 绿色阈值
    blue = (0, 100, 0, 30, -49, -25) # 蓝色阈值
    # 在图像中查找符合红色阈值的区域，步长为100
    blobs = img.find_blobs([red,green,blue], x_stride=20, y_stride=20)
    b = []

    if blobs:
        for i in range(0,len(blobs)) :
            b = blobs[i]
            img.draw_rectangle(b[0:4])    # 在图像中绘制检测到的区域的矩形框
            img.draw_cross(b[5], b[6])    # 在检测区域的中心点绘制十字标记
            print(blobs[i].code()) # 红色是1 绿色是2 蓝色是4
            car.send_color(blobs[i].code())

    time.sleep_ms(50)  # 延时50毫秒，控制检测频率


def line_detect(img,z_correct):
    # 图像预处理
    img = img.to_grayscale()  # 转换为灰度图像
    img.gaussian(1)  # 应用高斯模糊，减少噪声
    img.binary([THRESHOLD], invert=True)  # 二值化
    img = img.erode(1)


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
        blobs = img.find_blobs([THRESHOLD], roi=roi, x_stride=5, y_stride=5, pixels_threshold=20)

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

    # 计算 move_x 和 move_turn
    # move_x 以从下到上的第一个有效点为基准
    for n in range(regions-1, -1, -1):
        if line_points[n] > -500000:
            move_x = line_points[n]
            break
#    print(move_x)

    valid_points = [pt for pt in line_points if pt > -500000]
    valid_widths = [wd for wd in blobs_width if wd != 0 ]
#    print(valid_widths)
#    print(valid_points)

##    插值补全反光点
#    valid_points = []
#    for i in range(0,len(valid_points_raw)-1):
#        valid_points.append(valid_points_raw[i])
#        valid_points.append((valid_points_raw[i]+valid_points_raw[i+1])/2)

    # 记录是否检测到弯道
    turn = False
    if len(valid_points)>0:
        max_pt = max(valid_points)
        min_pt = min(valid_points)
        # print(max_pt)
        # print(min_pt)
        # print(valid_points[0])
        turn = True
        move_turn = sum(valid_points) / len(valid_points)
#        print("test",move_turn)
        if abs(move_turn) < 50 :
             move_turn = (valid_points[0]-valid_points[-1])*0.6
        print ("move",move_turn)

    else:
        move_turn = 0  # 无效测量

#    # 判断左右转向
#    turn_left = True
#    turn_right = True
#    if len(valid_points)>=4 :
#        max_pt = max(valid_points)
#        min_pt = min(valid_points)
#        offset_i = []
#        index = -500000


#        # print(valid_points)
#        # print(offset_i)
#        #right case1:
#        if valid_points[0] == min_pt:
#            offset_i = []
#            for i in range(1,len(valid_points)):
#                offset_i.append(abs(valid_points[i] - valid_points[i-1]))
##            print(offset_i)
#            for i in range(1,len(offset_i)):
#                if offset_i[i-1] <= offset_i[i]:
#                    continue
#                else:
#                    turn_right = False
#                    break
#            if turn_right == True :
#                move_x = valid_points[-1]
#        #right case2:
#        elif valid_points[-1] == min_pt:
#            offset_i = []
#            for i in range(1,len(valid_points)):
#                offset_i.append(abs(valid_points[i] - valid_points[i-1]))
#            for i in range(1,len(offset_i)):
#                if offset_i[i-1] >= offset_i[i]:
#                    continue
#                else:
#                    turn_right = False
#                    break
#        #right case3:
#        else :
#            offset_i = []
#            for i in range(1,len(valid_points)):
#                offset_i.append(abs(valid_points[i] - valid_points[i-1]))
#            for i in range(0,len(offset_i)):
#                if valid_points[i] == min_pt:
#                    index = i
#            if index!=-500000 :
#                # print("right2")
#                turn_right1 = True
#                turn_right2 = True
#                for i in range(index,len(offset_i)-1):
#                    if offset_i[i+1] >= offset_i[i]:
#                        continue
#                    else:
#                        # print("right1")
#                        turn_right = False
#                        break
#                for i in range(0,index):
#                    if offset_i[i] >= offset_i[i+1]:
#                        continue
#                    else:
#                        turn_right = False
#                        break
#                if turn_right1 and turn_right2 :
#                    turn_right = True



#        #left case1:
#        if valid_points[0] == max_pt:
#            offset_i = []
#            for i in range(1,len(valid_points)):
#                offset_i.append(abs(valid_points[i] - valid_points[i-1]))
##            print(offset_i)
#            for i in range(1,len(offset_i)):
#                if offset_i[i-1] <= offset_i[i]:
#                    continue
#                else:
#                    turn_left = False
#                    break
#            if turn_left == True :
#                move_x = valid_points[-1]
#        #left case2:
#        elif valid_points[-1] == max_pt:
#            offset_i = []
#            for i in range(1,len(valid_points)):
#                offset_i.append(abs(valid_points[i] - valid_points[i-1]))
#            for i in range(1,len(offset_i)):
#                if offset_i[i-1] >= offset_i[i]:
#                    continue
#                else:
#                    turn_left = False
#                    break
#        #left case3:
#        else :
#            offset_i = []
#            for i in range(1,len(valid_points)):
#                offset_i.append(abs(valid_points[i] - valid_points[i-1]))
#            for i in range(0,len(offset_i)):
#                if valid_points[i] == max_pt:
#                    index = i
#            if index != -500000  :
#                for i in range(index,len(offset_i)-1):
#                    if offset_i[i+1] >= offset_i[i]:
#                        continue
#                    else:
#                        turn_left = False
#                        break
#                for i in range(0,index):
#                    if offset_i[i] >= offset_i[i+1]:
#                        continue
#                    else:
#                        turn_left = False
#                        break
#    else :
#        turn_left = False
#        turn_right = False



    # 计算误差
    move_x_err = desired_move_x - move_x
    move_turn_err = desired_move_turn - move_turn

    # 使用PID控制器计算调整值
    scaler = 1  # 可以根据需要调整比例系数
    y_speed = -pid_x.get_pid(move_x_err, 1)  # 基于 move_x_err 控制 y 速度（左右偏移的调整）
#    print("testy",y_speed)
    z_speed = pid_turn.get_pid(move_turn_err, 1)  # 基于 move_turn_err 控制 z 速度（旋转角度的调整）
#    z_speed = move_turn_err*2
    print("move_turn_err",move_turn_err)
    print("move_x_err",move_x_err)
    if turn == True :
        print("turn")
#        if turn_left == True :
#            print("left")
#            y_speed = abs(y_speed)
#            z_speed = abs(z_speed)
#        if turn_right == True :
#            print("right")
##            y_speed = abs(y_speed)*(-1)
#            z_speed = abs(z_speed)*(-1)

    # 设置前进速度，可以根据需要调整或使用 PID 控制
    x_speed = 600-z_speed # 固定的前进速度，也可以根据需要调整
    if (abs(move_turn_err) != 0) :
        z_correct = move_turn_err/abs(move_turn_err)
    print("correct",z_correct)


    # 如果连续多次未检测到黑线，则停止小车并重置 PID
    print(line_none)
    if line_none >= regions - 1:
#        car.send_speed(0, 0, 0)  # 停止移动
        print("testcorrect")
        car.chassis_control(0,0,z_correct*200)
        pid_x.reset_I()  # 重置平移 PID 的积分项
        pid_turn.reset_I()  # 重置转向 PID 的积分项
        pass
    elif line_none >=3:
       print("correct")
       car.chassis_control(0,0,z_correct*200)
       pass


    if y_speed != 0 and z_speed !=0:
    # 控制小车的运动
        car.chassis_control(int(x_speed), int(y_speed), int(z_speed))#y向右为正 z逆时针为正
    print("y_speed: {}, z_speed: {}".format(y_speed, z_speed))

    return z_correct



# ================== 主程序开始 ==================

# 定义用于检测暗色物体的灰度阈值
THRESHOLD = (0, 35, -23, 15, -57, 0)  # 灰度阈值范围 (min_gray, max_gray, ...)

# 初始化PID控制器，分别用于平移（左右移动）和转向（旋转角度）的控制
# PID参数需要根据实际情况进行调整
pid_x = PID(p=1.1, i=0.01, d=0.02, imax=100 ,outmax = 150)      # 用于平移（左右移动）的PID
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
correct = 0

# 主循环
while(True):
    clock.tick()  # 开始记录一帧的时间
    img = sensor.snapshot()  # 拍摄一帧图像
    flag = car.read_data()
    print(flag)
    correct = line_detect(img, correct)
##    if flag == b'$COLOR!':
##        print("test")
##        color_detect(img)
#    time.sleep_ms(10)
#    color.send_speed(0,0,100)






