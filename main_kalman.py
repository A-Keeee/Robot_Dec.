# 导入所需的模块
import sensor, image, time
from pyb import LED
import car
from pid import PID
import math

# ================== 自定义 array 类定义 ==================

class array:
    def __init__(self, M: list):
        self.M = M
        self.shape = self.get_shape()
        self.ndim = len(self.shape)

    def __len__(self):
        return len(self.M)

    def __getitem__(self, *args):
        if isinstance(args[0], tuple):
            assert len(args[0]) <= self.ndim, 'Index out of range'
            indices = list(args[0])
            def get_value(a, num):
                if len(indices) - 1 == num:
                    return a[indices[num]]
                return get_value(a[indices[num]], num + 1)
            return get_value(self.M, 0)
        elif isinstance(args[0], int):
            return self.M[args[0]]

    def get_shape(self):
        shape = []
        def get_len(a):
            try:
                shape.append(len(a))
                get_len(a[0])
            except:
                pass
        get_len(self.M)
        return tuple(shape)

    def __add__(self, other):
        assert self.ndim == 2 and other.ndim == 2 and self.shape == other.shape, 'Addition requires two 2D arrays of the same shape'
        rows, cols = self.shape
        return array([[self[i][j] + other[i][j] for j in range(cols)] for i in range(rows)])

    def __sub__(self, other):
        assert self.ndim == 2 and other.ndim == 2 and self.shape == other.shape, 'Subtraction requires two 2D arrays of the same shape'
        rows, cols = self.shape
        return array([[self[i][j] - other[i][j] for j in range(cols)] for i in range(rows)])

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return array([[self[i][j] * other for j in range(self.shape[1])] for i in range(self.shape[0])])
        assert self.ndim == 2 and other.ndim == 2, 'Multiplication requires two 2D arrays'
        rows_a, cols_a = self.shape
        rows_b, cols_b = other.shape
        assert cols_a == rows_b, 'Cannot multiply: Incompatible dimensions'
        result = [[0 for _ in range(cols_b)] for _ in range(rows_a)]
        for i in range(rows_a):
            for j in range(cols_b):
                for k in range(cols_a):
                    result[i][j] += self[i][k] * other[k][j]
        return array(result)

    @property
    def T(self):
        assert self.ndim == 2, 'Transpose requires a 2D array'
        rows, cols = self.shape
        transposed = [[self[j][i] for j in range(rows)] for i in range(cols)]
        return array(transposed)

    def det(self):
        shape = self.shape
        assert self.ndim == 2 and shape[0] == shape[1], 'Determinant requires a square matrix'
        n = shape[0]
        m = [row[:] for row in self.M]  # Deep copy
        det = 1
        for col in range(n):
            # Find the pivot
            pivot = m[col][col]
            pivot_row = col
            for row in range(col + 1, n):
                if abs(m[row][col]) > abs(pivot):
                    pivot = m[row][col]
                    pivot_row = row
            if pivot == 0:
                return 0
            if pivot_row != col:
                # Swap rows
                m[col], m[pivot_row] = m[pivot_row], m[col]
                det *= -1
            det *= m[col][col]
            # Eliminate below
            for row in range(col + 1, n):
                factor = m[row][col] / m[col][col]
                for k in range(col, n):
                    m[row][k] -= factor * m[col][k]
        return det

    def inv(self):
        shape = self.shape
        assert self.det() != 0, 'Matrix is not invertible'
        n = shape[0]
        m = [row[:] for row in self.M]  # Deep copy
        I = [[1 if i == j else 0 for j in range(n)] for i in range(n)]
        for col in range(n):
            # Find the pivot
            pivot = m[col][col]
            pivot_row = col
            for row in range(col + 1, n):
                if abs(m[row][col]) > abs(pivot):
                    pivot = m[row][col]
                    pivot_row = row
            if pivot == 0:
                raise ValueError('Matrix is not invertible')
            if pivot_row != col:
                # Swap rows in both m and I
                m[col], m[pivot_row] = m[pivot_row], m[col]
                I[col], I[pivot_row] = I[pivot_row], I[col]
            # Normalize the pivot row
            pivot = m[col][col]
            m[col] = [element / pivot for element in m[col]]
            I[col] = [element / pivot for element in I[col]]
            # Eliminate other rows
            for row in range(n):
                if row != col:
                    factor = m[row][col]
                    m[row] = [m[row][k] - factor * m[col][k] for k in range(n)]
                    I[row] = [I[row][k] - factor * I[col][k] for k in range(n)]
        return array(I)

    def __str__(self):
        return str(self.M)

    @staticmethod
    def eye(size, value=1):
        M = [[value if i == j else 0 for j in range(size)] for i in range(size)]
        return array(M)

    @staticmethod
    def zeros(shape: tuple):
        return array([[0 for _ in range(shape[1])] for _ in range(shape[0])])

    @staticmethod
    def ones(shape: tuple):
        return array([[1 for _ in range(shape[1])] for _ in range(shape[0])])

    # 解线性方程组 Ax = B
    @staticmethod
    def solve(A, B):
        if A.det() == 0:
            raise ValueError("No solution exists")
        inv_A = A.inv()
        return inv_A * B

# ================== Tracker1D 类定义（1D 卡尔曼滤波器） ==================

class Tracker1D:
    def __init__(self, A, H, Q, R, ID, lose_threshold=20, motion_trail_len=10):
        """
        初始化 Tracker1D 实例
        A: 状态转移矩阵，array 实例，尺寸 2x2
        H: 观测矩阵，array 实例，尺寸 1x2
        Q: 过程噪声协方差矩阵，array 实例，尺寸 2x2
        R: 观测噪声协方差矩阵，array 实例，尺寸 1x1
        ID: 追踪器的唯一标识
        lose_threshold: 多少帧后认为目标丢失
        motion_trail_len: 运动轨迹长度
        """
        self.A = A  # 状态转移矩阵
        self.H = H  # 观测矩阵
        self.Q = Q  # 过程噪声协方差
        self.R = R  # 观测噪声协方差
        self.ID = ID
        self.lose_threshold = lose_threshold
        self.motion_trail_len = motion_trail_len

        self.P = array.eye(2)  # 误差协方差矩阵（初始化为单位矩阵）
        self.active = 0
        self.updated = False  # 是否在该帧已经更新了的标志位
        self.last_X_posterior = None  # 状态向量 [position, velocity]
        self.motion_trail_measure = []
        self.motion_trail_pre = []

    def __call__(self, measurement, find):
        if find:
            self.add_motion_trail_measure(measurement)
            if self.active == 0:
                # 初始化状态向量
                self.last_X_posterior = array([[measurement], [0]])  # 初始速度设为0
                self.active = self.lose_threshold
                self.updated = True
                return measurement
            else:
                dt = 1  # 时间间隔，可以根据实际情况调整
                # 更新状态转移矩阵 A 中的时间步长
                self.A.M[0][1] = dt
                # 预测
                X_prior = self.A * self.last_X_posterior  # A * X_prev
                P_prior = self.A * self.P * self.A.T + self.Q  # A * P * A^T + Q

                # 计算卡尔曼增益 K = P_prior * H^T * inv(H * P_prior * H^T + R)
                H_T = self.H.T  # H^T
                S = self.H * P_prior * H_T + self.R  # H * P_prior * H^T + R
                try:
                    S_inv = S.inv()
                except:
                    S_inv = array.eye(1)  # 如果不可逆，使用单位矩阵代替

                K = P_prior * H_T * S_inv  # P_prior * H^T * S_inv

                # 更新
                Y = array([[measurement]])  # 观测值
                H_X_prior = self.H * X_prior  # H * X_prior
                Z = Y - H_X_prior  # 观测残差
                self.last_X_posterior = X_prior + K * Z  # X_posterior = X_prior + K * Z
                self.P = (array.eye(2) - K * self.H) * P_prior  # P_posterior = (I - K * H) * P_prior

                # 记录预测值
                position = int(self.last_X_posterior[0][0])
                self.add_motion_trail_pre(position)
                self.active = self.lose_threshold
                self.updated = True
                return position
        else:
            self.active -= 1
            if self.last_X_posterior is not None:
                # 预测
                X_prior = self.A * self.last_X_posterior  # A * X_prev
                P_prior = self.A * self.P * self.A.T + self.Q  # A * P * A^T + Q

                self.last_X_posterior = X_prior
                self.P = P_prior  # 更新误差协方差矩阵

                # 记录预测值
                position = int(self.last_X_posterior[0][0])
                self.add_motion_trail_pre(position)
                return position
            else:
                return 0  # 默认返回0

    def add_motion_trail_measure(self, measurement):
        self.motion_trail_measure.append(int(measurement))
        if len(self.motion_trail_measure) >= self.motion_trail_len:
            self.motion_trail_measure.pop(0)

    def add_motion_trail_pre(self, position):
        self.motion_trail_pre.append(int(position))
        if len(self.motion_trail_pre) >= self.motion_trail_len:
            self.motion_trail_pre.pop(0)

# ================== 主程序开始 ==================

# 定义用于检测暗色物体的灰度阈值
THRESHOLD = (0, 40, -23, 15, -57, 0)  # 灰度阈值范围 (min_gray, max_gray, ...)

# 初始化PID控制器，分别用于平移（左右移动）和转向（旋转角度）的控制
# PID参数需要根据实际情况进行调整
pid_x = PID(p=1.2, i=0.2, d=0.01, imax=40)      # 用于平移（左右移动）的PID
pid_turn = PID(p=1.5, i=0.1, d=0.015)           # 用于转向（旋转角度）的PID

# 打开LED灯，指示程序运行状态
LED(1).on()  # 红灯
LED(2).on()  # 绿灯
LED(3).on()  # 蓝灯

# 初始化摄像头传感器
sensor.reset()  # 重置摄像头
# sensor.set_vflip(True)  # 垂直翻转图像
# sensor.set_hmirror(True)  # 水平镜像翻转图像
sensor.set_pixformat(sensor.RGB565)  # 设置像素格式为RGB565
sensor.set_framesize(sensor.QVGA)  # 设置分辨率为QVGA (320*240)
# sensor.set_windowing([0,20,80,40])  # 可选：设置窗口区域进行图像截取
sensor.skip_frames(time=2000)  # 跳过前2000毫秒的帧，等待摄像头稳定
clock = time.clock()  # 初始化时钟，用于计算帧率

# 设置期望的移动和转向值
desired_move_x = 0      # 期望的 move_x 为0，即中心对齐
desired_move_turn = 0   # 期望的 move_turn 为0，即不旋转

# 初始化卡尔曼滤波器
# 定义状态转移矩阵 A，观测矩阵 H，过程噪声协方差 Q，观测噪声协方差 R
# 这里为一维跟踪，状态为 [position, velocity]
A = array([[1, 1],
          [0, 1]])
H = array([[1, 0]])
Q = array([[0.1, 0],
          [0, 0.1]])
R = array([[1]])
#增大Q或减小R，更相信观测值

# 创建两个独立的 Tracker1D 实例，一个用于 move_x，一个用于 move_turn
tracker_move_x = Tracker1D(A, H, Q, R, ID=1, lose_threshold=20, motion_trail_len=10)
tracker_move_turn = Tracker1D(A, H, Q, R, ID=2, lose_threshold=20, motion_trail_len=10)

# 主循环
while(True):
    clock.tick()  # 开始记录一帧的时间
    img_raw = sensor.snapshot()  # 拍摄一帧图像

    # 获取图像的下半部分
    height = img_raw.height()  # 获取图像的高度
    half_height = height // 2  # 计算下半部分的高度

    # 裁剪下半部分图像
    # crop(x, y, w, h)
    img = img_raw.crop((0, half_height*3/2, img_raw.width(), half_height))

     #图像预处理
    img = img.to_grayscale()  # 转换为灰度图像
    # img.gaussian(1)  # 应用高斯模糊，减少噪声
    img.binary([THRESHOLD], invert=True)  # 二值化
    img = img.erode(1)

#    histogram = img.get_histogram() #获取图像的直方图
#    Thresholds = histogram.get_threshold() #获取直方图的阈值
#    img.binary([(Thresholds.value(), 255)]) #二值化图像，将低于阈值的像素设为黑色（0），高于阈值的像素设为白色（255）


    # 初始化 move_x 和 move_turn
    move_x = -500  # 默认无效测量
    move_turn = -500
    line_none = 0  # 记录没有发现黑线段的次数

    # 分段切割，定位黑线偏移量（模仿初始代码的逻辑）
    regions = 6
    region_height = img.height() // regions
    midpoint = img.width() // 2  # 图像中心的x坐标

    line_points = []
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
            if 0.375 * img.width() < blob.w() < 0.875 * img.width():
                offset *= 2
            line_points.append(offset)
        else:
            line_points.append(-500)
            line_none += 1

    # 计算 move_x 和 move_turn
    # move_x 以从下到上的第一个有效点为基准
    for n in range(regions-1, -1, -1):
        if line_points[n] > -500:
            move_x = line_points[n]
            break
#    print(move_x)

    valid_points = [pt for pt in line_points if pt > -500]
    if valid_points:
        max_pt = max(valid_points)
        min_pt = min(valid_points)
#        print(max_pt)
#        print(min_pt)
#        print(line_points[-1])
        tolerance = 40  # 设置容差值，根据实际情况调整
        turn_left = False
        turn_right = False
        if line_points[0] < max_pt-40 :
            turn_left = True
        if line_points[0] > min_pt+40 :
            turn_right = True

        # 如果所有有效点的差值小于容差，则认为move_turn稳定，设为0
        if (max_pt - min_pt) < tolerance:
            move_turn = 0
            turn = False
        else:
            turn = True
            move_turn = sum(valid_points) / len(valid_points)
    else:
        move_turn = -500  # 无效测量



    # 使用卡尔曼滤波器更新 move_x 和 move_turn
    # 对于 move_x
    if move_x != -500:  # 检查是否有有效测量
        filtered_move_x = tracker_move_x(move_x, True)
    else:
        filtered_move_x = tracker_move_x(0, False)

    # 对于 move_turn
    if move_turn != -500:  # 检查是否有有效测量
        filtered_move_turn = tracker_move_turn(move_turn, True)
    else:
        filtered_move_turn = tracker_move_turn(0, False)

    # 计算误差
    move_x_err = desired_move_x - filtered_move_x
    move_turn_err = desired_move_turn - filtered_move_turn
#    print(move_x_err,move_turn_err)

    # 使用PID控制器计算调整值
    scaler = 1  # 可以根据需要调整比例系数
    y_speed = -pid_x.get_pid(move_x_err, scaler)  # 基于 move_x_err 控制 y 速度（左右偏移的调整）
    z_speed = pid_turn.get_pid(move_turn_err, scaler)  # 基于 move_turn_err 控制 z 速度（旋转角度的调整）

    if turn == True :
        print("turn")
        if turn_left == True :
            print("left")
            y_speed = abs(y_speed)
            z_speed = abs(z_speed)
        if turn_right == True :
            print("right")
            y_speed = abs(y_speed)*(-1)
            z_speed = abs(z_speed)*(-1)

    # 设置前进速度，可以根据需要调整或使用 PID 控制
    x_speed = 300 - abs(z_speed) * 0.5  # 固定的前进速度，也可以根据需要调整

    # 控制小车的运动
    try:
        car.send_speed(x_speed, y_speed, z_speed)#y向右为正 z逆时针为正
    except Exception as e:
        print("Error during sending speed:", e)
        car.send_speed(50, -50, 0)  # 执行原地转向或其他安全操作

    # 打印 PID 输出值，方便调试
    print("y_speed: {}, z_speed: {}".format(y_speed, z_speed))

    # 如果连续多次未检测到黑线，则停止小车并重置 PID
    if line_none >= regions - 1:
        car.send_speed(0, 0, 0)  # 停止移动
        pid_x.reset_I()  # 重置平移 PID 的积分项
        pid_turn.reset_I()  # 重置转向 PID 的积分项

    # 可选：打印当前帧率
#     print(clock.fps())
