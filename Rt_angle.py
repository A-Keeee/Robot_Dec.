import sensor, image, time
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
clock = time.clock()
while(True):
    clock.tick()
    img = sensor.snapshot()
    lines = img.find_lines(threshold = 1000, theta_margin = 25, rho_margin = 25)
    for i in range(0,len(lines)-1):
        for j in range(i+1,len(lines)):
            ax1 = lines[i].x1()
            ay1 = lines[i].y1()
            ax2 = lines[i].x2()
            ay2 = lines[i].y2()
            if(ax1 == ax2):
                ax1 = ax1 + 0.01
            k0 = (ay2 - ay1)/(ax2 - ax1)      # 第一条直线斜率
            b0 = ay1 - k0*ax1                 # 第一条直线截距
            bx1 = lines[j].x1()
            by1 = lines[j].y1()
            bx2 = lines[j].x2()
            by2 = lines[j].y2()
            if(bx1 == bx2):
                bx1 = bx1 + 0.01
            k1 = (by2 - by1)/(bx2 - bx1)      # 第二条直线斜率
            b1 = by1 - k1*bx1                 # 第二条直线截距
            if(k0 == k1):
                k0 = k0 + 0.1
            # 计算角点坐标
            intersectionx = (b1-b0)/(k0-k1)
            intersectiony = k0*intersectionx + b0
            img.draw_cross(int(intersectionx), int(intersectiony),color = (0, 255, 0))
            # 判断是否垂直
            kk = float(k0) * float(k1)
            if(kk > -1.5 and kk < -0.5 ):
                img.draw_circle(int(intersectionx), int(intersectiony),10,color = (255, 0, 0))
    for l in lines:
        img.draw_line(l.line(), color = (255, 0, 255))
    print("FPS %f" % clock.fps())
