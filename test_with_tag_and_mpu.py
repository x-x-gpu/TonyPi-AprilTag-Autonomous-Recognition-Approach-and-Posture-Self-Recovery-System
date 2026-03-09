# coding=utf8
import sys
import cv2
import time
import math
import threading
import numpy as np
import hiwonder.Mpu6050 as Mpu6050
import hiwonder.PID as PID
import hiwonder.Misc as Misc
import hiwonder.Board as Board
import hiwonder.Camera as Camera
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle
import hiwonder.apriltag as apriltag

if __name__ == '__main__':
    from CameraCalibration.CalibrationConfig import *
else:
    from Functions.CameraCalibration.CalibrationConfig import *

# 自动踢球
debug = False

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 加载参数
param_data = np.load(calibration_param_path + '.npz')

# 获取参数
mtx = param_data['mtx_array']
dist = param_data['dist_array']
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('red',)
# 设置检测颜色
def setBallTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, (), 'SetBallColor')

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()

# 初始位置
def initMove():
    Board.setPWMServoPulse(1, servo_data['servo1'], 500)
    Board.setPWMServoPulse(2, servo_data['servo2'], 500)

t1 = 0
d_x = 20
d_y = 20
step = 1
step_ = 1
x_dis = servo_data['servo2']
y_dis = servo_data['servo1']
last_status = ''
start_count = True
centerX, centerY = -2, -2
x_pid = PID.PID(P=0.4, I=0.01, D=0.02)#pid初始化
y_pid = PID.PID(P=0.35, I=0.01, D=0.02)

# =======================================================
# 线程锁定义
data_lock = threading.Lock() # 用于保护 centerX, centerY 等共享数据
action_lock = threading.Lock() # 用于保护 AGC.runActionGroup() 调用
# =======================================================

# 变量重置
def reset():
    global t1
    global d_x, d_y
    global last_status
    global start_count
    global step, step_
    global x_dis, y_dis
    global __target_color
    global centerX, centerY

    t1 = 0
    d_x = 20
    d_y = 20
    step = 1
    step_ = 1
    x_pid.clear()
    y_pid.clear()
    x_dis = servo_data['servo2']
    y_dis = servo_data['servo1']
    last_status = ''
    start_count = True
    __target_color = ()
    
    # 重置共享变量时也要使用数据锁
    with data_lock:
        centerX, centerY = -2, -2
    
# app初始化调用
def init():
    print("KickBall Init")
    load_config()
    initMove()

__isRunning = False
# app开始玩法调用
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("KickBall Start")

# app停止玩法调用
def stop():
    global __isRunning
    __isRunning = False
    print("KickBall Stop")

# app退出玩法调用
def exit():
    global __isRunning
    __isRunning = False
    # 退出时执行的动作也需要上锁
    with action_lock:
        AGC.runActionGroup('stand_slow')
    print("KickBall Exit")

# 找出面积最大的轮廓
# 参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if 1000 > contour_area_temp >= 2:  # 只有在面积大于设定值时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓

CENTER_X = 350
#执行动作组
def move():
    global t1
    global d_x
    global d_y
    global step
    global step_
    global x_dis
    global y_dis
    global last_status
    global start_count
    
    while True:
        if debug:
            return
        if __isRunning:
            # 在读取共享变量之前加数据锁
            with data_lock:
                current_centerX = centerX
                current_centerY = centerY

            #print('x', centerX)
            if current_centerX >= 0:
                step_ = 1
                d_x, d_y = 20, 20
                start_count = True
                #print('STEP', step)
                if step == 1:                       
                    if x_dis - servo_data['servo2'] > 150:#不在中心，根据方向让机器人转向一步
                        with action_lock: # 动作加锁
                            AGC.runActionGroup('turn_left_small_step')
                    elif x_dis - servo_data['servo2'] < -150:
                        with action_lock: # 动作加锁
                            AGC.runActionGroup('turn_right_small_step')
                    else:
                        step = 2
                elif step == 2:
                    if y_dis == servo_data['servo1']:
                        if x_dis == servo_data['servo2'] - 400:
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('turn_right',2)
                        elif x_dis == servo_data['servo2'] + 400:
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('turn_left',2)
                        elif 350 < current_centerY <= 380:
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('go_forward_one_step')
                            last_status = 'go'
                            step = 1
                        elif 120 < current_centerY <= 350:
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('go_forward')
                            last_status = 'go'
                            step = 1
                        elif 0 <= current_centerY <= 120 and abs(x_dis - servo_data['servo2']) <= 200:
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('go_forward_fast')
                            last_status = 'go'
                            step = 1
                        else:
                            step = 3
                    else:
                        if x_dis == servo_data['servo2'] - 400:
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('turn_right',2)
                        elif x_dis == servo_data['servo2'] + 400:
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('turn_left',2)
                        else:
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('go_forward_fast')
                            last_status = 'go'
                elif step == 3:
                    if y_dis == servo_data['servo1']:
                        if abs(current_centerX - CENTER_X) <= 40:#不在中心，根据方向让机器人转向一步
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('left_move')
                        elif 0 < current_centerX < CENTER_X - 50 - 40:
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('left_move_fast')
                            time.sleep(0.2)
                        elif CENTER_X + 50 + 40 < current_centerX:                    
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('right_move_fast')
                            time.sleep(0.2)
                        else:
                            step = 4 
                    else:
                        if 270 <= x_dis - servo_data['servo2'] < 480:#不在中心，根据方向让机器人转向一步
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('left_move_fast')
                            time.sleep(0.2)
                        elif abs(x_dis - servo_data['servo2']) < 170:
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('left_move')
                        elif -480 < x_dis - servo_data['servo2'] <= -270:                     
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('right_move_fast')
                            time.sleep(0.2)
                        else:
                            step = 4          
                elif step == 4:
                    if y_dis == servo_data['servo1']:
                        if 380 < current_centerY <= 440:
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('go_forward_one_step')
                            last_status = 'go'
                        elif 0 <= current_centerY <= 380:
                            with action_lock: # 动作加锁
                                AGC.runActionGroup('go_forward')
                            last_status = 'go'
                        else:
                            # 踢球动作
                            # if centerX < CENTER_X:
                            #     with action_lock:
                            #         AGC.runActionGroup('left_shot_fast')
                            # else:
                            #     with action_lock:
                            #         AGC.runActionGroup('right_shot_fast')
                            # NEED TO CHANGE
                            step = 1
                    else:
                        step = 1
            elif current_centerX == -1:
                if last_status == 'go':
                    last_status = ''
                    with action_lock: # 动作加锁
                        AGC.runActionGroup('back_fast', with_stand=True)          
                elif start_count:
                    start_count = False
                    t1 = time.time()
                else:
                    if time.time() - t1 > 0.5:
                        #print(x_dis, y_dis, d_x)
                        #print(step_, servo_data['servo2'])
                        if step_ == 5:
                            x_dis += d_x
                            if abs(x_dis - servo_data['servo2']) <= abs(d_x):
                                with action_lock: # 动作加锁
                                    AGC.runActionGroup('turn_right')
                                step_ = 1
                        if step_ == 1 or step_ == 3:
                            x_dis += d_x            
                            if x_dis > servo_data['servo2'] + 400:
                                if step_ == 1:
                                    step_ = 2
                                d_x = -d_x
                            elif x_dis < servo_data['servo2'] - 400:
                                if step_ == 3:
                                    step_ = 4
                                d_x = -d_x
                        elif step_ == 2 or step_ == 4:
                            y_dis += d_y
                            if y_dis > 1200:
                                if step_ == 2:
                                    step_ = 3
                                d_y = -d_y
                            elif y_dis < servo_data['servo1']:
                                if step_ == 4:                                
                                    step_ = 5
                                d_y = -d_y
                            Board.setPWMServoPulse(1, y_dis, 20)
                            Board.setPWMServoPulse(2, x_dis, 20)
                            time.sleep(0.02)
            else:
                time.sleep(0.01)
        else:
            break 

#启动动作的线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

size = (320, 240)

def measure_single_tag_distance(im, detection, cam_matrix, dist_coeffs, tag_half_length):
    """
    对检测到的第一个 AprilTag 进行 PnP 姿态估计，并返回平移向量 tVec。
    
    Args:
        im (np.ndarray): 待绘制的图像。
        detections (list): pupil_apriltags.Detector.detect() 的返回结果列表。
        cam_matrix (np.ndarray): 摄像头内参矩阵 K。
        dist_coeffs (np.ndarray): 摄像头畸变系数 D。
        tag_half_length (float): 码的半边长。
        
    Returns:
        np.ndarray or None: 如果成功，返回平移向量 tVec (3x1 数组)；否则返回 None。
    """
    
    # 1. 定义 3D 世界坐标 (obj_points)
    # 顺序: 左上(tl), 右上(tr), 右下(br), 左下(bl)
    obj_points = np.array([
        [-tag_half_length, -tag_half_length, 0.0],
        [ tag_half_length, -tag_half_length, 0.0],
        [ tag_half_length,  tag_half_length, 0.0],
        [-tag_half_length,  tag_half_length, 0.0]
    ], dtype=np.float32)
    
    if not detection:
        print("未检测到 AprilTag。")
        return None

    # 仅处理第一个检测结果
    tag_id = detection.tag_id
    
    # 提取 2D 像素坐标 (img_points)
    corners = detection.corners 
    img_points = corners.astype(np.float32) 
    
    # 绘制边界
    corners_int = np.rint(corners).astype(np.int32)
    cv2.polylines(im, [corners_int], isClosed=True, color=(255, 0, 0), thickness=3)

    # 初始化旋转和平移向量
    rVec = np.zeros((3, 1), dtype=np.float64)
    tVec = np.zeros((3, 1), dtype=np.float64)

    # 2. 调用 cv2.solvePnP 求解位姿
    success, rVec, tVec = cv2.solvePnP(
        obj_points,      
        img_points,      
        cam_matrix,      
        dist_coeffs,     
        rVec,            
        tVec,            
        useExtrinsicGuess=False,
        flags=cv2.SOLVEPNP_ITERATIVE
    )

    if success:
        distance_z = tVec[2, 0]
        
        # 3. 打印结果 (非列表形式)
        print("\n=============================================")
        print(f"✅ 成功测距单个 Tag (ID: {tag_id})")
        print(f"平移向量 T (Tx, Ty, Tz):\n{tVec.T}")
        print(f"➡️ 沿 Z 轴的距离 (Tz): {distance_z:.2f} {tag_half_length} 的单位")
        print("=============================================")
        
        # 标注结果
        center = tuple(np.rint(detection.center).astype(int))
        cv2.putText(im, f"D: {distance_z:.2f}", center, 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        return distance_z
    else:
        print(f"Tag ID {tag_id}: PnP 求解失败。")
        return None

beer = True
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
def apriltagDetect(img):    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, return_image=False)

    if len(detections) != 0:
        for detection in detections:                 
            corners = np.rint(detection.corners)  # 获取四个角点
            cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)
            tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family
            tag_id = int(detection.tag_id)  # 获取tag_id

            object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])  # 中心点
            
            object_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # 计算旋转角
            
            return tag_family, tag_id, detection
    return None, None, None

def run(img):
    global tag_id
    global action_finish
    global mtx
    global dist
    global tag_half_length
    global centerX
    global centerY

    tag_id = None
    action_finish = True
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    tag_family, tag_id, detection = apriltagDetect(img) # apriltag检测
    
    
    # ----------------------------------------------------------------
    # 写入共享变量时加数据锁
    with data_lock:
        if tag_id is not None:
            centerX, centerY = int(detection.center[0]), int(detection.center[1])  # 中心点
            
            cv2.putText(img, "tag_id: " + str(tag_id), (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 0, 255], 2)
            cv2.putText(img, "tag_family: " + tag_family, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 0, 255], 2)
            
            radius = measure_single_tag_distance(img, detection, mtx, dist, tag_half_length)
        else:
            centerX, centerY = -1, -1
            
            cv2.putText(img, "tag_id: None", (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 0, 255], 2)
            cv2.putText(img, "tag_family: None", (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, [0, 0, 255], 2)
    # ----------------------------------------------------------------
    
    return img

#mpu6050初始化
mpu = Mpu6050.mpu6050(0x68)
mpu.set_gyro_range(mpu.GYRO_RANGE_2000DEG)
mpu.set_accel_range(mpu.ACCEL_RANGE_2G)

def standup():
    count1 = 0
    count2 = 0
    count3 = 0
    for i in range(20):
        try:
            accel_date = mpu.get_accel_data(g=True) #获取传感器值
            angle_y = int(math.degrees(math.atan2(accel_date['y'], accel_date['z']))) #转化为角度值
            
            #print(count1, count2, count3, angle_y)
            if abs(angle_y) > 160: 
                count1 += 1
            else:
                count1 = 0
            if abs(angle_y) < 10:
                count2 += 1
            else:
                count2 = 0
            time.sleep(0.1)
            count3 += 1
            if count3 > 5 and count1 < 3 and count2 < 3:
                break
            if count1 >= 10: #往后倒了一定时间后起来
                count1 = 0
                with action_lock: # 动作加锁
                    AGC.runActionGroup('stand_up_back')
                break
            elif count2 >= 10: #后前倒了一定时间后起来
                count2 = 0
                with action_lock: # 动作加锁
                    AGC.runActionGroup('stand_up_front')
                break
        except BaseException as e:
            print(e)

def StandUp():
    while not __isRunning :
        pass
    while __isRunning:
        standup()
th2 = threading.Thread(target=StandUp)
th2.setDaemon(True)
th2.start()

if __name__ == '__main__':
    debug = False
    if debug:
        print('Debug Mode')
    
    #加载参数
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']

    tag_half_length = 0.15  # 假设 Tag 一半的长度为 0.15m

    init()
    start()
    __target_color = ('red',)
    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open()         
    
    with action_lock: # 初始动作加锁
        AGC.runActionGroup('stand')
        
    while True:
        ret, img = my_camera.read()
        if ret:
            frame = img.copy()
            Frame = run(frame)          
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()