# TonyPi AprilTag Autonomous Targeting and Recovery

## 项目标题

TonyPi AprilTag 自主识别、接近与姿态自恢复系统

## 项目描述

这是一个基于幻尔科技 TonyPi 机器人的二次开发项目。项目在原有动作组控制能力基础上，融合 AprilTag 视觉识别、PnP 距离估计、云台联动搜索、步态决策控制和 MPU6050 姿态检测，实现机器人对目标标签的自主发现、接近和跌倒后自恢复。

当前工作区的核心脚本为 [test_with_tag_and_mpu.py](test_with_tag_and_mpu.py)，适用于已经部署 TonyPi 运行环境、标定参数和动作组资源的设备端运行。

## 项目特点

- 基于 AprilTag 的目标识别与标签 ID 解析
- 基于 OpenCV `solvePnP` 的标签距离估计
- 通过舵机云台扫描实现目标搜索与重新捕获
- 基于动作组的自主前进、转向、横移控制
- 使用 MPU6050 进行跌倒检测，并触发前倒或后倒起身动作
- 使用线程锁保护共享视觉数据和动作调用，降低并发冲突风险

## 功能概览

### 1. 视觉检测

程序通过 `hiwonder.apriltag` 检测画面中的 AprilTag，读取以下信息：

- `tag_family`
- `tag_id`
- 标签中心点坐标
- 四角点信息

检测成功后，程序会在图像中绘制标签轮廓，并在画面上叠加标签 ID、标签族和估计距离。

### 2. 距离测量

程序使用相机标定参数和 AprilTag 四个角点，调用 OpenCV 的 `solvePnP` 估算目标相对于摄像头的位姿，并输出 Z 轴方向距离值。该能力可用于后续的接近控制、触发动作或比赛策略判断。

### 3. 自主移动决策

`move()` 线程根据标签在图像中的位置和云台舵机偏移量，控制机器人执行：

- 小步左转 / 右转
- 前进 / 快速前进
- 左移 / 右移
- 目标丢失后的舵机扫描搜索

现有逻辑已经包含完整的搜索与接近流程，但最终“踢球动作”位置仍保留为待修改状态，适合根据你的比赛任务继续扩展。

### 4. 跌倒自恢复

`StandUp()` 线程持续读取 MPU6050 加速度数据，估算机体姿态角度：

- 当判断为后倒时，执行 `stand_up_back`
- 当判断为前倒时，执行 `stand_up_front`

这部分能力可以提升比赛中的连续作战稳定性。

## 运行环境

建议在 TonyPi 原生运行环境或已正确移植的 Python 3 环境中执行。

### 硬件依赖

- 幻尔科技 TonyPi 机器人本体
- 摄像头模组
- MPU6050 姿态传感器
- 已配置好的舵机与动作组文件

### 软件依赖

- Python 3
- OpenCV
- NumPy
- 幻尔科技 `hiwonder` 相关模块
- AprilTag Python 接口
- 相机标定参数文件

### 依赖的配置与资源

- 相机标定参数：`calibration_param_path + '.npz'`
- LAB 阈值配置：`yaml_handle.lab_file_path`
- 舵机配置：`yaml_handle.servo_file_path`
- 摄像头设置：`/boot/camera_setting.yaml`
- 动作组资源：如 `stand`、`go_forward`、`turn_left_small_step`、`stand_up_front` 等

## 使用方法

### 1. 部署脚本

将 [test_with_tag_and_mpu.py](test_with_tag_and_mpu.py) 放置到 TonyPi 对应功能目录，确保它可以正确导入以下模块：

- `hiwonder.Mpu6050`
- `hiwonder.PID`
- `hiwonder.Board`
- `hiwonder.Camera`
- `hiwonder.ActionGroupControl`
- `hiwonder.yaml_handle`
- `hiwonder.apriltag`

### 2. 检查标定与动作组

运行前确认：

- 相机标定文件存在且路径正确
- 舵机 YAML 配置已完成
- 所需动作组已经在设备中可调用
- AprilTag 实际尺寸与 `tag_half_length` 参数一致

### 3. 启动程序

```bash
python3 test_with_tag_and_mpu.py
```

程序启动后会：

- 初始化舵机位置
- 启动自主控制线程
- 启动姿态监测线程
- 打开摄像头并实时显示识别画面

按 `Esc` 可退出图像窗口。

## 代码结构说明

主要流程如下：

1. 加载相机标定与舵机配置
2. 初始化云台舵机位置
3. 启动动作线程 `move()`
4. 启动姿态恢复线程 `StandUp()`
5. 主线程采集摄像头画面并执行 `run()`
6. 检测 AprilTag 后更新共享目标位置
7. 动作线程根据目标位置执行接近策略

## 已知说明

- 脚本中仍保留了部分原“踢球”任务命名，例如 `KickBall Init`，但当前核心能力已经转向 AprilTag 目标识别与接近。
- `setBallTargetColor()` 和部分颜色识别变量目前未在主流程中实际使用，属于原项目遗留接口。
- `step == 4` 分支中的最终攻击或踢击动作还未完成，需要结合你的比赛规则补齐。

## 适用场景

- 机器人格斗赛中的目标识别与自主接近
- 基于视觉标签的定位触发任务
- 机器人自主对抗或自主导航的前期验证
- TonyPi 平台上的多线程控制与动作组联调实验

## 后续优化建议

- 将状态机逻辑拆分为独立类，提升可维护性
- 依据标签距离动态调节步态速度和动作切换阈值
- 补充最终攻击动作或比赛任务动作组
- 对 AprilTag 丢失、误检和遮挡情况增加容错策略
- 增加日志输出和调试开关，便于现场排障

## 文件说明

- [test_with_tag_and_mpu.py](test_with_tag_and_mpu.py)：主控制脚本，包含视觉识别、动作决策、距离估计和姿态恢复逻辑

## License

本仓库用于 TonyPi 机器人二次开发示例与比赛项目整理。若对外发布，请根据你所使用的第三方库、动作组资源和平台软件许可情况自行补充许可证说明。