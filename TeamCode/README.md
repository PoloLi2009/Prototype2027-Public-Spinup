# BIOBUZZ Spinup — 与 DECODE 一致的项目结构

本项目沿用 `Prototype2026-Public-ChinaChampionship` 的分层方法：

```text
opmodes/teleops/TeleOpBase
        ↓ 绑定默认命令、按键和遥测
commands/DriveCommand, IntakeCommand, TransitCommand, ShooterCommand, TurretCommand
commands/FieldCentricDriveCommand, TurretHeadingHoldCommand, OdCellAimCommand
        ↓ 每个命令只申请它实际控制的 subsystem
subsystems/drive | intake | transit | shooter | turret | localization | field
        ↓ 各自的 *Constants.java
FTC HardwareMap
```

主 TeleOp 为 `BIOBUZZ: Spinup TeleOp`。它继承 `TeleOpBase`，后续可在此结构内增加
自动、视觉瞄准或红蓝场地差异，而无需重写机构初始化。

## Driver Station 硬件名称

| 配置名 | 类型 | 机构 |
| --- | --- | --- |
| `leftFrontMotor`, `rightFrontMotor`, `leftBackMotor`, `rightBackMotor` | DC Motor | 四电机麦克纳姆底盘 |
| `transitMotor` | DC Motor | 合并 intake/transit |
| `turretMotor` | DC Motor | 5:2 Yellow Jacket + 同步带云台 |
| `od` | goBILDA Pinpoint | 底盘位置与航向 |
| `leftShooterMotor`, `rightShooterMotor` | DC Motor（仅左侧 Encoder） | 两个直驱电机，机械联动三飞轮 |
| `leftPitchServo`, `rightPitchServo` | Servo | 双舵机反向联动的发射角度 |
| `limitServo` | Servo | 入云台挡球 gate/limit |

独立测试 `Test / Pitch Servo Trim Test` 只初始化 `leftPitchServo` 和
`rightPitchServo`。INIT 分别向左右舵机发送起始位置，不再关闭 PWM。
每按一次十字键上/下：仅左舵机目标 +/-0.01；十字键右/左：仅右舵机目标 +/-0.01。
X：左目标 +0.01、右目标 -0.01；Y：左目标 -0.01、右目标 +0.01。
两只舵机各自使用独立目标值。
`PitchServos.java` 内的两路初始目标目前都是 `0.50`，可分别修改。
Driver Station 遥测显示两路目标值和 SDK 最后命令值。三线 PWM 舵机没有供 Control Hub
读取的实际角度反馈，手动转动输出轴不会使目标值更新。

独立测试 `Test / Transit Shooter Test` 初始化 `transitMotor`、`leftShooterMotor`、
`rightShooterMotor` 和 `limitServo`。右扳机按住启动双飞轮，X / Y / A 按住分别让传接电机正转、反转、
FEED；这些键可与右扳机同时按。十字键上/下每按一次调整目标转速 100 RPM，
范围 0–6000 RPM，初始目标 3000 RPM。只有左飞轮连接编码器，遥测显示其 RPM；
两侧飞轮共用左侧反馈算出的功率。左扳机按住打开挡球舵机，松开关闭；停止程序时也会关闭。
传接电机三种功率分别为吸球 `+0.65`、吐球 `-0.65`、快速送球 `+1.00`；同时按键时 A 优先。
`limitServo` 实测开位为 `0.15`、关位为 `0.32`。

`Test / Field Centric OD Cell Shooter Test` 是不使用视觉的组合测试。左摇杆无头驾驶、
右摇杆 X 转底盘、按下左摇杆重置驾驶方向；Y 每按一次切换蓝/红方。
OD 场地 X 坐标决定上/下半场，结合红蓝方选择四个 CELL 中的一个。
左扳机按住以 `+0.85` 吸球，左肩键按住以 `-0.85` 吐球；这两个功率仅用于此测试。
右扳机按住后先用 OD 瞄准云台，进入 `TurretConstants.angleToleranceDegrees` 容差后
启动双飞轮。左飞轮编码器转速连续 250 ms 位于目标 `±100 RPM` 内时打开 `limitServo`，
再等待 500 ms，最后以 `+1.0` 转动 `transitMotor` 上球。松开右扳机停止射球、
关闭挡球；失去瞄准也会关挡并停止上球。十字键上/下调俯仰，左/右每次调整目标转速
`-100/+100 RPM`，初始 `3000 RPM`。开局场地位姿须在 `PinpointConstants.java` 的
`testStartFieldXmm`、`testStartFieldYmm`、`testStartFieldHeadingDegrees` 填入实测值。

Test 组的底盘、云台、舵机、飞轮、传接和视觉采集控制均由
`commands/` 与 `subsystems/` 实现；各 Test OpMode 只负责构造硬件、绑定命令和显示遥测。
`Manual Chassis Aim Test` 也使用共用的 `SequencedShotCommand`，达速后先开挡球，
等待 500 ms 再上球。`DashTuner` 的可调参数现位于 Dashboard 的 `DashboardTuner` 配置项。

`Test / Field Centric Transit Shooter Test` 使用四个底盘电机和 `od` 无头驾驶，
左摇杆控制平移，右摇杆 X 转向，按下左摇杆将当前车头设为驾驶方向零点。
双俯仰舵机在 INIT 回到 `0.50`；十字键上/下每次调整 `0.01`，右舵机方向反转。
十字键左/右每次调整双飞轮目标转速 `-100/+100 RPM`，范围 `0–6000 RPM`，初始 `3000 RPM`。
左扳机按住正转吸球，左肩键按住反转吐球。右扳机按住时，先启动双飞轮；
左侧编码器转速连续 250 ms 保持在目标转速 `±100 RPM` 内后，先打开 `limitServo`；
再等 500 ms 才让传接电机以 FEED 功率 `1.0` 送球。松开右扳机即先停止送球、关闭挡球舵机并停止射球流程。
左肩键优先于右扳机的送球流程，可立即关挡球并反转传接电机。
射球过程中修改目标 RPM 会重新等待达速并重新执行开挡球、延时、送球的顺序。
左飞轮编码器的 `28 counts/rev` 换算和速度控制参数仍须按实车校准。
飞轮转速模式使用左编码器反馈，对两台电机输出相同的 PI 功率；
Driver Station 显示目标/实测 RPM、误差、实际命令功率、挡球状态与阶段。

## 视觉定位与 Pinpoint 融合测试

入口 `Test / Vision Pose Test`，配置需要 `limelight`、`od` 和 `turretMotor`。
INIT 前将云台对正车头，保持底盘水平并等待 OD 就绪。此入口只观察位姿，不驱动底盘、
不自动转动云台、不发射。Y 切换蓝/红方，pipeline 0/1 分别上传
`src/main/assets/vision/BIOBUZZ_HiveFlat_Blue.fmap` 与 `BIOBUZZ_HiveFlat_Red.fmap`。
项目中保存文件不会自动上传到 Limelight。

两份地图使用同一个场地中心原点；+X 朝裁判/上半场、+Y 朝蓝方、Z 向上。
蓝方 38–41、红方 34–37 位于 +X；其余 Tag 位于 -X。原始文件未改动。
地图的场地尺寸元数据为 17.5482504 × 8.0519016 m，本测试不使用这个尺寸平移坐标，
也不使用 WPI blue/red 角点坐标输出；使用原始 `getBotpose()`。

Limelight 网页相机位姿保持用户实测设置：Forward/Right/Up = 0.117/0/0.278 m，
Roll/Pitch/Yaw = 180/50/0 deg。这是随云台旋转的参考坐标，原点在机器人中心地面投影。
程序不会重复减去这段相机偏移。必须保留 MT1 完整的 Z、pitch、roll，不能将其预先强制归零。
从 botpose 的完整旋转矩阵求 HIVE 翻转角，按场地 Y 轴、X=0、枢轴高度 43.95 in
修正三维位姿；该高度来自 BIOBUZZ V0.9 手册 9.6.1。转换假设底盘水平、同一阵营
HIVE 为刚体，滤除超出 35°、地面高度误差超过 0.15 m 或横向倾斜超过 10° 的结果。
编码器以顺时针为正，因此底盘航向 = 修正后的云台航向 + 拍摄时刻的云台编码器角。

数据流程：新 MT1 帧 → HIVE 倾斜修正 → 历史云台角补偿 → 场地底盘位姿 → 两路估计。
每帧只处理一次，至少需要两枚同阵营 Tag；混入另一阵营/未知 Tag 的帧被拒绝。
使用 Hub 单调时钟减去帧年龄与采集/处理延迟估算拍摄时刻，并保存 1.5 s 的 OD/云台历史。
越出历史范围、快速云台运动、HIVE 翻转尚未稳定的帧不参与融合。时间同步仍是近似值。

- `WpiPinpointPoseEstimator`：WPILib v2025.3.2 PoseEstimator 的 Java 8/FTC 适配，
  沿用 SE(2) 位姿误差、闭式 Kalman 增益以及历史时刻纠正再搬运至当前的做法。
  直接接收 Pinpoint 位姿；只接受按拍摄时间递增的视觉帧。不是未经修改的 FRC 类库。
  源码参考 https://github.com/wpilibsuite/allwpilib/blob/v2025.3.2/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java ，
  许可见 `third-party/WPILib-LICENSE.md`。
- `PinpointEkf`：状态为 X/Y/heading，使用 Pinpoint 局部位移预测、完整 3×3 协方差、
  Joseph 更新、航向环绕和创新门限。视觉在拍摄时刻更新，再重放后续 OD 增量。
  两路各自接收同一原始测量，并列显示，不串联、不相互喂入估计结果。

第一帧通过全部检查的视觉测量建立场地原点关系；此前 `Field anchored=false`，
估计值只是 OD 本地坐标。暂时看不到 Tag 时继续 OD 预测并增长 EKF 不确定度，
通过最后视觉更新年龄和 sigma 判断可靠性。OD 不就绪时清空历史并重新定位。
`VisionLocalization.getEstimatedPosition()` 返回 EKF 结果；WPI 适配结果通过
`getWpiPosition()` 获取。融合定位只由专用的视觉测试入口使用，不修改 Pinpoint 原始位姿。

另有 `Test / Vision OD Cell Chassis Aim Test` 使用融合位姿进行底盘自瞄。
X 切换红/蓝方和对应 Limelight 管线；融合场地 X 坐标决定选择上半场或下半场 CELL。
左摇杆控制平移；松开右摇杆 X 时底盘自动朝向 CELL，推动右摇杆时手动转向覆盖自动转向。
首次定位需要先收到有效 Limelight 位姿，之后融合位姿由 OD 连续推进。
此测试只驱动底盘，不驱动云台、飞轮、传接或挡球机构。

噪声与门限为初始调试值，尚未进行实车数据标定；不能宣称精度已经提高。
数值测试：`python3 tools/localization_tests/run.py`，覆盖正负倾斜、不同朝向和云台角、
180° 环绕、历史越界、延迟重放、视觉离群点与长时间协方差更新。

## 手柄映射

| 手柄 | 控制 |
| --- | --- |
| 1 左摇杆、右摇杆 X | 平移、旋转底盘 |
| 2 左扳机（按住） | 吸球；gate 关闭 |
| 2 左肩键（按住） | 反转清球；gate 关闭 |
| 2 X / Y（按住） | 低 / 高飞轮功率 |
| 2 右扳机（按住） | 若飞轮已启动：先撤挡杆，等待后送球；否则不送球 |
| 2 十字键左 / 右 | 临时手动转动云台，松开恢复自动瞄准 |
| 2 十字键上 / 下 | 高 / 低发射角 |

## 必须在上车前校准

所有硬件名、舵机位置、飞轮起始功率、挡杆延时与云台软限位都在各 subsystem 的 `*Constants.java`。
目前数值只是保守起点，先抬空车、断开飞轮或降低功率确认方向；随后用实物 Pollen 校准。云台刻度读取的是接在 `turretMotor` Encoder Port 的 Through Bore 增量编码器。

该机构使用正好 8 个电机（4 底盘 + 1 intake/transit + 1 云台 + 2 飞轮），达到
BIOBUZZ 的数量上限。还需确保起始为 18×18×18 in，且任何展开状态不超过 18×24×29 in。
详见官方 [赛季资料页](https://ftc-resources.firstinspires.org/ftc/game?hsLang=en) 与
[Robot Construction Rules](https://ftc-resources.firstinspires.org/ftc/game/manual-12)。

Through Bore 安装在云台电机轴，每圈为 8192 counts。电机端 20 齿、云台端 142 齿，
所以云台角度为 `motorTicks × 360 / 8192 × 20 / 142`；`Turret.java` 的
`ENCODER_DIRECTION` 与 `TurretConstants.powerSign` 需要实车确认正方向。
目标相对角归入 [-180,180]，例如 -190 转为 +170。
当前位置到目标的误差不再取最短环绕误差，因此 +179 到 -179 会经过 0 绕回，
不会越过 +180 的线材限位。±180 两种等价目标取靠近当前位置的一种。
闭环采用限功率 PID 控制，初始 kI/kD 为 0；kP、容差和最大功率需要实车标定。

## 独立云台限位测试

程序列表选择 Test / Turret Limit Test，仅初始化名为 `turretMotor` 的电机。
先手动把炮口摆向车头再按 INIT，Through Bore 计数清零，此时云台为 0°。
1 号手柄左摇杆 X 手动转云台，松开停转。向外转到约 +180° 时，
自动暂停手动控制，反向经过 0° 转到 -180°；向外转到约 -180° 时相反。
绕回完成后恢复摇杆控制。回绕使用物理角度目标，不会把两侧等效角度归并成原地不动。
回绕先以满功率 1.0 转 0.215 秒，再切入 PID（P=0.03，功率 0.20 到 0.75）继续到目标。
手动摇杆最大功率为 0.50，实际功率随摇杆幅度变化。
该测试已移除 Rewind Fault 锁定逻辑；按 STOP 仍会立刻让云台电机停转。

总行程只需修改 TurretConstants.travelDegrees：360 表示 ±180，
300 表示 ±150，270 表示 ±135。
这个整圈回绕测试只在行程至少 360° 时触发；较小行程只保留软限位，不执行绕回。
该配置假定左右行程相对车头对称。

## 底盘与云台定向测试

程序列表选择 Test / Drive Turret Heading Hold Test。只初始化四个底盘电机、
`turretMotor` 和 `od`。云台朝车头时按 INIT，车静止后按 START。
1 号手柄左摇杆按场地方向控制前后与横移，右摇杆 X 控制底盘转向。
车身转动后，左摇杆方向仍相对于按 START 时的场地方向；按下左摇杆后，以当前车头方向作为新的场地正前方。
底盘前后和旋转输入已经按实车反馈反向；麦克纳姆底盘的横移项也已按实车反馈反向。
按下左摇杆将当前车头方向设为测试用的 0°，同时保持原云台目标不跳变；
按下右摇杆让云台对准当前车头方向，并把这一刻的车头场地方向设为新的保持目标；松开后仍保持。
程序用 Pinpoint 的航向变化调整云台，默认保持按 START 时的方向，右摇杆按下后保持新方向；
遥测分别显示 OD 航向和重置后的驾驶航向，并显示云台角度、目标角和编码器计数。
Pinpoint 的 Pod 型号与安装偏移仍需在 `PinpointConstants.java` 按实物填写。
若实车航向或云台角度的正方向与当前约定相反，先核对遥测，再调整测试程序的
`HEADING_TO_TURRET_SIGN`。云台接近 ±180° 时会按现有软限位绕回，绕回期间会短暂失去定向。
此测试的云台控制不设近目标最小功率；为提高底盘转动时的跟踪速度，
P 设为 0.030，最大功率设为 0.75。OD CELL 自瞄测试使用相同的云台 P 和最大功率。
遥测会显示云台目标误差，方便实车继续调参。

## OD CELL 云台自瞄测试

程序列表选择 Test / OD Cell Turret Aim Test。它只初始化底盘、云台和 `od`，
左摇杆无头驾驶，右摇杆 X 转底盘，按下左摇杆重置驾驶方向，X 键切换蓝/红方。
测试按 Overturn 的场地坐标：中心 `(0,0)`、`+X` 为裁判/上半场、`-X` 为观众/下半场，
`+Y` 为蓝方；机器人 X 决定上下半场，红蓝方决定对应的 CELL。
四个 CELL 地面瞄点为 X `±17.7 in`、Y `±12.75 in`，进入目标计算前换算成毫米。
云台用 OD 的场地 X/Y 和航向计算指向目标的角度。
选 CELL 和角度换算放在 `subsystems/field/CellTarget.java`；
`commands/OdCellAimCommand.java` 调用云台闭环控制；测试 OpMode 负责绑定命令和遥测。

只用 OD 时必须给出真实的开局场地位姿。编辑 `PinpointConstants.java` 中的
`testStartFieldXmm`、`testStartFieldYmm`、`testStartFieldHeadingDegrees`；默认 `(0,0,0)`
只适用于机器人中心位于场地中心且车头朝 `+X` 的情况。
`PinpointConstants.java` 中 X/Y Pod 偏移已按实测填入 `-101.3 mm` 和 `-154.4 mm`。
X Pod 是沿车头前后滚动的轮子，偏移测它相对机器人中心的左右距离：左正右负；
Y Pod 是沿车身左右滚动的轮子，偏移测它相对机器人中心的前后距离：前正后负。
