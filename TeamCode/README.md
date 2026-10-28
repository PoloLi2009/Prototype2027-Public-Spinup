# BIOBUZZ Spinup — 与 DECODE 一致的项目结构

本项目沿用 `Prototype2026-Public-ChinaChampionship` 的分层方法：

```text
opmodes/teleops/TeleOpBase
        ↓ 绑定默认命令、按键和遥测
commands/DriveCommand, IntakeCommand, TransitCommand, ShooterCommand, TurretCommand
        ↓ 每个命令只申请它实际控制的 subsystem
subsystems/drive | intake | transit | shooter | turret
        ↓ 各自的 *Constants.java
FTC HardwareMap
```

主 TeleOp 为 `BIOBUZZ: Spinup TeleOp`。它继承 `TeleOpBase`，后续可在此结构内增加
自动、视觉瞄准或红蓝场地差异，而无需重写机构初始化。

## Driver Station 硬件名称

| 配置名 | 类型 | 机构 |
| --- | --- | --- |
| `frontLeft`, `backLeft`, `frontRight`, `backRight` | DC Motor | 四电机麦克纳姆底盘 |
| `intakeTransit` | DC Motor | 合并 intake/transit |
| `turret` | DC Motor | 5:2 Yellow Jacket + 同步带云台 |
| `flywheelLeft`, `flywheelRight` | DC Motor（Encoder） | 两个直驱电机，机械联动三飞轮 |
| `shooterPitch` | Servo | 发射角度 |
| `ballGate` | Servo | 入云台挡球 gate/limit |
| `ballRamp` | Servo | 与 limit 杆配合的坡/挡片 |

## 手柄映射

| 手柄 | 控制 |
| --- | --- |
| 1 左摇杆、右摇杆 X | 平移、旋转底盘 |
| 2 左扳机（按住） | 吸球；gate 关闭、坡放下 |
| 2 左肩键（按住） | 反转清球；gate 关闭、坡放下 |
| 2 X / Y（按住） | 低 / 高飞轮功率 |
| 2 右扳机（按住） | 若飞轮已启动：先撤挡杆，等待，再抬坡，等待后送球；否则不送球 |
| 2 十字键左 / 右 | 临时手动转动云台，松开恢复自动瞄准 |
| 2 十字键上 / 下 | 高 / 低发射角 |

## 必须在上车前校准

所有硬件名、舵机位置、飞轮起始功率、挡杆/坡的延时与云台软限位都在各 subsystem 的 `*Constants.java`。
目前数值只是保守起点，先抬空车、断开飞轮或降低功率确认方向；随后用实物 Pollen 校准。云台的刻度读取的是 Yellow Jacket 电机内置编码器，不需要也不会读取外置编码器。

该机构使用正好 8 个电机（4 底盘 + 1 intake/transit + 1 云台 + 2 飞轮），达到
BIOBUZZ 的数量上限。还需确保起始为 18×18×18 in，且任何展开状态不超过 18×24×29 in。
详见官方 [赛季资料页](https://ftc-resources.firstinspires.org/ftc/game?hsLang=en) 与
[Robot Construction Rules](https://ftc-resources.firstinspires.org/ftc/game/manual-12)。

## 云台自动瞄准接口

选目标现在按机器人所在半场：Y <= halfFieldBoundaryYmm 时选择己方 A CELL，
Y > halfFieldBoundaryYmm 时选择己方 B CELL。默认分界 Y=1828.8mm，可在
AimingConstants 修改。视觉只需提供有效位姿，redRaisedCell/blueRaisedCell
保留为兼容字段但不再参与选目标，可以传 UNKNOWN。该逻辑不判断 CELL 是否升起。

坐标原点为蓝方靠观众侧的场地内角，+X 朝红方，+Y 远离观众。
单位毫米；heading 为弧度，朝 +X 为 0，俯视逆时针为正。
机器人坐标就是云台中心坐标。炮口朝车头时必须再按 INIT，使内置编码器归零。
A CELL 靠观众；B CELL 远离观众。这里 A/B 是队伍约定，不是手册命名。

在各红蓝 OpMode 初始化时设置：

```java
AimingConstants.alliance = AimingConstants.Alliance.BLUE;
```

在 AimingConstants 中填写 redAX/redAY、redBX/redBY、blueAX/blueAY、
blueBX/blueBY（对应 CELL 升起状态下开口中心的地面投影）。
当前未填，用 NaN 表示未知，不会使用假坐标驱动。

你的视觉程序将一帧结果提交给 TeleOpBase.vision：

```java
vision.update(new AimingInput.Observation(
        xMm, yMm, headingRadians,
        redRaisedCell, blueRaisedCell, // Cell.A / Cell.B / Cell.UNKNOWN
        captureNanos, true));        // captureNanos 使用 System.nanoTime() 同一时钟
```

摄像头位姿需先由视觉换算到车体中心位姿。坐标、角度必须采用以上约定；
捕获时间必须对应真实数据时间，不能用新时间戳反复提交旧帧。
尚未接入视觉、目标未知、坐标未填或观测超过 500ms 时，自动命令停转。

电机输出轴 145.1 ticks/rev 已含 5.2:1 减速；20:142 传动后，
云台一圈为 1030.21 ticks。TurretConstants.encoderSign 与 powerSign
需要确认使角度与控制输出的正方向都对应逆时针。
目标相对角归入 [-180,180]，例如 -190 转为 +170。
当前位置到目标的误差不再取最短环绕误差，因此 +179 到 -179 会经过 0 绕回，
不会越过 +180 的线材限位。±180 两种等价目标取靠近当前位置的一种。
闭环采用限功率 PID 控制，初始 kI/kD 为 0；kP、容差和最大功率需要实车标定。
距离只计算并显示，尚未用于飞轮转速或 pitch 控制。

## 独立云台限位测试

程序列表选择 Test / Turret Limit Test，仅初始化名为 turret 的电机。
先手动把炮口摆向车头再按 INIT；START 后会闭环保持零度。
1 号手柄十字键左为目标 +30 度，右为 -30 度。
首次按下立即调整一次，持续按住 500ms 后每 250ms 再调整一次；
松开后 PID 保持最后目标。左右同时按不调整。
例如 +170 +30 -> -160，-170 -30 -> +160；绕回经过零点。
初始 PID 为 kP=0.01、kI=0、kD=0，输出限于 0.25。

总行程只需修改 TurretConstants.travelDegrees：360 表示 ±180，
300 表示 ±150，270 表示 ±135，比赛自动瞄准同步使用该配置。
行程小于 360 后会存在无法瞄准的方向：例如 ±135 内不存在 +150 的
等价角，测试程序会提示 UNREACHABLE STEP 并保持上次目标，不跨越机械边界。
该配置假定左右行程相对车头对称。
