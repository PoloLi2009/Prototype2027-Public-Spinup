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
| `leftFrontMotor`, `rightFrontMotor`, `leftBackMotor`, `rightBackMotor` | DC Motor | 四电机麦克纳姆底盘 |
| `intakeTransit` | DC Motor | 合并 intake/transit |
| `turretMotor` | DC Motor | 5:2 Yellow Jacket + 同步带云台 |
| `od` | goBILDA Pinpoint | 底盘位置与航向 |
| `Left Flywheel Motor`, `Right Flywheel Motor` | DC Motor（仅右侧 Encoder） | 两个直驱电机，机械联动三飞轮 |
| `Left Pitch Servo`, `Right Pitch Servo` | Servo | 双舵机反向联动的发射角度 |
| `ballGate` | Servo | 入云台挡球 gate/limit |

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
回绕先以满功率 1.0 转 0.215 秒，再切入 PID（P=0.02，功率 0.20 到 0.50）继续到目标。
手动摇杆最大功率为 0.50，实际功率随摇杆幅度变化。
该测试已移除 Rewind Fault 锁定逻辑；按 STOP 仍会立刻让云台电机停转。

总行程只需修改 TurretConstants.travelDegrees：360 表示 ±180，
300 表示 ±150，270 表示 ±135。
这个整圈回绕测试只在行程至少 360° 时触发；较小行程只保留软限位，不执行绕回。
该配置假定左右行程相对车头对称。

## 底盘与云台定向测试

程序列表选择 Test / Drive Turret Heading Hold Test。只初始化四个底盘电机、
`turretMotor` 和 `od`。云台朝车头时按 INIT，车静止后按 START。
1 号手柄左摇杆控制前后与横移，右摇杆 X 控制底盘转向。
底盘前后和旋转输入已经按实车反馈反向；麦克纳姆底盘的横移项也已按实车反馈反向。
按下左摇杆将当前车头方向设为测试用的 0°，同时保持原云台目标不跳变；
按下右摇杆将云台当前指向设为新的保持目标。
程序用 Pinpoint 的航向变化调整云台，默认保持按 START 时的方向，右摇杆按下后保持新方向；
遥测显示底盘航向、云台角度、目标角和编码器计数。
Pinpoint 的 Pod 型号与安装偏移仍需在 `PinpointConstants.java` 按实物填写。
若实车航向或云台角度的正方向与当前约定相反，先核对遥测，再调整测试程序的
`HEADING_TO_TURRET_SIGN`。云台接近 ±180° 时会按现有软限位绕回，绕回期间会短暂失去定向。
此测试的云台控制已取消近目标最小功率，并将 P 从 0.02 调为 0.012、最大功率调为 0.40，
用于减少到点后反复摆动；实车仍需观察是否能克服静摩擦。
