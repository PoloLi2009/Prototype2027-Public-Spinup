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
| 2 右扳机（按住） | 若飞轮已启动：抬坡、开 gate、送球；否则不送球 |
| 2 十字键左 / 右 | 手动转动云台 |
| 2 十字键上 / 下 | 高 / 低发射角 |

## 必须在上车前校准

所有硬件名、舵机位置、飞轮起始功率与云台软限位都在各 subsystem 的 `*Constants.java`。
目前数值只是保守起点，先抬空车、断开飞轮或降低功率确认方向；随后用实物 Pollen 校准。

该机构使用正好 8 个电机（4 底盘 + 1 intake/transit + 1 云台 + 2 飞轮），达到
BIOBUZZ 的数量上限。还需确保起始为 18×18×18 in，且任何展开状态不超过 18×24×29 in。
详见官方 [赛季资料页](https://ftc-resources.firstinspires.org/ftc/game?hsLang=en) 与
[Robot Construction Rules](https://ftc-resources.firstinspires.org/ftc/game/manual-12)。
