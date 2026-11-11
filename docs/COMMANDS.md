# Spinup 共用 Command（2026-11-11）

手动和自动使用同一批动作类。原 Gamepad / GamepadEx 构造方法保留，内部转成实时 Supplier 输入；自动可直接给定常量或状态条件，不需要创建假手柄。原重构保留机构参数与各程序算法；后续用户明确要求的 Solo 900 TPS 默认目标和Overtune Blue 同款驾驶逻辑及 Pedro＋Foresight 自动导航见下文。

## 动作入口

| 功能 | 共用入口 | 完成条件 / 占用资源 |
| --- | --- | --- |
| 闭环预转 | `FlywheelRpmCommand(shooter, rpm)`，也支持 DoubleSupplier | 持续运行；Shooter |
| 达速射击 | `ShootCommand(intake, shooter, transit, rpm, aimReady, feedMs, timeoutMs)` | 连续送料达到 feedMs 或整个命令达到 timeoutMs；Intake + Shooter + Transit |
| 手动射击操作 | `SequencedShotCommand` | 原按键适配器，内部执行同一个 ShootCommand，保留预转/反转/调速/后板操作 |
| 吸球 | `IntakeCommand` | 持续；Intake + Transit，初始化关门，结束停止吸球 |
| 反转排堵 | `ReverseIntakeCommand` | 持续；Intake + Transit，支持原速和 FAST 档 |
| 单独驱动吸球/送料电机 | `RunIntakeCommand(intake, state)` | 持续；Intake，结束停止；送料的放行条件由外层射击命令负责 |
| 原有送料策略 | `TransitCommand` | 持续；Intake + Transit，保留 isRunning 和原开门等待策略 |
| 挡板 | `GateCommand(transit, state)` | 持续；Transit，结束关门 |
| 后板档位 / 微调 | `SetPitchCommand(shooter, PitchState)` / `(shooter, adjustment)` | 一次设置；Shooter；完成后保持位置 |
| 原开环预设 | `ShooterCommand(shooter, speedSupplier, pitchSupplier)` | 持续；Shooter；保留原 HIGH/LOW 开环行为，RPM 闭环使用 FlywheelRpmCommand |
| 机器人坐标底盘 | `DriveCommand(drive, forward, strafe, turn)` | 持续；Drive；三个 DoubleSupplier 使用 moveRobot 的输入方向 |
| 场地坐标底盘 | `FieldCentricDriveCommand` 的 Supplier 重载 | 持续；Drive；保留定位有效条件、回零与原手柄轴符号 |
| Solo 底盘与转向瞄准叠加 | `BlueManualDriveCommand` | 持续；Drive；Overtune Blue 摇杆、相对航向、回零、四轮混控；Solo只由手柄控制转向，不调用自瞄 |
| 旧测试底盘适配器 | `DriveWithAimCommand` | 保留旧测试的轴映射；Solo 不再使用 |
| 跟路径 | `FollowPathCommand(pedroDrive, paths...)` | 全部路径完成或定位丢失；PedroDrive；每段 5 秒超时后停止该段并推进；结束/取消停止，重新调度从第一段开始 |
| 底盘视觉自瞄 | `VisionCellChassisAimCommand` 的 Supplier 重载 | 持续；Drive；`isOnTarget()` 可作为射击条件；`updateTurn` 只计算不写底盘 |
| 原 OD 底盘自瞄 | `ManualChassisAimCommand` 的 Supplier 重载 | 持续；Drive；`setAlliance` 直接选边 |
| 原瞄准+位置保持+射击策略 | `ChassisAimShootCommand` 的 Supplier 重载 | 持续；Drive + Shooter + Transit + Intake；保留它自身原有策略，不替换成 Solo 算法 |
| 云台转动 | `TurretCommand(turret, powerSupplier)` | 持续；Turret；结束停止 |
| 云台初始角保持 | `TurretStartupHoldCommand` | 持续；Turret；initialize 捕获当前角度 |
| 云台场地航向保持 | `TurretHeadingHoldCommand` 的 Supplier 重载 | 持续；Turret；输入为重新捕获车头朝向的请求 |
| OD 云台瞄准 | `OdCellAimCommand` 的 Supplier 重载 | 持续；Turret；`setBlueAlliance` 直接选边 |
| 云台限位回绕 | `TurretLimitCommand` 的 DoubleSupplier 重载 | 持续；Turret；保留原限位和回绕时序 |
| 定位融合与选边 | `VisionReadCommand` 的 Supplier 重载 | 持续；Vision + VisionLocalization；无手柄也可融合定位 |
| 设置场地起点 / 重置 | `SetFieldPoseCommand(pinpoint, xMm, yMm, headingDeg)` / `(pinpoint)` | 一次设置；Pinpoint；放在运动前执行 |
| 独立测试动作 | `TransitShooterManualCommand`、`TransitMotorTestCommand`、`TurretEncoderTestCommand`、`PitchServoTrimCommand`、`LimitServoTrimCommand` | 提供不依赖手柄的 Supplier 重载，原测试入口仍用旧构造方法 |
| 其他已有独立动作 | `ShooterMotorTestCommand`、`DashboardTuningCommand` | 原本就不依赖手柄，继续共用 |

所有 Supplier 都在运行时读取。保留手柄轴语义的构造重载，其前进/横移/转向符号以该类注释为准；不要把不同坐标系的数值直接混用。用于边沿操作的 BooleanSupplier 需要 false→true 才触发再次调整。

## 调用和生命周期

自动通常通过 FTCLib `CommandScheduler` 调度，并在 OpMode 活动循环中调用 `run()`。它会更新已注册 Subsystem 的 `periodic()`。持续命令可用 `.withTimeout(milliseconds)` 或作为 `ParallelDeadlineGroup` 的伴随动作；结束时清理输出状态。OpMode 的 finally 仍应停止机构并应用停止输出。

例如（时间与转速仅演示接口，实际值需标定）：

```java
Command sequence = new SequentialCommandGroup(
        new FollowPathCommand(pedroDrive, pathToShot),
        new ShootCommand(intake, shooter, transit,
                targetRpm, () -> true, feedMs, totalShotTimeoutMs));
sequence.schedule();
// 活动循环：CommandScheduler.getInstance().run();
// 停止：取消本入口拥有的命令，并停止硬件输出。
```

示例的 `() -> true` 表示只按达速放行，不包含自瞄。现有 SpinupAutoFormal 仍只执行原八段路线，没有增加射击点。自动如需增加自瞄，须以同一个 PedroDrive 实现和声明底盘资源；不要在 PedroDrive 旁另建手动 Drive/Pinpoint，否则会重复控制同一组电机和 OD。ShootCommand 不占用或写底盘。

射击达速判定为**实际转速处于当前目标的 ±6% 内**，沿用 Overtune 的相对误差语义；默认900TPS是Solo目标，不再是独立的送料下限。900TPS目标的窗口为846～954TPS，下调目标后窗口同步下调。目标仍按28ticks/rev在RPM/TPS间转换。ShootCommand保留唯一250ms稳定计时及500ms开门等待，不额外叠加Overtune的50ms稳定计时。掉速、超速或失去瞄准会关门、停送料并重新判定；有限命令feedMs连续计时，timeoutMs从initialize计时。

Spinup现使用共用PIDF：`power = kF*targetRpm + kP*errorRpm + integral - kD*filteredMeasuredRpmSlope`，F只加一次；2026-11-11用户批准的实测默认P=.00020000、I=.00060000、D=0、F=.00029167（RPM单位），右编码器闭环、两电机同功率；Overtune使用FTCLib PIDF，单位TPS，P=0.004、I=D=0、F=0.00063。两台车的增益不能直接按同一个数值复制。沿用Spinup控制器及本车实测增益；将循环>=200ms时直接丢弃积分改为按min(dt,0.2)做有界积分，并把积分状态范围从±0.5改为±1，输出仍限0～1，饱和时只允许纠偏方向的积分。相同目标重复调用不清积分，改目标/停止会清零。Solo新增F/P/I/循环周期遥测供实测排查，不能据软件测试断言实车已经能达到所有目标。

手动适配器对内部子命令直接调用生命周期，子命令不另行调度；外层命令声明全部资源。SetPitchCommand 是一次动作，手动边沿触发时调用 initialize。Solo 继续使用原来的传感器→控制→输出顺序及显式 periodic，避免改变急停和定位故障降级行为。

## 手动底盘：Overtune Blue

Solo 的 `BlueManualDriveCommand` 使用 GamepadEx 的 LeftY、LeftX、RightX，保留 Spinup 的摇杆死区、横移校正和输出倍率。START 捕获驾驶航向，L3 按下重新捕获；驾驶相对航向为 OD 航向减参考航向。随后完全按 Overtune Blue 的坐标变换、四轮混控及 L1 归一化直接写电机，不使用 FTCLib MecanumDrive 类，也不创建 Pedro follower。

2026-11-11实车反馈六方向全部反向后，DriveConstants已把四轮方向统一反转为左FORWARD、右REVERSE，Blue混控和OD符号保持。Pedro配置复用这组硬件方向，标定前需低速验证；独立旧测试程序的轴映射未重写，但它们也会使用更新的硬件方向。Y 不切换驾驶坐标，RB 不再降低底盘速度。

Solo已按最新要求移除自瞄、视觉初始化、选边和R3瞄准切换；云台初始角保持与B急停/回中保护保留，云台是否到位不再作为送料条件。OD 不可用时退回机器人坐标；可用性变化仍需摇杆回中。Vision Cell Chassis Aim Test 等独立测试继续用原逻辑，不随 Solo 改方向。

## 自动底盘：Pedro＋Foresight

`PedroDrive` 独占四轮与 Pinpoint，使用与 Overtune 相同的 Pedro core/revhub 3.0.2、Foresight 和 follower.follow/isBusy 生命周期。SpinupAutoFormal 通过 FollowPathCommand 执行原八段路线；每段 5 秒超时停止该段并进入下一段，INIT 不计时；仍保留 28.5 秒整段上限，所以八段不保证各获得完整 5 秒。未重新添加原 45% 总功率或 30% 转向限制，具体输出由 Foresight 控制；原 AutoFormalFollower 已移除。

`pedro/Constants.java` 的 `foresightConfig` **当前为 null，必须实车标定后才能运行自动**。自动入口会在创建电机/OD/云台之前提示并退出；Solo 不受此限制。电机名称/方向取 Spinup DriveConstants，OD 名称、偏移、pod 类型和编码器方向取 Spinup PinpointConstants；没有借用 Overtune 的实测参数。

新增 `pedro/Tuning.java` 的 @Tuner 入口和从 Overtune 原样移植的 ForesightTuner，依赖 tuning 1.0.1。装车后进入 AutoTune，选择 Foresight Tuner，按页面进行前进、横移、旋转和制动测量；它不依赖已经存在的 foresightConfig。完成后把 Java 导出配置放入本工程 `pedro/Constants.java` 对应字段，再编译上传。路径单位为英寸，OD 安装偏移以毫米配置。

退出时逐个取消本入口的路径和云台命令，随后停止四轮、关闭云台并重置 scheduler；避免 FTCLib 2.1.1 同时运行多个命令时 cancelAll 的集合修改异常。

## 验证

```sh
python3 tools/solo_tests/run.py
python3 tools/command_tests/run.py
python3 tools/command_tests/check_shooter.py
python3 tools/command_tests/check_auto.py
python3 tools/command_tests/check_manual_wheels.py
```

Solo 测试执行真实入口配假硬件，共 2060 项断言，包含 2000 组输入与本地 Overtune Blue 实际源码提取的算法逐轮比较，以及回零、自瞄、射击、900 TPS 默认目标、急停和 OD 故障处理。源对照要求相邻 Overtune 工程可用。

command_tests 使用真实 FTCLib scheduler 和 Pedro 路径配硬件替身，验证射击生命周期、资源冲突及逐段 5 秒边界。check_shooter验证生产Shooter的目标相对窗口、下调目标、无效输入、慢循环积分、饱和防积分累积和超速退积分；check_manual_wheels穿过真实Blue命令和Drive验证六轴混控输出、四轮硬件方向反转及停止。check_auto 执行真实 AutoFormal 入口，覆盖未标定不创建硬件、INIT 取消、OD 未准备、八段结束、28.5 秒整段限制和异常清理。

这些检查和 Android 构建不代替实车标定/试跑。最初重构时的 9000 周期回放只证明当时机构逻辑；后续明确变更的 Solo 驾驶方式不再声称与旧 Solo 等价。

## Shooter PID AutoTune：自动试参并复测

1. 上传程序，在DS选择 **Shooter PID AutoTune**（Test组）。先清空球，固定好机器人，测试会自动转射手。
2. Dashboard 的 `ShooterPidAutoTune` 可设 `targetTps`（默认900，范围200～2000且不超过最大RPM折算值）及 `maxPower`（默认0.85）。参数在按A时快照；不会在运行中改变本轮试验范围。
3. START后释放再按A开始；B或STOP立即停止。只控制射手，吸球保持停机、送料挡板保持关闭；不初始化底盘、视觉、云台和后板。
4. 测试基线一次，然后依次搜索F/P/I/D，各三个候选，共13组，再独立复测选中组。每组跑 `0.8×目标 → 目标 → 0.8×目标`，各3秒；组间停机滑行0.5～4秒。含非零D，D=0仅作为对照。预计2～3分钟，整体210秒上限。
5. Dashboard绘图字段：`targetTps`、`actualTps`、`output`、`P`、`I`、`D`、`F`。DS显示候选参数、进度、推荐值与日志绝对路径。

使用真实 `Shooter`、`ShooterPidAutoTuneCommand` 和纯计算 `tuning/ShooterPidSearch`，不复制另一套测试PID。D对测量RPM的斜率做一阶滤波（默认0.05秒）；切目标不制造D尖峰。前馈独立参数是 `ShooterConstants.flywheelKfPowerPerRpm`，不再通过修改maxRpm间接改变F。测试覆盖是每个Shooter实例的临时参数，结束/异常后清除，公共比赛参数不自动改写。

评分考虑全程相对误差、各阶段最后1秒的稳态误差、波动及输出饱和比例。复测要求每阶段平均绝对误差≤6%、标准差≤3%、最大瞬时稳态误差≤12%，总饱和比例<80%；不合格不推荐。它是在有限候选内寻找较好的参数，不承诺全局最优。负载、电压和温度变化仍会影响参数，空载推荐须再做带球验证。

超过最高目标的140%、有功率而连续1.5秒编码器速度<20TPS、控制循环>250ms、无效读数/参数或达到总时限会停止并说明原因。控制输出还受本轮maxPower限制。

文件保存到机器人应用外部文件目录下 `shooter-pid-autotune/run-时间戳/`（以DS显示的完整路径为准）：
- `samples.csv`：每循环的目标/实际转速、功率、P/I/D/F、周期。
- `results.csv`：每组参数、误差、波动、峰值、评分和合格状态。
- `recommendation.txt`：复测结论；合格时包含可粘贴的四条ShooterConstants赋值。单位为P功率/RPM、I功率/(RPM·秒)、D功率/(RPM/秒)、F功率/RPM；不能直接填入Overtune的TPS参数位置。

验证：`python3 tools/command_tests/check_pid_autotune.py`检查完整13+1组、四类参数均试验、复测失败拒绝推荐、超速/无编码器/慢循环/时限/取消停机、无送料与公共参数不变；`check_shooter.py`验证真实D反应、无目标阶跃D冲击、F无重复叠加、临时限功率及清理。假设备测试不代替实车调参。

### 2026-11-11 实测推荐已应用

用户批准第14轮复测得到的 `P=.00020000, I=.00060000, D=0, F=.00029167`。
统一保存在ShooterConstants，Solo、预转、共用手动/自动ShootCommand、底盘自瞄射击和闭环射手测试都由同一个Shooter.periodic读取，不维护各自PIDF副本。AutoTune下次从这组参数开始，试验覆盖结束后恢复这组默认参数。原开环档位/全功率电机测试不使用PIDF，通用DashTuner的裸电机调参器是独立实验工具，不把RPM增益直接填入其SDK/TPS控制器。

### 最新 Solo：速度达标即放行，无自瞄

Solo不再创建Limelight/VisionLocalization/VisionCellChassisAimCommand；X选边/R3自瞄切换移除。Pinpoint继续用于Blue无头驾驶，L3回零不变。Solo的Shooter实例单独使用±5%速度窗口，共用默认±6%不影响其他程序。

按住RT且目标>0，转速进入目标±5%立即开挡板（Solo的readyHoldMs=0），挡板打开500ms后以原送料功率送料。900TPS目标的闭区间为855～945TPS；转速离开范围时立即关门、停送料，再达速重新开门计时。A仍只预转，LB反转/B急停/RT释放仍优先停止射击；无需视觉定位、车头朝向或云台稳定。其他射击入口仍用默认250ms达速稳定等待。本轮PIDF实测系数不变。

验证覆盖±5%上下边界、无视觉/云台未稳定/手动转向仍可开门、取消250ms等待、保留500ms挡板行程、掉速停料、R3/X无自瞄副作用、RT释放和LB反转优先级。

### 最新 Solo 达速窗口改为固定 ±30 TPS

覆盖上节的±5%：Solo以 `setReadyAbsoluteToleranceTicksPerSecond(30)` 设置实例范围。900TPS目标对应870～930TPS；600TPS对应570～630TPS。转速变化后宽度仍为30TPS，不随目标百分比缩放。900TPS时这比原±45TPS更严格。其他程序默认相对容差不变；无自瞄、RT达速立即开门、500ms后送料、掉速停料和PIDF系数都保持。

## Overtune ShootAssist / AutoShoot 移植（2026-11-11 晚，替代上面的“Solo 无自瞄”）

`solo` 现在按 Overtune TeleOpBase 的流程运行：RT 运行 `ShootAssistCommand`（底盘自瞄 + 每次按住 RT 两次视觉校准 + 定点保持 + 达速/对准/俯仰门控 → 开挡板 → 反转 0.1 s → 持续送料）。云台锁在启动角度；Spinup 车头出球，`ShootAssistConstants.shooterYawDegrees = 0`（Overtune 为 180）。

- 逻辑文件（除参数外与 Overtune 一致）：`commands/ShootAssistCommand.java`、`commands/AutoShootCommand.java`、`subsystems/drive/AutoAimController.java`、`PositionHoldController.java`。
- Spinup 硬件适配：`ShootAssistDrive`（Pinpoint+Blue 混控）、`ShootAssistShooter`（右编码器、固定目标 TPS）、`ShootAssistTransit`（limitServo + 单电机）、`ShootAssistIntake`、`ShootAssistVision`（Limelight → HiveFlatPoseSolver → Pinpoint）、`AutoHiveTarget`。
- 参数集中在 `subsystems/drive/ShootAssistConstants.java`，注释标明取自 Spinup 还是沿用 OT 默认。
- 按键：RT 瞄准射击，LT 吸球，LB 反转，RB 排堵，B 停止，A 手动预转，X 切红蓝，D-pad 左右调转速，手柄 2 D-pad 上下切俯仰，L3 重设无头方向，INIT 时按住 Back 关闭 Pinpoint。
- `AutoShootCommand` 已移植，未被任何自动调用。
- 验证：`python3 tools/shoot_assist_tests/run.py`。`tools/solo_tests` 针对旧 Solo（SequencedShotCommand 版）编写，已不适用。
- 自适应射击（2026-11-11 20:20）：`ShootAssistShooter.setDistanceTarget` 照抄 Overtune 距离表算法（近/远各 5 行插值、超出端点钳位、RT 时俯仰跟随、预转不动俯仰、正值覆盖只换转速）。距离来自 Limelight 校准后的 Pinpoint 底盘位置。表在 `ShootAssistConstants.near/farShotPoints`，Spinup 尚无实测数据，目前每行都是 900 TPS / 俯仰 0.50（行为与之前相同），停车后按遥测 "Shot distance cm" 填表。D-pad 左右设手动转速覆盖，Y 恢复查表。
