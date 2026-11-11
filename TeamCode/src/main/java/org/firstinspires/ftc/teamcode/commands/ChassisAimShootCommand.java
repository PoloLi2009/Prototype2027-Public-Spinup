package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.utils.FieldTargetConstants;

/**
 * 底盘自瞄 + 射击序列，完全参考 Overtune 的 ShootAssistCommand。
 *
 * <p>Spinup 的云台锁在启动角（{@code TurretConstants.LOCK_STARTUP_ANGLE}），本命令把它当作
 * 不存在：全部瞄准由底盘转向完成，转向时用位置保持锁住 XY，射击门控沿用
 * 「锁定 + 转速到位 + 有目标」才放行。</p>
 *
 * <p>周期内依次做：</p>
 * <ol>
 *   <li>读 Pinpoint 位姿，按 {@link FieldTargetConstants} 取 CELL 口，算 ±180° 内的航向误差；</li>
 *   <li>右摇杆没输入时用 PIDF 自瞄转向；右摇杆一动立刻交还人工；</li>
 *   <li>左摇杆没输入时用位置保持锁 XY 锚点，左摇杆一动就重锚；</li>
 *   <li>按住 RT 才开飞轮；自瞄锁定、转速到位、目标有效后先确认 0.25 s，再开门，
 *       开门 0.5 s 后送料；任一条件丢失立即关门停料，飞轮继续转到松 RT 为止。</li>
 * </ol>
 *
 * <p>云台（{@code Turret}）不在本命令的 requirement 里，也不会被本命令驱动。</p>
 *
 * <p>自瞄转向增益直接照抄自动的航向环：Overtune {@code pedro/Constants.java} 里
 * {@code headingFeedback = Controller.proportional(4.734866377156585)}。Pedro 的航向误差是
 * 弧度、输出是归一化转向功率，和这里的用法一致，所以 Kp 原样搬过来，Kd/Ki 取 0
 * （Pedro 的航向环本身只有比例项 + 刹车前馈）。</p>
 */
public class ChassisAimShootCommand extends CommandBase {
    public enum ShotState { IDLE, SPINNING_UP, GATE_OPEN, FEEDING }

    /* ---- 自瞄转向（Overtune AutoAimController 内联版） ---- */
    /** 照抄 pedro/Constants.java:62 的 headingFeedback 比例增益，单位：功率 / 弧度。 */
    private static final double AIM_KP = 4.734866377156585;
    private static final double AIM_KI = 0.0;
    private static final double AIM_KD = 0.0;
    private static final double AIM_KF = 0.0;
    /** 锁定圈外的静摩擦补偿，对应 Pedro 的 headingStaticFF。 */
    private static final double AIM_KS = 0.06;
    private static final double AIM_MAX_TURN_POWER = 0.65;
    private static final double AIM_TEST_MAX_TURN_POWER = 0.65;
    private static final double AIM_YAW_TO_TURN_SIGN = 1.0;
    private static final double AIM_LOCK_TOLERANCE_DEGREES = 2.0;
    private static final double AIM_UNLOCK_TOLERANCE_DEGREES = 3.0;
    private static final double AIM_LOCK_STABLE_SECONDS = 0.10;
    private static final double AIM_MIN_RANGE_MM = 100.0;
    /** Drive 的正转向是逆时针，也就是场地航向增大的方向。 */
    private static final double CCW_TURN_SIGN = 1.0;

    /* ---- 瞄准期间的位置保持（Overtune PositionHoldController 内联版） ---- */
    private static final boolean POSITION_HOLD_ENABLED = true;
    private static final double HOLD_KP = 0.0047;
    private static final double HOLD_KI = 0.0;
    private static final double HOLD_KD = 0.00028;
    private static final double HOLD_MAX_POWER = 0.60;
    private static final double HOLD_ENTER_TOLERANCE_MM = 19.05;
    private static final double HOLD_EXIT_TOLERANCE_MM = 19.05;
    private static final double HOLD_MAX_SPEED_MM_PER_SECOND = 50.8;
    private static final double HOLD_LOCK_STABLE_SECONDS = 0.15;

    /* ---- 射击序列 ---- */
    private static final double TRIGGER_THRESHOLD = 0.5;
    /** 超过这个距离用高 pitch 档。 */
    private static final double PITCH_HIGH_RANGE_MM = 2500.0;
    /** 锁定后再确认这么久才开门。 */
    private static final double READY_HOLD_SECONDS = 0.25;
    /** 开门到开始送料的间隔，与 SequencedShotCommand 一致。 */
    private static final double GATE_OPEN_DELAY_SECONDS = 0.5;

    private final Drive drive;
    private final Pinpoint pinpoint;
    private final Shooter shooter;
    private final Transit transit;
    private final Intake intake;
    private final DoubleSupplier forwardAxis;
    private final DoubleSupplier strafeAxis;
    private final DoubleSupplier turnAxis;
    private final DoubleSupplier shootRequest;
    private final BooleanSupplier toggleAllianceRequested;

    private final PIDFController aimController =
            new PIDFController(AIM_KP, AIM_KI, AIM_KD, 0.0);
    private final PIDFController holdXController =
            new PIDFController(HOLD_KP, HOLD_KI, HOLD_KD, 0.0);
    private final PIDFController holdYController =
            new PIDFController(HOLD_KP, HOLD_KI, HOLD_KD, 0.0);
    private final ElapsedTime readyTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();

    private FieldTargetConstants.Alliance alliance = FieldTargetConstants.Alliance.BLUE;
    private boolean lastAllianceButton;
    private ShotState shotState = ShotState.IDLE;

    private double[] target;
    private double headingErrorDegrees = Double.NaN;
    private double distanceMm = Double.NaN;
    private boolean aimLocked;
    private boolean manualTurn;
    private boolean manualTranslation;
    private long aimWithinToleranceSinceNanos = -1;

    private boolean holdAnchored;
    private boolean holdLocked;
    private double holdAnchorXmm;
    private double holdAnchorYmm;
    private double holdErrorMm = Double.NaN;
    private double holdForward;
    private double holdRight;
    private long holdWithinToleranceSinceNanos = -1;

    private double lastXmm = Double.NaN;
    private double lastYmm = Double.NaN;
    private long lastSampleNanos;
    private double speedMmPerSecond;
    private String status = "Idle";

    public ChassisAimShootCommand(Drive drive, Pinpoint pinpoint, Shooter shooter,
                                  Transit transit, Intake intake, Gamepad gamepad) {
        this(drive, pinpoint, shooter, transit, intake, () -> gamepad.left_stick_y, () -> gamepad.left_stick_x,
                () -> gamepad.right_stick_x, () -> gamepad.right_trigger, () -> gamepad.x);
    }

    /** Live control inputs; axes retain the original command signs and deadbands. */
    public ChassisAimShootCommand(Drive drive, Pinpoint pinpoint, Shooter shooter,
                                  Transit transit, Intake intake, DoubleSupplier forwardAxis,
                                          DoubleSupplier strafeAxis, DoubleSupplier turnAxis,
                                          DoubleSupplier shootRequest,
                                          BooleanSupplier toggleAllianceRequested) {
        this.drive = drive;
        this.pinpoint = pinpoint;
        this.shooter = shooter;
        this.transit = transit;
        this.intake = intake;
        this.forwardAxis = Objects.requireNonNull(forwardAxis);
        this.strafeAxis = Objects.requireNonNull(strafeAxis);
        this.turnAxis = Objects.requireNonNull(turnAxis);
        this.shootRequest = Objects.requireNonNull(shootRequest);
        this.toggleAllianceRequested = Objects.requireNonNull(toggleAllianceRequested);
        // 云台故意不在这里：它锁在启动角，不参与瞄准。
        addRequirements(drive, shooter, transit, intake);
    }

    @Override
    public void initialize() {
        resetAim();
        resetHold();
        lastAllianceButton = toggleAllianceRequested.getAsBoolean();
        lastXmm = Double.NaN;
        lastYmm = Double.NaN;
        lastSampleNanos = 0;
        speedMmPerSecond = 0.0;
        closeGateAndStopFeed();
        shotState = ShotState.IDLE;
        status = "Idle";
    }

    @Override
    public void execute() {
        updateAllianceEdge();

        if (!pinpoint.isReady() || !pinpoint.isFieldPoseInitialized()) {
            stopDrive();
            clearTarget();
            closeGateAndStopFeed();
            shotState = ShotState.IDLE;
            status = "Waiting for Pinpoint";
            return;
        }

        double xMm = pinpoint.getXmm();
        double yMm = pinpoint.getYmm();
        double headingDegrees = pinpoint.getHeadingDegrees();
        if (!Double.isFinite(xMm) || !Double.isFinite(yMm) || !Double.isFinite(headingDegrees)) {
            stopDrive();
            clearTarget();
            closeGateAndStopFeed();
            shotState = ShotState.IDLE;
            status = "Waiting for a finite pose";
            return;
        }
        updateSpeed(xMm, yMm);

        updateAim(xMm, yMm, headingDegrees);
        boolean hasTarget = Double.isFinite(headingErrorDegrees);

        manualTurn = deadband(turnAxis.getAsDouble()) != 0.0;
        double turnPower;
        if (manualTurn) {
            // 右摇杆往右要顺时针转。
            resetAim();
            turnPower = -CCW_TURN_SIGN * deadband(turnAxis.getAsDouble());
        } else {
            turnPower = calculateTurnPower(hasTarget, headingErrorDegrees);
        }

        double fieldForward = -deadband(forwardAxis.getAsDouble());
        double fieldRight = deadband(strafeAxis.getAsDouble());
        manualTranslation = fieldForward != 0.0 || fieldRight != 0.0;
        boolean shooting = shootRequest.getAsDouble() >= TRIGGER_THRESHOLD;
        if (!shooting || manualTranslation || !hasTarget) {
            // 位置保持只在射击时生效；左摇杆一动就重锚，松手不会回弹。
            resetHold();
        } else {
            updateHold(xMm, yMm, speedMmPerSecond);
            fieldForward = holdForward;
            fieldRight = holdRight;
        }

        drive.moveFieldCentric(fieldForward, fieldRight, turnPower, headingDegrees);

        updateShot(hasTarget);
    }

    /* ------------------------------------------------------------------ 瞄准 */

    private void updateAim(double xMm, double yMm, double headingDegrees) {
        target = FieldTargetConstants.target(alliance, yMm);
        if (target == null) {
            clearTarget();
            return;
        }
        double dx = target[0] - xMm;
        double dy = target[1] - yMm;
        distanceMm = Math.hypot(dx, dy);
        if (distanceMm < AIM_MIN_RANGE_MM) {
            // 站在目标上时方位角没有意义，这一帧不许开火。
            headingErrorDegrees = Double.NaN;
            return;
        }
        double bearing = Math.toDegrees(Math.atan2(dy, dx));
        headingErrorDegrees = normalizeDegrees(bearing - headingDegrees);
    }

    private double calculateTurnPower(boolean hasValidTarget, double yawErrorDegrees) {
        double limit = Math.min(1.0, Math.min(AIM_MAX_TURN_POWER, AIM_TEST_MAX_TURN_POWER));
        if (!hasValidTarget || !Double.isFinite(yawErrorDegrees) || limit <= 0.0) {
            resetAim();
            return 0.0;
        }

        updateAimLock(Math.abs(yawErrorDegrees));

        double yawErrorRadians = Math.toRadians(yawErrorDegrees);
        double output = (aimController.calculate(-yawErrorRadians, 0.0)
                + AIM_KF * yawErrorRadians) * AIM_YAW_TO_TURN_SIGN;
        // 锁定圈外补一点静摩擦，避免停在刚好差一点点的地方。
        if (Math.abs(yawErrorDegrees) > AIM_LOCK_TOLERANCE_DEGREES) {
            output += Math.signum(output) * AIM_KS;
        }
        if (!Double.isFinite(output)) {
            resetAim();
            return 0.0;
        }
        return Range.clip(output, -limit, limit);
    }

    private void updateAimLock(double absoluteYawErrorDegrees) {
        if (aimLocked) {
            if (absoluteYawErrorDegrees > AIM_UNLOCK_TOLERANCE_DEGREES) {
                aimLocked = false;
                aimWithinToleranceSinceNanos = -1;
            }
            return;
        }
        if (absoluteYawErrorDegrees > AIM_LOCK_TOLERANCE_DEGREES) {
            aimWithinToleranceSinceNanos = -1;
            return;
        }
        long now = System.nanoTime();
        if (aimWithinToleranceSinceNanos < 0) {
            aimWithinToleranceSinceNanos = now;
            return;
        }
        aimLocked = (now - aimWithinToleranceSinceNanos) / 1_000_000_000.0
                >= AIM_LOCK_STABLE_SECONDS;
    }

    private void resetAim() {
        aimLocked = false;
        aimWithinToleranceSinceNanos = -1;
        aimController.reset();
    }

    /* -------------------------------------------------------------- 位置保持 */

    private void updateHold(double xMm, double yMm, double currentSpeedMmPerSecond) {
        holdForward = 0.0;
        holdRight = 0.0;
        if (!POSITION_HOLD_ENABLED) {
            return;
        }
        if (!holdAnchored) {
            holdAnchorXmm = xMm;
            holdAnchorYmm = yMm;
            holdAnchored = true;
            holdErrorMm = 0.0;
            holdXController.reset();
            holdYController.reset();
            return;
        }

        holdErrorMm = Math.hypot(holdAnchorXmm - xMm, holdAnchorYmm - yMm);
        updateHoldLock(holdErrorMm, currentSpeedMmPerSecond);
        if (holdErrorMm <= HOLD_ENTER_TOLERANCE_MM) {
            holdXController.reset();
            holdYController.reset();
            return;
        }

        double xPower = Range.clip(holdXController.calculate(xMm, holdAnchorXmm),
                -HOLD_MAX_POWER, HOLD_MAX_POWER);
        double yPower = Range.clip(holdYController.calculate(yMm, holdAnchorYmm),
                -HOLD_MAX_POWER, HOLD_MAX_POWER);
        double magnitude = Math.hypot(xPower, yPower);
        if (magnitude > HOLD_MAX_POWER && magnitude > 0.0) {
            xPower *= HOLD_MAX_POWER / magnitude;
            yPower *= HOLD_MAX_POWER / magnitude;
        }

        holdForward = xPower;
        // Drive 的场地 right 轴就是机器人右侧，在 +Y 朝上的场地系里是 -Y。
        holdRight = -yPower;
    }

    private void updateHoldLock(double errorMm, double currentSpeedMmPerSecond) {
        if (holdLocked) {
            if (errorMm > HOLD_EXIT_TOLERANCE_MM
                    || currentSpeedMmPerSecond > HOLD_MAX_SPEED_MM_PER_SECOND) {
                holdLocked = false;
                holdWithinToleranceSinceNanos = -1;
            }
            return;
        }
        if (errorMm > HOLD_ENTER_TOLERANCE_MM
                || currentSpeedMmPerSecond > HOLD_MAX_SPEED_MM_PER_SECOND) {
            holdWithinToleranceSinceNanos = -1;
            return;
        }
        long now = System.nanoTime();
        if (holdWithinToleranceSinceNanos < 0) {
            holdWithinToleranceSinceNanos = now;
            return;
        }
        holdLocked = (now - holdWithinToleranceSinceNanos) / 1_000_000_000.0
                >= HOLD_LOCK_STABLE_SECONDS;
    }

    private void resetHold() {
        holdAnchored = false;
        holdLocked = false;
        holdErrorMm = Double.NaN;
        holdForward = 0.0;
        holdRight = 0.0;
        holdWithinToleranceSinceNanos = -1;
        holdXController.reset();
        holdYController.reset();
    }

    /* ---------------------------------------------------------------- 射击 */

    private void updateShot(boolean hasTarget) {
        if (shootRequest.getAsDouble() < TRIGGER_THRESHOLD) {
            releaseShot();
            status = "Aiming (RT to shoot)";
            return;
        }

        if (!shooter.isRunning()) {
            shooter.startFlywheelsAtTargetRpm(ShooterConstants.targetRpm);
        }
        if (Double.isFinite(distanceMm)) {
            shooter.setPitchState(distanceMm >= PITCH_HIGH_RANGE_MM
                    ? Shooter.PitchState.HIGH : Shooter.PitchState.LOW);
        }

        boolean ready = hasTarget && shooter.isAtTargetRpm() && aimLocked;
        if (!ready) {
            // 飞轮继续转，但没锁定就绝不送料。
            if (shotState != ShotState.SPINNING_UP) {
                closeGateAndStopFeed();
                shotState = ShotState.SPINNING_UP;
                readyTimer.reset();
            }
            status = !hasTarget ? "No valid target"
                    : !shooter.isAtTargetRpm() ? "Waiting for flywheel"
                    : "Waiting for aim lock";
            return;
        }

        switch (shotState) {
            case SPINNING_UP:
                if (readyTimer.seconds() >= READY_HOLD_SECONDS) {
                    gateTimer.reset();
                    transit.setGateState(Transit.GateState.OPEN);
                    shotState = ShotState.GATE_OPEN;
                }
                status = "Confirming aim";
                break;
            case GATE_OPEN:
                if (gateTimer.seconds() >= GATE_OPEN_DELAY_SECONDS) {
                    shotState = ShotState.FEEDING;
                }
                status = "Gate open";
                break;
            case FEEDING:
                intake.setIntakeState(Intake.IntakeState.FEED);
                status = "Feeding";
                break;
            default:
                // 刚按下 RT：先进入确认阶段。
                closeGateAndStopFeed();
                shotState = ShotState.SPINNING_UP;
                readyTimer.reset();
                status = "Confirming aim";
                break;
        }
    }

    private void releaseShot() {
        if (shotState != ShotState.IDLE) {
            closeGateAndStopFeed();
        }
        shooter.stop();
        shotState = ShotState.IDLE;
    }

    private void closeGateAndStopFeed() {
        transit.setGateState(Transit.GateState.CLOSED);
        intake.stop();
    }

    /* ---------------------------------------------------------------- 工具 */

    private void updateAllianceEdge() {
        if (toggleAllianceRequested.getAsBoolean() && !lastAllianceButton) {
            alliance = alliance == FieldTargetConstants.Alliance.BLUE
                    ? FieldTargetConstants.Alliance.RED
                    : FieldTargetConstants.Alliance.BLUE;
        }
        lastAllianceButton = toggleAllianceRequested.getAsBoolean();
    }

    private void updateSpeed(double xMm, double yMm) {
        long now = System.nanoTime();
        if (Double.isFinite(lastXmm) && Double.isFinite(lastYmm) && lastSampleNanos != 0) {
            double dt = (now - lastSampleNanos) / 1e9;
            if (dt > 1e-4 && dt < 0.5) {
                speedMmPerSecond = Math.hypot(xMm - lastXmm, yMm - lastYmm) / dt;
            }
        }
        lastXmm = xMm;
        lastYmm = yMm;
        lastSampleNanos = now;
    }

    private void stopDrive() {
        drive.stop();
        resetHold();
        resetAim();
    }

    private void clearTarget() {
        target = null;
        headingErrorDegrees = Double.NaN;
        distanceMm = Double.NaN;
    }

    private static double normalizeDegrees(double degrees) {
        double wrapped = degrees % 360.0;
        return wrapped > 180.0 ? wrapped - 360.0
                : wrapped < -180.0 ? wrapped + 360.0 : wrapped;
    }

    private static double deadband(double value) {
        return Math.abs(value) > DriveConstants.joystickDeadband ? value : 0.0;
    }

    @Override
    public boolean isFinished() {
        // 整个 TeleOp 都跑，松 RT 结束一次射击。
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        releaseShot();
        closeGateAndStopFeed();
        drive.stop();
        resetAim();
        resetHold();
        status = interrupted ? "Interrupted" : "Stopped";
    }

    public ShotState getShotState() { return shotState; }
    public double[] getTarget() { return target; }
    public double getHeadingErrorDegrees() { return headingErrorDegrees; }
    public double getDistanceMm() { return distanceMm; }
    public boolean isAimLocked() { return aimLocked; }
    public boolean isManualTurn() { return manualTurn; }
    public boolean isManualTranslation() { return manualTranslation; }
    public boolean isPositionHoldLocked() { return holdLocked; }
    public double getPositionErrorMm() { return holdErrorMm; }
    public double getSpeedMmPerSecond() { return speedMmPerSecond; }
    public void setAlliance(FieldTargetConstants.Alliance alliance) {
        this.alliance = Objects.requireNonNull(alliance);
    }

    public FieldTargetConstants.Alliance getAlliance() { return alliance; }
    public String getStatus() { return status; }
}
