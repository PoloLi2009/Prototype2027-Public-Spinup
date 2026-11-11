package org.firstinspires.ftc.teamcode.subsystems.vision;

import java.util.Objects;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drive.ShootAssistConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.ShootAssistDrive;
import org.firstinspires.ftc.teamcode.subsystems.field.CellTarget;

/**
 * Port of Overtune Vision.java's aim/calibration flow:
 * Limelight frame -> flat-HIVE corrected pose -> per-RT-hold vision overwrite of Pinpoint
 * (exactly {@value #REQUIRED_RT_CALIBRATIONS} accepted writes, then Pinpoint only) -> CELL target
 * selection by field X -> coordinate yaw error and horizontal distance.
 *
 * <p>Spinup substitutions (hardware/geometry only): frames come from Spinup {@link LimelightVision}
 * (its pipeline/age/tag checks) and are corrected by Spinup {@link HiveFlatPoseSolver} with the
 * turret angle; CELL points are Spinup {@link CellTarget}; shooter offset is
 * {@link ShootAssistConstants#shooterYawDegrees} (Spinup shoots forward: 0 deg, Overtune 180).</p>
 *
 * <pre>
 * ===================== 视觉上车前必看（2026-11-11 代码审查结论） =====================
 * 代码链路已核对：Limelight MT1 帧 -> HiveFlatPoseSolver（HIVE 倾斜修正 + 云台角）-> 场地中心坐标(mm)
 * -> 写入 Pinpoint（跳变门：>1 in 或 >5° 先拒绝，连续 3 帧才强制）-> 与 CellTarget 同一坐标系算偏航误差和距离。
 * 逻辑上可用；以下几点代码里无法保证，必须上车确认：
 *
 * 1. Robot Configuration 里必须有名为 "limelight" 的 Limelight 3A（VisionConstants.hardwareName）。
 *    solo 在 INIT 就会构造 LimelightVision，没有这个设备会直接报错退出（旧版 solo 不需要它）。
 * 2. Limelight 网页设置必须与 VisionConstants.cameraWebPose 一致（前 0.117 m、离地 0.278 m、
 *    俯仰 50°、偏航 0°、横滚 180° = 相机倒置）。代码不会再套一次这些值，也不会翻转图像。
 * 3. 蓝方 pipeline 0、红方 pipeline 1，各自只识别本方 HIVE 标签（蓝 38–45，红 30–37）。
 *    一帧里只要出现对方标签，整帧丢弃（遥测 Vision 显示 "Unexpected HIVE tag ID(s)"）。
 * 4. 至少同时看到 2 个本方标签（VisionConstants.minimumTagCount）才会产生位姿；
 *    站位太偏时可能一直校准不上，看遥测 Vision / Calibration 两行。
 * 5. INIT 前云台必须手动对正车头：车的朝向 = 相机算出的云台朝向 + 启动锁定角，
 *    没对正会让位置、朝向和瞄准一起偏。
 * 6. 默认蓝方；打红方在 INIT 按 X 切换（遥测 "Alliance / CELL"）。
 * 7. 帧从拍摄到写入 Pinpoint 最多约 200 ms，期间车的运动没有补偿（与 Overtune 相同）。
 *    按住 RT 后车会原地锁位，影响小；开动中校准会有误差。
 * 8. 上车判断视觉是否工作：遥测 "Localization" 出现 "Initialized..." 或
 *    "Vision correction applied"，且 "Shot distance cm" 有数值。
 * ==================================================================================
 * </pre>
 */
public class ShootAssistVision {
    private static final int REQUIRED_RT_CALIBRATIONS = 2;

    public enum HiveState { UNKNOWN, LOW, MIDDLE, HIGH }

    private final LimelightVision limelight;
    private final DoubleSupplier turretClockwiseDegrees;
    private HiveFrameSolver.Observation heldObservation;
    private HiveFrameSolver.Observation currentObservation;
    private String poseStatus = "Waiting for AprilTags";
    private String targetStatus = "Field pose not initialized";
    private Pose2D correctedRobotFieldPose;
    private double frameTimestamp = Double.NaN;
    private boolean hiveSolveSuccessful;
    private boolean correctedPoseAccepted;
    private double hiveAngleDegrees = Double.NaN;
    private boolean hasKnownHiveAngle;
    private double lastKnownHiveAngleDegrees;
    private boolean correctionApplied;
    private String calibrationStatus = "Waiting for corrected vision";
    private boolean calibrationSucceeded;
    private long lastSuccessfulCalibrationNanos;
    private boolean rtAimRequested;
    private int rtCalibrationSuccessCount;
    private boolean rtStopLatched;
    private boolean validAimingTarget;
    private boolean validShootingTarget;
    private double targetHorizontalDistanceInches = Double.NaN;
    private double yawErrorDegrees = Double.NaN;
    private double targetXInches = Double.NaN;
    private double targetYInches = Double.NaN;
    private double desiredHeadingDegrees = Double.NaN;
    private CellTarget selectedTarget;
    private HiveState targetHiveState = HiveState.UNKNOWN;

    public ShootAssistVision(LimelightVision limelight, DoubleSupplier turretClockwiseDegrees) {
        this.limelight = Objects.requireNonNull(limelight);
        this.turretClockwiseDegrees = Objects.requireNonNull(turretClockwiseDegrees);
    }

    /** Overtune Vision.update(): read the camera, keep only a fresh corrected frame. */
    public void update() {
        clearCurrentFrame();
        limelight.periodic();
        LimelightVision.Frame frame = limelight.getNewFrame();
        poseStatus = limelight.getStatus();
        if (frame != null) {
            heldObservation = HiveFrameSolver.solve(frame, turretClockwiseDegrees.getAsDouble());
            if (heldObservation == null) {
                poseStatus = "HIVE geometry / floor / level mismatch";
                return;
            }
        }
        if (heldObservation == null) return;
        // Overtune re-reads the latest result every loop until it is older than the age limit.
        long ageMs = (System.nanoTime() - heldObservation.captureNanos) / 1_000_000L;
        if (ageMs > VisionConstants.maxFrameAgeMs) {
            poseStatus = "Limelight result stale: " + ageMs + " ms";
            heldObservation = null;
            return;
        }
        frameTimestamp = heldObservation.timestamp;
        currentObservation = heldObservation;
        poseStatus = "HIVE corrected pose";
    }

    private void updateHiveCorrection() {
        HiveFrameSolver.Observation correction = currentObservation;
        hiveSolveSuccessful = correction != null;
        correctedRobotFieldPose = correction == null ? null : new Pose2D(DistanceUnit.MM,
                correction.xMm, correction.yMm, AngleUnit.DEGREES, correction.headingDegrees);
        if (!hiveSolveSuccessful) return;
        hiveAngleDegrees = correction.hiveTiltDegrees;
        hasKnownHiveAngle = true;
        lastKnownHiveAngleDegrees = hiveAngleDegrees;
    }

    // Distance-only callers share idle calibration without starting an RT aim session.
    public void updateAimTarget(ShootAssistDrive drive, boolean stopped) {
        updateAimTarget(drive, stopped, false);
    }

    public void updateAimTarget(ShootAssistDrive drive, boolean stopped, boolean aimRequested) {
        updateAimTarget(drive, stopped, aimRequested, System.nanoTime());
    }

    void updateAimTarget(ShootAssistDrive drive, boolean stopped, boolean aimRequested, long nowNanos) {
        if (!aimRequested) {
            rtCalibrationSuccessCount = 0;
            rtStopLatched = false;
        }
        rtAimRequested = aimRequested;
        if (stopped && aimRequested) rtStopLatched = true;
        updateFieldTarget(drive, !stopped && !rtStopLatched, nowNanos);
        if (stopped) calibrationStatus = "Paused: B stop";
    }

    public void updateFieldTarget(ShootAssistDrive drive, boolean correctFromVision) {
        updateFieldTarget(drive, correctFromVision, System.nanoTime());
    }

    void updateFieldTarget(ShootAssistDrive drive, boolean correctFromVision, long nowNanos) {
        updateHiveCorrection();
        updateLocalization(drive, correctFromVision, nowNanos);
        correctedPoseAccepted = correctedRobotFieldPose != null
                && drive.hasAcceptedVisionFrame(frameTimestamp);
        validAimingTarget = false;
        validShootingTarget = false;
        selectedTarget = null;
        targetHiveState = HiveState.UNKNOWN;
        targetXInches = Double.NaN;
        targetYInches = Double.NaN;
        desiredHeadingDegrees = Double.NaN;
        targetHorizontalDistanceInches = Double.NaN;
        yawErrorDegrees = Double.NaN;
        if (rtAimRequested && rtStopLatched) {
            targetStatus = "B stop latched; release RT to resume";
            return;
        }
        if (!ShootAssistConstants.pinpointConfigurationConfirmed) {
            targetStatus = "Pinpoint configuration needs confirmation";
            return;
        }
        if (!drive.isFieldPoseInitialized()) {
            targetStatus = "Waiting for visual field initialization or Pinpoint readiness";
            return;
        }
        Pose2D robotPose = drive.getPose();
        selectedTarget = CellTarget.forRobotX(limelight.isBlueAlliance(), robotPose.getX(DistanceUnit.MM));
        double[] target = getFieldTarget(selectedTarget);
        if (target == null) {
            targetStatus = "CELL field coordinates missing";
            return;
        }
        targetXInches = target[0];
        targetYInches = target[1];
        double dx = targetXInches - robotPose.getX(DistanceUnit.INCH);
        double dy = targetYInches - robotPose.getY(DistanceUnit.INCH);
        targetHorizontalDistanceInches = Math.hypot(dx, dy);
        desiredHeadingDegrees = Math.IEEEremainder(
                Math.toDegrees(Math.atan2(dy, dx)) - ShootAssistConstants.shooterYawDegrees, 360.0);
        yawErrorDegrees = coordinateYawErrorDegrees(
                robotPose, targetXInches, targetYInches, ShootAssistConstants.shooterYawDegrees);
        validAimingTarget = Double.isFinite(yawErrorDegrees)
                && Double.isFinite(targetHorizontalDistanceInches);
        // Retain the last observed HIVE geometry while odometry continues through tag loss.
        targetHiveState = !hasKnownHiveAngle ? HiveState.UNKNOWN
                : Math.abs(lastKnownHiveAngleDegrees) <= ShootAssistConstants.highHiveTiltThresholdDegrees
                ? HiveState.MIDDLE : -Math.sin(Math.toRadians(lastKnownHiveAngleDegrees))
                * targetXInches > 0.0 ? HiveState.HIGH : HiveState.LOW;
        validShootingTarget = validAimingTarget && targetHiveState == HiveState.HIGH;
        targetStatus = validAimingTarget ? "Coordinate aim ready from Pinpoint" : "Invalid aim geometry";
    }

    private void updateLocalization(ShootAssistDrive drive, boolean correctFromVision, long nowNanos) {
        if (!correctFromVision || rtStopLatched) {
            calibrationStatus = rtStopLatched ? "Paused: B stop" : "Vision correction not requested";
            return;
        }
        if (rtCalibrationSuccessCount >= REQUIRED_RT_CALIBRATIONS) return;
        if (correctedRobotFieldPose == null) {
            drive.resetVisionRejectionSequence();
            calibrationStatus = "Waiting for corrected vision: " + poseStatus;
            return;
        }
        boolean applied = drive.updateVisionPose(correctedRobotFieldPose, frameTimestamp, nowNanos);
        correctionApplied |= applied;
        if (applied) {
            calibrationSucceeded = true;
            lastSuccessfulCalibrationNanos = nowNanos;
            if (rtAimRequested) rtCalibrationSuccessCount++;
        }
        calibrationStatus = rtAimRequested && !applied && drive.hasAcceptedVisionFrame(frameTimestamp)
                ? "Waiting for a new Limelight frame" : drive.getLocalizationStatus();
    }

    /** Spinup CellTarget (mm) as Overtune's {x, y} inches; null when missing. */
    private static double[] getFieldTarget(CellTarget target) {
        if (target == null || !Double.isFinite(target.xMm) || !Double.isFinite(target.yMm)) return null;
        return new double[]{DistanceUnit.MM.toInches(target.xMm), DistanceUnit.MM.toInches(target.yMm)};
    }

    /** Overtune VisionMath.coordinateYawErrorDegrees. */
    public static double coordinateYawErrorDegrees(
            Pose2D robotPose, double targetXInches, double targetYInches,
            double shooterYawDegrees
    ) {
        if (robotPose == null || !Double.isFinite(targetXInches)
                || !Double.isFinite(targetYInches) || !Double.isFinite(shooterYawDegrees)) {
            return Double.NaN;
        }
        double dx = targetXInches - robotPose.getX(DistanceUnit.INCH);
        double dy = targetYInches - robotPose.getY(DistanceUnit.INCH);
        double heading = robotPose.getHeading(AngleUnit.RADIANS);
        if (!Double.isFinite(dx) || !Double.isFinite(dy)
                || !Double.isFinite(heading) || Math.hypot(dx, dy) <= 1e-6) {
            return Double.NaN;
        }
        double error = Math.atan2(dy, dx) - Math.toRadians(shooterYawDegrees) - heading;
        return Math.toDegrees(Math.atan2(Math.sin(error), Math.cos(error)));
    }

    public boolean shouldPauseForAimCalibration() {
        return !rtStopLatched && !isAimCalibrationReady() && correctedRobotFieldPose != null;
    }

    public boolean isAimCalibrationReady() {
        return !rtAimRequested || (!rtStopLatched && rtCalibrationSuccessCount >= REQUIRED_RT_CALIBRATIONS);
    }

    public String getRtCalibrationStatus() {
        if (rtStopLatched) return "B stop latched; release RT to resume";
        if (!rtAimRequested) return calibrationStatus;
        String progress = rtCalibrationSuccessCount + "/" + REQUIRED_RT_CALIBRATIONS;
        return isAimCalibrationReady() ? "RT calibration complete (" + progress + "). Using Pinpoint"
                : "RT calibration pending (" + progress + "). " + calibrationStatus;
    }

    public String getCalibrationStatus() { return getRtCalibrationStatus(); }
    public double getSecondsSinceLastCalibration() {
        return calibrationSucceeded
                ? Math.max(0.0, (System.nanoTime() - lastSuccessfulCalibrationNanos) / 1_000_000_000.0)
                : Double.NaN;
    }
    public int getRtCalibrationSuccessCount() { return rtCalibrationSuccessCount; }
    public String getTargetStatus() { return targetStatus; }
    public String getPoseStatus() { return poseStatus; }
    public boolean isHiveSolveSuccessful() { return hiveSolveSuccessful; }
    public boolean isCorrectedPoseAccepted() { return correctedPoseAccepted; }
    public double getHiveAngleDegrees() { return hiveAngleDegrees; }
    public double getTargetXInches() { return targetXInches; }
    public double getTargetYInches() { return targetYInches; }
    public double getDesiredHeadingDegrees() { return desiredHeadingDegrees; }
    public boolean wasCorrectionApplied() { return correctionApplied; }
    public boolean hasValidAimingTarget() { return validAimingTarget; }
    public boolean hasValidShootingTarget() { return validShootingTarget; }
    public double getTargetHorizontalDistanceInches() { return targetHorizontalDistanceInches; }
    public double getYawErrorDegrees() { return yawErrorDegrees; }
    public boolean isBlueAlliance() { return limelight.isBlueAlliance(); }
    public HiveState getTargetHiveState() { return targetHiveState; }
    public Optional<CellTarget> getSelectedTarget() { return Optional.ofNullable(selectedTarget); }
    /** The fresh corrected HIVE observation of this loop, if any (AutoHiveTarget). */
    public Optional<HiveFrameSolver.Observation> getCurrentObservation() {
        return Optional.ofNullable(currentObservation);
    }
    public Optional<Pose2D> getCorrectedRobotFieldPose() {
        return Optional.ofNullable(correctedRobotFieldPose);
    }

    public void stop() {
        limelight.stop();
        clearCurrentFrame();
        heldObservation = null;
        validAimingTarget = false;
        validShootingTarget = false;
    }

    private void clearCurrentFrame() {
        poseStatus = "Waiting for AprilTags";
        correctionApplied = false;
        currentObservation = null;
        correctedRobotFieldPose = null;
        frameTimestamp = Double.NaN;
        hiveSolveSuccessful = false;
        correctedPoseAccepted = false;
        hiveAngleDegrees = Double.NaN;
        validShootingTarget = false;
    }
}
