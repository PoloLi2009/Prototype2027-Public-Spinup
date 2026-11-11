package org.firstinspires.ftc.teamcode.subsystems.drive;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.BlueManualDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;

/**
 * The Overtune Drive API used by ShootAssistCommand / Vision, ported onto Spinup's hardware owners:
 * Spinup {@link Drive} (motors), Spinup {@link Pinpoint} ("od", polled by Solo) and Solo's
 * {@link BlueManualDriveCommand} (Overtune Blue headless mixer + driver-heading reference).
 * Logic of getPose / updateVisionPose / clearVisionPose / hasAcceptedVisionFrame /
 * isFieldPoseInitialized / moveFieldCentric / moveRobot follows Overtune Drive.java.
 */
public class ShootAssistDrive {
    private static final Pose2D UNKNOWN_POSE = new Pose2D(
            DistanceUnit.INCH, Double.NaN, Double.NaN, AngleUnit.RADIANS, Double.NaN);

    private final Drive drive;
    private final Supplier<Pinpoint> pinpointSupplier;
    private final BooleanSupplier odometryReady;
    private final BlueManualDriveCommand blueDrive;
    private boolean fieldPoseInitialized;
    private boolean pinpointReadInterrupted;
    private double lastVisionTimestamp = Double.NaN;
    private double lastProcessedVisionTimestamp = Double.NaN;
    private int rejectedVisionFrameCount;
    private String localizationStatus = "Waiting for Limelight field initialization";

    /** odometryReady is Solo's polled Pinpoint state; pinpointSupplier may return null when OD is disabled. */
    public ShootAssistDrive(Drive drive, Supplier<Pinpoint> pinpointSupplier,
                            BooleanSupplier odometryReady, BlueManualDriveCommand blueDrive) {
        this.drive = Objects.requireNonNull(drive);
        this.pinpointSupplier = Objects.requireNonNull(pinpointSupplier);
        this.odometryReady = Objects.requireNonNull(odometryReady);
        this.blueDrive = Objects.requireNonNull(blueDrive);
    }

    /** Overtune Drive.periodic: a Pinpoint that is not READY drops the vision field registration. */
    public void periodic() {
        if (!isPinpointReady()) {
            if (fieldPoseInitialized || !"Pinpoint not ready".equals(localizationStatus)) {
                clearVisionPose("Pinpoint not ready");
            }
            pinpointReadInterrupted = true;
            return;
        }
        if (!isFinitePose(readPinpointPose())) {
            clearVisionPose("Pinpoint returned invalid X/Y/H");
            pinpointReadInterrupted = true;
            return;
        }
        if (pinpointReadInterrupted) {
            pinpointReadInterrupted = false;
            localizationStatus = fieldPoseInitialized
                    ? "Pinpoint read recovered; continuing field odometry"
                    : "Pinpoint read recovered; waiting for vision calibration";
        }
    }

    // Vision establishes the field frame; Pinpoint tracks movement between vision overwrites.
    public Pose2D getPose() {
        return isFieldPoseInitialized() ? readPinpointPose() : UNKNOWN_POSE;
    }

    public boolean updateVisionPose(Pose2D pose, double frameTimestamp, long nowNanos) {
        if (!ShootAssistConstants.pinpointConfigurationConfirmed) {
            localizationStatus = "Pinpoint configuration needs confirmation";
            resetVisionRejectionSequence();
            return false;
        }
        if (pinpointReadInterrupted) {
            localizationStatus = "Waiting for valid Pinpoint data";
            return false;
        }
        if (!isFinitePose(pose) || !Double.isFinite(frameTimestamp) || frameTimestamp <= 0.0) {
            localizationStatus = "Invalid Limelight field pose";
            resetVisionRejectionSequence();
            return false;
        }
        if (frameTimestamp == lastProcessedVisionTimestamp) return false;
        if (!isPinpointReady()) {
            clearVisionPose("Pinpoint not ready for vision overwrite");
            pinpointReadInterrupted = true;
            return false;
        }
        lastProcessedVisionTimestamp = frameTimestamp;
        if (!Double.isFinite(ShootAssistConstants.maximumVisionPositionDifferenceInches)
                || ShootAssistConstants.maximumVisionPositionDifferenceInches < 0.0
                || !Double.isFinite(ShootAssistConstants.maximumVisionHeadingDifferenceDegrees)
                || ShootAssistConstants.maximumVisionHeadingDifferenceDegrees < 0.0
                || ShootAssistConstants.visionRejectedFramesBeforeOverride < 1) {
            localizationStatus = "Invalid vision correction gates";
            resetVisionRejectionSequence();
            return false;
        }
        Pose2D predictedPose = readPinpointPose();
        boolean initialized = fieldPoseInitialized;
        double positionDifference = isFinitePose(predictedPose) ? Math.hypot(
                pose.getX(DistanceUnit.INCH) - predictedPose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH) - predictedPose.getY(DistanceUnit.INCH)) : Double.NaN;
        double headingDifference = isFinitePose(predictedPose) ? Math.abs(Math.IEEEremainder(
                pose.getHeading(AngleUnit.DEGREES) - predictedPose.getHeading(AngleUnit.DEGREES),
                360.0)) : Double.NaN;
        boolean outsideGate = initialized && (
                positionDifference > ShootAssistConstants.maximumVisionPositionDifferenceInches
                || headingDifference > ShootAssistConstants.maximumVisionHeadingDifferenceDegrees);
        if (outsideGate) {
            rejectedVisionFrameCount++;
            if (rejectedVisionFrameCount < ShootAssistConstants.visionRejectedFramesBeforeOverride) {
                localizationStatus = "Rejected vision jump " + rejectedVisionFrameCount
                        + "/" + ShootAssistConstants.visionRejectedFramesBeforeOverride + "; using Pinpoint";
                return false;
            }
        }
        Pinpoint pinpoint = pinpointSupplier.get();
        double previousHeading = Math.toRadians(pinpoint.getHeadingDegrees());
        try {
            pinpoint.setFieldPose(pose.getX(DistanceUnit.MM), pose.getY(DistanceUnit.MM),
                    pose.getHeading(AngleUnit.DEGREES));
            pinpoint.periodic(); // Overtune readPinpoint("after vision write").
        } catch (RuntimeException failure) {
            clearVisionPose("Pinpoint write failed: " + failure.getClass().getSimpleName());
            pinpointReadInterrupted = true;
            return false;
        }
        if (!pinpoint.isReady()) {
            clearVisionPose("Pinpoint not ready after vision overwrite");
            pinpointReadInterrupted = true;
            return false;
        }
        if (!isFinitePose(readPinpointPose())) {
            clearVisionPose("Pinpoint returned invalid X/Y/H after vision overwrite");
            pinpointReadInterrupted = true;
            return false;
        }
        // A field-heading correction must not change the driver's headless reference.
        if (Double.isFinite(previousHeading)) {
            blueDrive.shiftDriverHeadingReference(Math.IEEEremainder(
                    pose.getHeading(AngleUnit.RADIANS) - previousHeading, 2.0 * Math.PI));
        }
        fieldPoseInitialized = true;
        pinpointReadInterrupted = false;
        lastVisionTimestamp = frameTimestamp;
        resetVisionRejectionSequence();
        localizationStatus = outsideGate ? "Forced vision override"
                : initialized ? "Vision correction applied" : "Initialized from first corrected frame";
        return true;
    }

    public void resetVisionRejectionSequence() {
        rejectedVisionFrameCount = 0;
    }

    public void clearVisionPose(String reason) {
        fieldPoseInitialized = false;
        pinpointReadInterrupted = false;
        lastVisionTimestamp = Double.NaN;
        lastProcessedVisionTimestamp = Double.NaN;
        resetVisionRejectionSequence();
        localizationStatus = reason;
    }

    public boolean hasAcceptedVisionFrame(double timestamp) {
        return isFieldPoseInitialized() && Double.isFinite(timestamp) && timestamp == lastVisionTimestamp;
    }

    public String getLocalizationStatus() { return localizationStatus; }

    public boolean isFieldPoseInitialized() {
        return fieldPoseInitialized && !pinpointReadInterrupted && isPinpointReady();
    }

    public double getTranslationSpeedInchesPerSecond() {
        return isFieldPoseInitialized() ? DistanceUnit.MM.toInches(
                pinpointSupplier.get().getTranslationSpeedMmPerSecond()) : Double.NaN;
    }

    /** Overtune Drive.moveFieldCentric via Solo's Blue headless reference and mixer. */
    public void moveFieldCentric(double forward, double strafe, double turn) {
        blueDrive.moveFieldCentric(forward, strafe, turn);
    }

    /** Overtune Drive.moveRobot (same mixer, Spinup strafingBalance). */
    public void moveRobot(double forward, double strafe, double turn) {
        if (!Double.isFinite(forward) || !Double.isFinite(strafe) || !Double.isFinite(turn)) {
            stop();
            return;
        }
        blueDrive.moveRobot(forward, strafe, turn);
    }

    public void stop() {
        drive.stop();
    }

    private boolean isPinpointReady() {
        Pinpoint pinpoint = pinpointSupplier.get();
        return pinpoint != null && odometryReady.getAsBoolean() && pinpoint.isReady();
    }

    private Pose2D readPinpointPose() {
        Pinpoint pinpoint = pinpointSupplier.get();
        if (pinpoint == null) return UNKNOWN_POSE;
        return new Pose2D(DistanceUnit.MM, pinpoint.getXmm(), pinpoint.getYmm(),
                AngleUnit.DEGREES, pinpoint.getHeadingDegrees());
    }

    private static boolean isFinitePose(Pose2D pose) {
        return pose != null
                && Double.isFinite(pose.getX(DistanceUnit.INCH))
                && Double.isFinite(pose.getY(DistanceUnit.INCH))
                && Double.isFinite(pose.getHeading(AngleUnit.RADIANS));
    }
}
