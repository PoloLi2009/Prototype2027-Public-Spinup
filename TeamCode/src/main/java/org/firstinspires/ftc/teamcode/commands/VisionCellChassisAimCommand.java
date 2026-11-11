package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.field.CellTarget;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.localization.PinpointConstants;
import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;

public class VisionCellChassisAimCommand extends CommandBase {
    private static final double AIM_POWER_PER_DEGREE = 0.018;
    private static final double MIN_AIM_POWER = 0.05;
    private static final double MAX_AIM_POWER = 0.45;
    private static final double AIM_TOLERANCE_DEGREES = 3.0;
    // Drive's negative forward input moves both sides forward. A positive turn
    // therefore drives the right side forward and increases CCW heading.
    private static final double CCW_TURN_SIGN = 1.0;

    private final Drive drive;
    private final Pinpoint pinpoint;
    private final VisionLocalization localization;
    private final LimelightVision vision;
    private final TurretStartupHoldCommand turretHold;
    private final Gamepad gamepad;
    private final Pose2d startFieldPose = new Pose2d(
            PinpointConstants.testStartFieldXmm / 1000.0,
            PinpointConstants.testStartFieldYmm / 1000.0,
            Rotation2d.fromDegrees(PinpointConstants.testStartFieldHeadingDegrees));
    private boolean lastOriginButton;
    private double originXmm;
    private double originYmm;
    private double driverXmm = Double.NaN;
    private double driverYmm = Double.NaN;
    private double targetHeadingDegrees = Double.NaN;
    private double headingErrorDegrees = Double.NaN;
    private double turnPower;
    private boolean settled;
    private CellTarget target;
    private Pose2d aimPose;
    private String aimStatus = "Manual";

    public VisionCellChassisAimCommand(Drive drive, Pinpoint pinpoint,
                                       VisionLocalization localization, LimelightVision vision,
                                       TurretStartupHoldCommand turretHold, Gamepad gamepad) {
        this.drive = drive;
        this.pinpoint = pinpoint;
        this.localization = localization;
        this.vision = vision;
        this.turretHold = turretHold;
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        boolean originButton = gamepad.left_stick_button;
        boolean resetOrigin = originButton && !lastOriginButton;
        lastOriginButton = originButton;
        double odHeading = pinpoint.getHeadingDegrees();
        if (!pinpoint.isReady() || !Double.isFinite(odHeading)) {
            clearTarget();
            aimPose = null;
            driverXmm = driverYmm = Double.NaN;
            settled = false;
            aimStatus = "Waiting for Pinpoint";
            turnPower = 0.0;
            drive.stop();
            return;
        }

        double odXmm = pinpoint.getXmm();
        double odYmm = pinpoint.getYmm();
        if (resetOrigin && Double.isFinite(odXmm) && Double.isFinite(odYmm)) {
            originXmm = odXmm;
            originYmm = odYmm;
        }
        // Driver coordinates can be re-zeroed independently of field localization.
        driverXmm = odXmm - originXmm;
        driverYmm = odYmm - originYmm;

        updateTarget(odXmm, odYmm, odHeading);
        // Pushing the right stick right requests a clockwise turn.
        turnPower = -CCW_TURN_SIGN * deadband(gamepad.right_stick_x);
        aimStatus = "Manual";
        if (gamepad.right_trigger < 0.5 || gamepad.left_bumper || turnPower != 0.0) {
            settled = false;
        } else if (!turretHold.isOnTarget()) {
            settled = false;
            aimStatus = "Waiting for turret hold";
        } else if (!Double.isFinite(headingErrorDegrees)) {
            settled = false;
            aimStatus = "Waiting for valid CELL bearing";
        } else {
            settled = Math.abs(headingErrorDegrees) <= AIM_TOLERANCE_DEGREES;
            if (!settled) {
                // Keep enough power to turn the chassis when it stops just outside tolerance.
                double power = Range.clip(Math.abs(headingErrorDegrees) * AIM_POWER_PER_DEGREE,
                        MIN_AIM_POWER, MAX_AIM_POWER);
                turnPower = CCW_TURN_SIGN * Math.copySign(power, headingErrorDegrees);
            }
            aimStatus = settled ? "On target" : "Turning";
        }

        drive.moveFieldCentric(-deadband(gamepad.left_stick_y),
                deadband(gamepad.left_stick_x), turnPower, odHeading);
    }

    private void updateTarget(double odXmm, double odYmm, double odHeading) {
        clearTarget();
        aimPose = null;
        if (!Double.isFinite(odXmm) || !Double.isFinite(odYmm)) return;

        // The estimator propagates the last camera correction with Pinpoint on every loop.
        Pose2d fieldPose = localization.hasFieldPose() ? localization.getEstimatedPosition()
                : startFieldPose.plus(new Transform2d(new Translation2d(odXmm / 1000.0, odYmm / 1000.0),
                        Rotation2d.fromDegrees(odHeading)));
        double robotXmm = fieldPose.getX() * 1000.0;
        double robotYmm = fieldPose.getY() * 1000.0;
        double heading = fieldPose.getRotation().getRadians();
        if (!Double.isFinite(robotXmm) || !Double.isFinite(robotYmm)
                || !Double.isFinite(heading)) return;

        aimPose = fieldPose;
        target = CellTarget.forRobotX(vision.isBlueAlliance(), robotXmm);
        double dx = target.xMm - robotXmm;
        double dy = target.yMm - robotYmm;
        if (Math.hypot(dx, dy) < 1.0) return;

        double bearing = Math.atan2(dy, dx);
        targetHeadingDegrees = Math.toDegrees(bearing);
        double difference = bearing - heading;
        headingErrorDegrees = Math.toDegrees(Math.atan2(Math.sin(difference), Math.cos(difference)));
    }

    private void clearTarget() {
        target = null;
        targetHeadingDegrees = Double.NaN;
        headingErrorDegrees = Double.NaN;
    }

    private double deadband(double input) {
        return Math.abs(input) > DriveConstants.joystickDeadband ? input : 0.0;
    }

    public CellTarget getTarget() { return target; }
    public Pose2d getAimPose() { return aimPose; }
    public double getDriverXmm() { return driverXmm; }
    public double getDriverYmm() { return driverYmm; }
    public double getTargetHeadingDegrees() { return targetHeadingDegrees; }
    public double getHeadingErrorDegrees() { return headingErrorDegrees; }
    public double getTurnPower() { return turnPower; }
    public String getAimStatus() { return aimStatus; }
    public boolean isOnTarget() {
        return settled && pinpoint.isReady() && turretHold.isOnTarget()
                && deadband(gamepad.right_stick_x) == 0.0;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
