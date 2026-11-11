package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PositionHoldController {
    private final PIDFController xController = new PIDFController(
            ShootAssistConstants.positionHoldKp,
            ShootAssistConstants.positionHoldKi,
            ShootAssistConstants.positionHoldKd,
            0.0
    );
    private final PIDFController yController = new PIDFController(
            ShootAssistConstants.positionHoldKp,
            ShootAssistConstants.positionHoldKi,
            ShootAssistConstants.positionHoldKd,
            0.0
    );

    private boolean anchored;
    private boolean locked;
    private double anchorX;
    private double anchorY;
    private double forwardPower;
    private double strafePower;
    private double positionErrorInches = Double.NaN;
    private long withinToleranceSinceNanos = -1;

    public void update(Pose2D pose, boolean poseValid,
                       double speedInchesPerSecond) {
        forwardPower = 0.0;
        strafePower = 0.0;
        if (!hasValidConfiguration() || pose == null
                || !poseValid
                || !Double.isFinite(speedInchesPerSecond)) {
            reset();
            return;
        }

        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)) {
            reset();
            return;
        }

        if (!anchored) {
            anchorX = x;
            anchorY = y;
            anchored = true;
            positionErrorInches = 0.0;
            xController.reset();
            yController.reset();
            return;
        }

        positionErrorInches = Math.hypot(anchorX - x, anchorY - y);
        updateLockState(positionErrorInches, speedInchesPerSecond);

        if (positionErrorInches <= ShootAssistConstants.positionLockEnterToleranceInches) {
            xController.reset();
            yController.reset();
            return;
        }

        xController.setPIDF(
                ShootAssistConstants.positionHoldKp,
                ShootAssistConstants.positionHoldKi,
                ShootAssistConstants.positionHoldKd,
                0.0
        );
        yController.setPIDF(
                ShootAssistConstants.positionHoldKp,
                ShootAssistConstants.positionHoldKi,
                ShootAssistConstants.positionHoldKd,
                0.0
        );

        double maxPower = Math.max(0.0, Math.min(1.0, ShootAssistConstants.positionHoldMaxPower));
        double fieldXPower = Range.clip(xController.calculate(x, anchorX), -maxPower, maxPower);
        double fieldYPower = Range.clip(yController.calculate(y, anchorY), -maxPower, maxPower);
        double magnitude = Math.hypot(fieldXPower, fieldYPower);
        if (magnitude > maxPower && magnitude > 0.0) {
            fieldXPower *= maxPower / magnitude;
            fieldYPower *= maxPower / magnitude;
        }

        forwardPower = fieldXPower * Math.cos(heading) + fieldYPower * Math.sin(heading);
        strafePower = fieldXPower * Math.sin(heading) - fieldYPower * Math.cos(heading);
    }

    public void reset() {
        anchored = false;
        locked = false;
        forwardPower = 0.0;
        strafePower = 0.0;
        positionErrorInches = Double.NaN;
        withinToleranceSinceNanos = -1;
        xController.reset();
        yController.reset();
    }

    public boolean isLocked() {
        return locked;
    }

    public double getForwardPower() {
        return forwardPower;
    }

    public double getStrafePower() {
        return strafePower;
    }

    public double getPositionErrorInches() {
        return positionErrorInches;
    }

    private static boolean hasValidConfiguration() {
        return ShootAssistConstants.positionHoldCalibrated
                && Double.isFinite(ShootAssistConstants.positionHoldKp)
                && Double.isFinite(ShootAssistConstants.positionHoldKi)
                && Double.isFinite(ShootAssistConstants.positionHoldKd)
                && Double.isFinite(ShootAssistConstants.positionHoldMaxPower)
                && ShootAssistConstants.positionHoldMaxPower > 0.0
                && ShootAssistConstants.positionHoldMaxPower <= 1.0
                && Double.isFinite(ShootAssistConstants.positionLockEnterToleranceInches)
                && ShootAssistConstants.positionLockEnterToleranceInches >= 0.0
                && Double.isFinite(ShootAssistConstants.positionLockExitToleranceInches)
                && ShootAssistConstants.positionLockExitToleranceInches
                >= ShootAssistConstants.positionLockEnterToleranceInches
                && Double.isFinite(ShootAssistConstants.positionLockMaxSpeedInchesPerSecond)
                && ShootAssistConstants.positionLockMaxSpeedInchesPerSecond >= 0.0
                && Double.isFinite(ShootAssistConstants.positionLockStableTimeSeconds)
                && ShootAssistConstants.positionLockStableTimeSeconds >= 0.0;
    }

    private void updateLockState(double errorInches, double speedInchesPerSecond) {
        if (locked) {
            if (errorInches > ShootAssistConstants.positionLockExitToleranceInches
                    || speedInchesPerSecond > ShootAssistConstants.positionLockMaxSpeedInchesPerSecond) {
                locked = false;
                withinToleranceSinceNanos = -1;
            }
            return;
        }

        if (errorInches > ShootAssistConstants.positionLockEnterToleranceInches
                || speedInchesPerSecond > ShootAssistConstants.positionLockMaxSpeedInchesPerSecond) {
            withinToleranceSinceNanos = -1;
            return;
        }

        long now = System.nanoTime();
        if (withinToleranceSinceNanos < 0) {
            withinToleranceSinceNanos = now;
            return;
        }

        locked = (now - withinToleranceSinceNanos) / 1_000_000_000.0
                >= ShootAssistConstants.positionLockStableTimeSeconds;
    }
}
