package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Four-motor hardware owner. Solo uses the Overtune Blue manual-drive command. */
public class Drive extends SubsystemBase {
    public final DcMotor leftFrontMotor;
    public final DcMotor leftBackMotor;
    public final DcMotor rightFrontMotor;
    public final DcMotor rightBackMotor;

    public enum DriveState {
        STOP,
        TELEOP
    }

    private DriveState driveState = DriveState.STOP;

    public Drive(HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, DriveConstants.leftFrontMotorName);
        leftBackMotor = hardwareMap.get(DcMotor.class, DriveConstants.leftBackMotorName);
        rightFrontMotor = hardwareMap.get(DcMotor.class, DriveConstants.rightFrontMotorName);
        rightBackMotor = hardwareMap.get(DcMotor.class, DriveConstants.rightBackMotorName);

        // Change directions here after the first safe wheel-on-blocks test.
        leftFrontMotor.setDirection(DriveConstants.leftFrontMotorDirection);
        leftBackMotor.setDirection(DriveConstants.leftBackMotorDirection);
        rightFrontMotor.setDirection(DriveConstants.rightFrontMotorDirection);
        rightBackMotor.setDirection(DriveConstants.rightBackMotorDirection);

        setBrakeMode(true);
        stop();
    }

    public void moveRobot(double forward, double strafe, double turn) {
        // This chassis moves opposite the requested lateral direction with the usual sign.
        double correctedStrafe = -strafe * DriveConstants.strafingBalance;
        double denominator = Math.max(Math.abs(forward) + Math.abs(correctedStrafe)
                + Math.abs(turn), 1.0);

        setWheelPowers((forward + correctedStrafe + turn) / denominator,
                (forward - correctedStrafe + turn) / denominator,
                (forward - correctedStrafe - turn) / denominator,
                (forward + correctedStrafe - turn) / denominator);
    }

    /** Field forward/right commands, using the tested motor signs of this chassis. */
    public void moveFieldCentric(double fieldForward, double fieldRight, double turn,
                                 double headingDegrees) {
        if (!Double.isFinite(headingDegrees)) {
            stop();
            return;
        }
        double radians = Math.toRadians(headingDegrees);
        double cos = Math.cos(radians);
        double sin = Math.sin(radians);
        double robotForward = fieldForward * cos - fieldRight * sin;
        double robotRight = fieldForward * sin + fieldRight * cos;
        moveRobot(-robotForward, robotRight, turn);
    }

    /** Apply normalized wheel requests without adding another coordinate transform. */
    public void setWheelPowers(double leftFront, double leftBack, double rightFront, double rightBack) {
        leftFrontMotor.setPower(leftFront * DriveConstants.teleOpScale);
        leftBackMotor.setPower(leftBack * DriveConstants.teleOpScale);
        rightFrontMotor.setPower(rightFront * DriveConstants.teleOpScale);
        rightBackMotor.setPower(rightBack * DriveConstants.teleOpScale);
    }

    public void stop() {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    public void setDriveState(DriveState driveState) {
        this.driveState = driveState;
    }

    public DriveState getDriveState() {
        return driveState;
    }

    private void setBrakeMode(boolean brake) {
        DcMotor.ZeroPowerBehavior behavior = brake
                ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        leftFrontMotor.setZeroPowerBehavior(behavior);
        leftBackMotor.setZeroPowerBehavior(behavior);
        rightFrontMotor.setZeroPowerBehavior(behavior);
        rightBackMotor.setZeroPowerBehavior(behavior);
    }
}
