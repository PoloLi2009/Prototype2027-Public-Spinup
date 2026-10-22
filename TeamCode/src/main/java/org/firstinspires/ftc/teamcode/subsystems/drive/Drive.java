package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Four-motor mecanum drive subsystem. It is intentionally robot-centric for spinup. */
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
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setBrakeMode(true);
        stop();
    }

    public void moveRobot(double forward, double strafe, double turn) {
        double correctedStrafe = strafe * DriveConstants.strafingBalance;
        double denominator = Math.max(Math.abs(forward) + Math.abs(correctedStrafe)
                + Math.abs(turn), 1.0);

        leftFrontMotor.setPower((forward + correctedStrafe + turn) / denominator
                * DriveConstants.teleOpScale);
        leftBackMotor.setPower((forward - correctedStrafe + turn) / denominator
                * DriveConstants.teleOpScale);
        rightFrontMotor.setPower((forward - correctedStrafe - turn) / denominator
                * DriveConstants.teleOpScale);
        rightBackMotor.setPower((forward + correctedStrafe - turn) / denominator
                * DriveConstants.teleOpScale);
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
