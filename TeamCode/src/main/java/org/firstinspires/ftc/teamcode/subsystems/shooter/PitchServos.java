package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/** Independent two-servo trim fixture; both SDK directions remain FORWARD. */
public class PitchServos extends SubsystemBase {
    private final Servo left;
    private final Servo right;
    private double leftPosition = 0.50;
    private double rightPosition = 0.50;

    public PitchServos(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, ShooterConstants.leftPitchServoName);
        right = hardwareMap.get(Servo.class, ShooterConstants.rightPitchServoName);
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.FORWARD);
        periodic();
    }

    public void adjust(double leftDelta, double rightDelta) {
        leftPosition = Range.clip(leftPosition + leftDelta, 0.0, 1.0);
        rightPosition = Range.clip(rightPosition + rightDelta, 0.0, 1.0);
    }

    public double getLeftPosition() { return leftPosition; }
    public double getRightPosition() { return rightPosition; }
    public double getLeftSdkPosition() { return left.getPosition(); }
    public double getRightSdkPosition() { return right.getPosition(); }

    @Override
    public void periodic() {
        left.setPosition(leftPosition);
        right.setPosition(rightPosition);
    }
}
