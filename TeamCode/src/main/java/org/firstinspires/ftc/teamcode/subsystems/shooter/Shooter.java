package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Two motors mechanically coupled by gears and belts to the three flywheels. */
public class Shooter extends SubsystemBase {
    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
    private final Servo leftPitchServo;
    private final Servo rightPitchServo;
    private ShooterState shooterState = ShooterState.STOP;
    private PitchState pitchState = PitchState.LOW;
    private double pitchPosition = ShooterConstants.pitchLowPosition;
    private boolean rightVelocityControlEnabled;
    private double targetRpm = ShooterConstants.targetRpm;

    public enum ShooterState {
        STOP(0),
        LOW(ShooterConstants.lowPower),
        HIGH(ShooterConstants.highPower);

        final double power;

        ShooterState(double power) {
            this.power = power;
        }
    }

    public enum PitchState {
        LOW(ShooterConstants.pitchLowPosition),
        HIGH(ShooterConstants.pitchHighPosition);

        final double position;

        PitchState(double position) {
            this.position = position;
        }
    }

    public Shooter(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);
        leftPitchServo = hardwareMap.get(Servo.class, ShooterConstants.leftPitchServoName);
        rightPitchServo = hardwareMap.get(Servo.class, ShooterConstants.rightPitchServoName);

        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        // The two linkages move opposite mechanically, so mirror the right servo command.
        rightPitchServo.setDirection(Servo.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stop();
    }

    public void setShooterState(ShooterState shooterState) {
        rightVelocityControlEnabled = false;
        this.shooterState = shooterState;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public boolean isRunning() {
        return shooterState != ShooterState.STOP;
    }

    /** Starts both flywheels; only the right flywheel has encoder feedback. */
    public void startFlywheelsAtTargetRpm(double rpm) {
        targetRpm = Math.max(0.0, rpm);
        rightVelocityControlEnabled = true;
        shooterState = ShooterState.HIGH;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getRightFlywheelRpm() {
        return Math.abs(rightShooter.getVelocity()) * 60.0
                / ShooterConstants.rightEncoderTicksPerRevolution;
    }

    public boolean isAtTargetRpm() {
        return rightVelocityControlEnabled && targetRpm > 0.0
                && Math.abs(getRightFlywheelRpm() - targetRpm)
                <= ShooterConstants.readyToleranceRpm;
    }

    public void setPitchState(PitchState pitchState) {
        this.pitchState = pitchState;
        pitchPosition = pitchState.position;
    }

    public PitchState getPitchState() {
        return pitchState;
    }

    public void adjustPitch(double delta) {
        pitchPosition = Math.max(0.0, Math.min(1.0, pitchPosition + delta));
    }

    public double getPitchPosition() {
        return pitchPosition;
    }

    public double getAverageVelocity() {
        return Math.abs(rightShooter.getVelocity());
    }

    public void stop() {
        rightVelocityControlEnabled = false;
        shooterState = ShooterState.STOP;
    }

    @Override
    public void periodic() {
        if (rightVelocityControlEnabled) {
            if (rightShooter.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            // 6000 RPM with the 28-count encoder is 2800 ticks/s.
            rightShooter.setVelocity(targetRpm * ShooterConstants.rightEncoderTicksPerRevolution / 60.0);
            leftShooter.setPower(ShooterConstants.leftFlywheelFollowPower);
        } else {
            if (rightShooter.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            leftShooter.setPower(shooterState.power);
            rightShooter.setPower(shooterState.power);
        }
        leftPitchServo.setPosition(pitchPosition);
        rightPitchServo.setPosition(pitchPosition);
    }
}
