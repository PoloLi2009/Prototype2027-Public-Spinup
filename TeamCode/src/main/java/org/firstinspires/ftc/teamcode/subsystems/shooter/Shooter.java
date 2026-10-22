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
    private final Servo pitchServo;
    private ShooterState shooterState = ShooterState.STOP;
    private PitchState pitchState = PitchState.LOW;

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
        pitchServo = hardwareMap.get(Servo.class, ShooterConstants.pitchServoName);

        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stop();
    }

    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public boolean isRunning() {
        return shooterState != ShooterState.STOP;
    }

    public void setPitchState(PitchState pitchState) {
        this.pitchState = pitchState;
    }

    public PitchState getPitchState() {
        return pitchState;
    }

    public double getAverageVelocity() {
        return (Math.abs(leftShooter.getVelocity()) + Math.abs(rightShooter.getVelocity())) / 2.0;
    }

    public void stop() {
        setShooterState(ShooterState.STOP);
    }

    @Override
    public void periodic() {
        leftShooter.setPower(shooterState.power);
        rightShooter.setPower(shooterState.power);
        pitchServo.setPosition(pitchState.position);
    }
}
