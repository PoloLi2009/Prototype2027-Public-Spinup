package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/** Two motors mechanically coupled by gears and belts to the three flywheels. */
public class Shooter extends SubsystemBase {
    private final DcMotorEx leftShooter;
    private final DcMotorEx rightShooter;
    private final Servo leftPitchServo;
    private final Servo rightPitchServo;
    private ShooterState shooterState = ShooterState.STOP;
    private PitchState pitchState = PitchState.LOW;
    private double pitchPosition = ShooterConstants.pitchLowPosition;
    private boolean velocityControlEnabled;
    private double targetRpm = ShooterConstants.targetRpm;
    private double velocityIntegralPower;
    private long lastVelocityUpdateNanos;
    private double commandedFlywheelPower;

    public enum ShooterState {
        STOP(0),
        LOW(ShooterConstants.lowPower),
        HIGH(ShooterConstants.highPower),
        TEST_FULL(1.0);

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
        this(hardwareMap, true);
    }

    /** Set usePitchServos to false for motor-only tests. */
    public Shooter(HardwareMap hardwareMap, boolean usePitchServos) {
        leftShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);
        leftPitchServo = usePitchServos
                ? hardwareMap.get(Servo.class, ShooterConstants.leftPitchServoName) : null;
        rightPitchServo = usePitchServos
                ? hardwareMap.get(Servo.class, ShooterConstants.rightPitchServoName) : null;

        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        if (usePitchServos) {
            // The two linkages move opposite mechanically, so mirror the right servo command.
            leftPitchServo.setDirection(Servo.Direction.FORWARD);
            rightPitchServo.setDirection(Servo.Direction.REVERSE);
        }
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stop();
    }

    public void setShooterState(ShooterState shooterState) {
        velocityControlEnabled = false;
        velocityIntegralPower = 0.0;
        lastVelocityUpdateNanos = 0;
        this.shooterState = shooterState;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public boolean isRunning() {
        return shooterState != ShooterState.STOP;
    }

    /** Starts both flywheels using feedback from the left motor encoder. */
    public void startFlywheelsAtTargetRpm(double rpm) {
        double requestedRpm = Range.clip(rpm, 0.0, ShooterConstants.maxRpm);
        if (!velocityControlEnabled || requestedRpm != targetRpm) {
            velocityIntegralPower = 0.0;
            lastVelocityUpdateNanos = 0;
        }
        targetRpm = requestedRpm;
        if (targetRpm == 0.0) {
            stop();
            return;
        }
        velocityControlEnabled = true;
        shooterState = ShooterState.HIGH;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getLeftFlywheelRpm() {
        return Math.abs(leftShooter.getVelocity()) * 60.0
                / ShooterConstants.leftEncoderTicksPerRevolution;
    }

    public double getCommandedFlywheelPower() {
        return commandedFlywheelPower;
    }

    public boolean isAtTargetRpm() {
        return velocityControlEnabled && targetRpm > 0.0
                && Math.abs(getLeftFlywheelRpm() - targetRpm)
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
        return Math.abs(leftShooter.getVelocity());
    }

    public void stop() {
        velocityControlEnabled = false;
        velocityIntegralPower = 0.0;
        lastVelocityUpdateNanos = 0;
        shooterState = ShooterState.STOP;
    }

    @Override
    public void periodic() {
        if (velocityControlEnabled) {
            // One encoder measures the mechanically linked wheels; drive both motors equally.
            double errorRpm = targetRpm - getLeftFlywheelRpm();
            long nowNanos = System.nanoTime();
            double dt = lastVelocityUpdateNanos == 0 ? 0.0
                    : (nowNanos - lastVelocityUpdateNanos) / 1e9;
            lastVelocityUpdateNanos = nowNanos;
            double basePower = targetRpm / ShooterConstants.maxRpm
                    + ShooterConstants.flywheelKpPowerPerRpm * errorRpm;
            double unsaturatedPower = basePower + velocityIntegralPower;
            if (dt > 0.0 && dt < 0.2 && Double.isFinite(errorRpm)
                    && ((unsaturatedPower > 0.0 && unsaturatedPower < 1.0)
                    || (unsaturatedPower >= 1.0 && errorRpm < 0.0)
                    || (unsaturatedPower <= 0.0 && errorRpm > 0.0))) {
                velocityIntegralPower = Range.clip(velocityIntegralPower
                        + ShooterConstants.flywheelKiPowerPerRpmSecond * errorRpm * dt,
                        -0.5, 0.5);
            }
            commandedFlywheelPower = Double.isFinite(basePower) ?
                    Range.clip(basePower + velocityIntegralPower, 0.0, 1.0) : 0.0;
        } else {
            commandedFlywheelPower = shooterState.power;
        }
        leftShooter.setPower(commandedFlywheelPower);
        rightShooter.setPower(commandedFlywheelPower);
        if (leftPitchServo != null) {
            leftPitchServo.setPosition(pitchPosition);
            rightPitchServo.setPosition(pitchPosition);
        }
    }
}
