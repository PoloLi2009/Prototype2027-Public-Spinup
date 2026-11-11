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
    private double feedforwardPower, proportionalPower, derivativePower, velocityUpdateSeconds;
    private double previousMeasuredRpm = Double.NaN, filteredRpmSlope;
    private double[] tuningGains; // Per-instance P/I/D/F override; never writes shared constants.
    private double tuningPowerLimit = 1.0;
    private Double readyToleranceOverride;
    private Double readyAbsoluteToleranceTicksPerSecond;

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

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
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
        previousMeasuredRpm = Double.NaN; filteredRpmSlope = 0;
        this.shooterState = shooterState;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public boolean isRunning() {
        return shooterState != ShooterState.STOP;
    }

    /** Starts both motors using feedback from the right motor encoder. */
    public void startFlywheelsAtTargetRpm(double rpm) {
        if (!Double.isFinite(rpm)) { stop(); return; }
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

    public double getRightFlywheelRpm() {
        return getFlywheelVelocityTicksPerSecond() * 60.0
                / ShooterConstants.rightEncoderTicksPerRevolution;
    }

    public double getCommandedFlywheelPower() {
        return commandedFlywheelPower;
    }

    /** Per-instance absolute TPS window or default relative window; timing belongs to ShootCommand. */
    public boolean isAtTargetRpm() {
        if (readyAbsoluteToleranceTicksPerSecond != null) {
            double targetTps = targetRpm * ShooterConstants.rightEncoderTicksPerRevolution / 60.0;
            double measuredTps = getFlywheelVelocityTicksPerSecond();
            return velocityControlEnabled && Double.isFinite(targetTps) && targetTps > 0
                    && Double.isFinite(measuredTps) && measuredTps > 0
                    && Math.abs(targetTps - measuredTps) <= readyAbsoluteToleranceTicksPerSecond
                    + 4 * Math.ulp(targetTps);
        }
        double measuredRpm = getRightFlywheelRpm();
        double tolerance = getReadyRelativeTolerance();
        return velocityControlEnabled && Double.isFinite(targetRpm) && targetRpm > 0.0
                && Double.isFinite(measuredRpm) && Double.isFinite(tolerance)
                && tolerance >= 0.0 && tolerance < 1.0
                && // Allow only floating-point conversion roundoff at inclusive +/- tolerance edges.
                Math.abs(targetRpm - measuredRpm) <= targetRpm * tolerance + 4 * Math.ulp(targetRpm);
    }

    /** Per-OpMode readiness window, without changing other programs' shared default. */
    public void setReadyRelativeTolerance(double tolerance) {
        if (!Double.isFinite(tolerance) || tolerance < 0 || tolerance >= 1)
            throw new IllegalArgumentException("Readiness tolerance must be in [0,1)");
        readyToleranceOverride = tolerance;
        readyAbsoluteToleranceTicksPerSecond = null;
    }
    public double getReadyRelativeTolerance() {
        return readyToleranceOverride == null ? ShooterConstants.readyRelativeTolerance : readyToleranceOverride;
    }

    public void setReadyAbsoluteToleranceTicksPerSecond(double tolerance) {
        if (!Double.isFinite(tolerance) || tolerance < 0)
            throw new IllegalArgumentException("TPS tolerance must be finite and nonnegative");
        readyAbsoluteToleranceTicksPerSecond = tolerance;
    }
    public double getReadyToleranceTicksPerSecond() {
        return readyAbsoluteToleranceTicksPerSecond != null ? readyAbsoluteToleranceTicksPerSecond
                : targetRpm * ShooterConstants.rightEncoderTicksPerRevolution / 60.0 * getReadyRelativeTolerance();
    }

    /** Test-only per-instance gains: P/RPM, I/(RPM*s), D/(RPM/s), F/RPM. */
    public void setTuningGains(double p, double i, double d, double f, double powerLimit) {
        for (double value : new double[]{p, i, d, f, powerLimit}) {
            if (!Double.isFinite(value) || value < 0) throw new IllegalArgumentException("Invalid tuning gain");
        }
        if (powerLimit <= 0 || powerLimit > 1 || f <= 0) throw new IllegalArgumentException("Invalid tuning limit/F");
        stop();
        tuningGains = new double[]{p, i, d, f};
        tuningPowerLimit = powerLimit;
    }
    public void clearTuningGains() { stop(); tuningGains = null; tuningPowerLimit = 1.0; }
    public double getDerivativePower() { return derivativePower; }

    public double getFeedforwardPower() { return feedforwardPower; }
    public double getProportionalPower() { return proportionalPower; }
    public double getIntegralPower() { return velocityIntegralPower; }
    public double getVelocityUpdateSeconds() { return velocityUpdateSeconds; }

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

    public double getFlywheelVelocityTicksPerSecond() {
        return Math.abs(rightShooter.getVelocity());
    }

    public void stop() {
        velocityControlEnabled = false;
        velocityIntegralPower = 0.0;
        lastVelocityUpdateNanos = 0;
        shooterState = ShooterState.STOP;
        previousMeasuredRpm = Double.NaN; filteredRpmSlope = 0; derivativePower = 0;
    }

    @Override
    public void periodic() {
        if (velocityControlEnabled) {
            // One encoder measures the mechanically linked wheels; drive both motors equally.
            double measuredRpm = getRightFlywheelRpm();
            double errorRpm = targetRpm - measuredRpm;
            long nowNanos = System.nanoTime();
            double dt = lastVelocityUpdateNanos == 0 ? 0.0
                    : (nowNanos - lastVelocityUpdateNanos) / 1e9;
            lastVelocityUpdateNanos = nowNanos;
            velocityUpdateSeconds = dt;
            double kp = tuningGains == null ? ShooterConstants.flywheelKpPowerPerRpm : tuningGains[0];
            double ki = tuningGains == null ? ShooterConstants.flywheelKiPowerPerRpmSecond : tuningGains[1];
            double kd = tuningGains == null ? ShooterConstants.flywheelKdPowerPerRpmPerSecond : tuningGains[2];
            double kf = tuningGains == null ? ShooterConstants.flywheelKfPowerPerRpm : tuningGains[3];
            double limit = tuningPowerLimit;
            if (dt > 0 && dt < 0.2 && Double.isFinite(measuredRpm) && Double.isFinite(previousMeasuredRpm)) {
                double slope = (measuredRpm - previousMeasuredRpm) / dt;
                double alpha = dt / (Math.max(0.001, ShooterConstants.flywheelDerivativeFilterSeconds) + dt);
                filteredRpmSlope += alpha * (slope - filteredRpmSlope);
            } else {
                filteredRpmSlope = 0; // No derivative spike after initialization or a long gap.
            }
            previousMeasuredRpm = measuredRpm;
            feedforwardPower = kf * targetRpm;
            proportionalPower = kp * errorRpm;
            derivativePower = -kd * filteredRpmSlope; // D on measurement, not setpoint.
            double basePower = feedforwardPower + proportionalPower + derivativePower;
            double unsaturatedPower = basePower + velocityIntegralPower;
            // Bound long-loop integration instead of discarding it entirely. Anti-windup
            // uses the real 0..1 output limits; allow I enough range to reach full output.
            if (dt > 0.0 && Double.isFinite(dt) && Double.isFinite(errorRpm)
                    && ((unsaturatedPower > 0.0 && unsaturatedPower < limit)
                    || (unsaturatedPower >= limit && errorRpm < 0.0)
                    || (unsaturatedPower <= 0.0 && errorRpm > 0.0))) {
                velocityIntegralPower = Range.clip(velocityIntegralPower
                        + ki * errorRpm * Math.min(dt, 0.2),
                        -1.0, 1.0);
            }
            commandedFlywheelPower = Double.isFinite(basePower) ?
                    Range.clip(basePower + velocityIntegralPower, 0.0, limit) : 0.0;
        } else {
            feedforwardPower = proportionalPower = derivativePower = velocityUpdateSeconds = 0.0;
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
