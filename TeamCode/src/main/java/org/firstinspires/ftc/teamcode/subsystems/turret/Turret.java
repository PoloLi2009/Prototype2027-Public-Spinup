package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.utils.TurretMath;

/** Controls the turret motor and reads the Through Bore encoder on its encoder port. */
public class Turret extends SubsystemBase {
    private static final double TICKS_PER_REV = 8192.0;
    private static final double MOTOR_PULLEY_TEETH = 20.0;
    private static final double TURRET_PULLEY_TEETH = 142.0;
    // Change to -1.0 if the Through Bore encoder counts opposite the desired direction.
    private static final double ENCODER_DIRECTION = 1.0;

    private final DcMotorEx turretMotor;
    private double targetDegrees = Double.NaN;
    private double integral, previousError;
    private long previousNanos;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, TurretConstants.turretMotorName);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoder();
        stop();
    }

    public void rotate(double power) {
        double output = Range.clip(power, -1.0, 1.0);
        if (TurretConstants.useSoftLimits) {
            double angle = getAngleDegrees();
            if ((angle <= TurretConstants.minDegrees() && output < 0)
                    || (angle >= TurretConstants.maxDegrees() && output > 0)) {
                output = 0;
            }
        }
        turretMotor.setPower(output * TurretConstants.powerSign);
    }

    public double getAngleDegrees() {
        // The encoder measures the motor shaft; 20:142 belt reduction converts it to turret angle.
        return getEncoderTicks() * ENCODER_DIRECTION * 360.0 / TICKS_PER_REV
                * MOTOR_PULLEY_TEETH / TURRET_PULLEY_TEETH;
    }

    public int degreesToTicks(double degrees) {
        // Reverse the output-angle conversion back to the motor-shaft encoder ticks.
        return (int) Math.round(degrees * TICKS_PER_REV * TURRET_PULLEY_TEETH
                / (360.0 * MOTOR_PULLEY_TEETH * ENCODER_DIRECTION));
    }

    public int getEncoderTicks() {
        // This encoder port is connected to the Through Bore incremental quadrature output.
        return turretMotor.getCurrentPosition();
    }

    public void setPower(double power) {
        turretMotor.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Called every scheduler cycle. Uses physical, unwrapped position error. */
    public boolean aimAtDegrees(double requestedDegrees) {
        double current = getAngleDegrees();
        double reachable = TurretMath.reachableDegrees(requestedDegrees, current,
                TurretConstants.minDegrees(), TurretConstants.maxDegrees());
        return aimAtPhysicalDegrees(reachable);
    }

    /** Follows a specific physical angle, so +180 to -180 traverses through zero. */
    public boolean aimAtPhysicalDegrees(double physicalDegrees) {
        return aimAtPhysicalDegrees(physicalDegrees, 0.0);
    }

    /** Minimum power helps a high-reduction turret overcome friction during the unwind test. */
    public boolean aimAtPhysicalDegrees(double physicalDegrees, double minimumPower) {
        return aimAtPhysicalDegrees(physicalDegrees, minimumPower,
                TurretConstants.maxAimPower, TurretConstants.kP);
    }

    /** Test-specific PID strength; other aiming callers keep their usual tuning. */
    public boolean aimAtPhysicalDegrees(double physicalDegrees, double minimumPower,
                                        double maximumPower, double kP) {
        return aimAtPhysicalDegrees(physicalDegrees, minimumPower, maximumPower, kP,
                TurretConstants.kD,
                TurretConstants.angleToleranceDegrees);
    }

    /** Strong, tight position hold used by the chassis aiming test to prevent turret wobble. */
    public boolean holdAtPhysicalDegrees(double physicalDegrees) {
        return aimAtPhysicalDegrees(physicalDegrees, 0.0,
                TurretConstants.startupHoldMaxPower, TurretConstants.startupHoldKp,
                TurretConstants.startupHoldKd,
                TurretConstants.startupHoldToleranceDegrees);
    }

    private boolean aimAtPhysicalDegrees(double physicalDegrees, double minimumPower,
                                         double maximumPower, double kP, double kD,
                                         double toleranceDegrees) {
        if (!Double.isFinite(physicalDegrees)
                || physicalDegrees < TurretConstants.minDegrees()
                || physicalDegrees > TurretConstants.maxDegrees()) {
            stop();
            return false;
        }
        double current = getAngleDegrees();
        double previousTarget = targetDegrees;
        targetDegrees = physicalDegrees;
        double error = targetDegrees - current;
        long now = System.nanoTime();
        double dt = previousNanos == 0 ? 0 : (now - previousNanos) / 1e9;
        if (!Double.isFinite(previousTarget) || Math.abs(previousTarget - targetDegrees) > 1e-6) {
            integral = 0;
            dt = 0; // Avoid derivative kick on target changes, including a rewind.
        }
        double derivative = dt > 0 && dt < 0.2 ? (error - previousError) / dt : 0;
        if (dt > 0 && dt < 0.2) {
            integral = Range.clip(integral + error * dt,
                    -TurretConstants.integralLimitDegreeSeconds,
                    TurretConstants.integralLimitDegreeSeconds);
        }
        double output = Range.clip(error * kP
                + integral * TurretConstants.kI + derivative * kD,
                -maximumPower, maximumPower);
        if (Math.abs(error) <= toleranceDegrees) {
            output = 0;
            integral = 0;
        } else if (minimumPower > 0 && Math.abs(output) < minimumPower) {
            output = Math.copySign(Math.min(minimumPower, maximumPower), error);
        }
        previousError = error;
        previousNanos = now;
        rotate(output);
        return true;
    }

    public double getTargetDegrees() { return targetDegrees; }

    public int getTurretPos() {
        return getEncoderTicks();
    }

    public void stop() {
        targetDegrees = Double.NaN;
        integral = 0;
        previousNanos = 0;
        turretMotor.setPower(0);
    }
}
