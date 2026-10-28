package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.utils.TurretMath;

/**
 * Manual first-stage turret control using the Yellow Jacket motor's built-in
 * encoder. No separate external encoder is configured or read on this robot.
 * Future closed-loop aiming belongs here.
 */
public class Turret extends SubsystemBase {
    private final DcMotorEx turretMotor;
    private double targetDegrees = Double.NaN;
    private double integral, previousError;
    private long previousNanos;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, TurretConstants.turretMotorName);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        double ticksPerTurn = TurretConstants.motorOutputTicksPerRevolution
                * TurretConstants.turretPulleyTeeth / TurretConstants.motorPulleyTeeth;
        return getTurretPos() * TurretConstants.encoderSign * 360.0 / ticksPerTurn;
    }

    /** Called every scheduler cycle. Uses physical, unwrapped position error. */
    public boolean aimAtDegrees(double requestedDegrees) {
        double current = getAngleDegrees();
        double previousTarget = targetDegrees;
        targetDegrees = TurretMath.reachableDegrees(requestedDegrees, current,
                TurretConstants.minDegrees(), TurretConstants.maxDegrees());
        if (!Double.isFinite(targetDegrees)) {
            stop();
            return false;
        }
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
        double output = Range.clip(error * TurretConstants.kP
                + integral * TurretConstants.kI + derivative * TurretConstants.kD,
                -TurretConstants.maxAimPower, TurretConstants.maxAimPower);
        if (Math.abs(error) <= TurretConstants.angleToleranceDegrees) {
            output = 0;
            integral = 0;
        }
        previousError = error;
        previousNanos = now;
        rotate(output);
        return true;
    }

    public double getTargetDegrees() { return targetDegrees; }

    public int getTurretPos() {
        // DcMotorEx.getCurrentPosition() is the motor's integrated encoder value.
        return turretMotor.getCurrentPosition();
    }

    public void stop() {
        targetDegrees = Double.NaN;
        integral = 0;
        previousNanos = 0;
        turretMotor.setPower(0);
    }
}
