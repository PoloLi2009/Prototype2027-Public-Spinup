package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

/** Manual turret control with automatic full-turn cable unwind at either limit. */
@TeleOp(name = "Turret Limit Test", group = "Test")
public class TurretLimitTest extends OpMode {
    private static final double STICK_DEADBAND = 0.08;
    private static final double REWIND_TRIGGER_MARGIN_DEGREES = 1.0;
    private static final double REWIND_COMPLETE_MARGIN_DEGREES = 1.0;
    private static final long REWIND_BURST_NANOS = 215_000_000L;
    private static final double REWIND_PID_MIN_POWER = 0.20;
    private static final double REWIND_PID_MAX_POWER = 0.50;
    private static final double REWIND_PID_KP = 0.02;

    private Turret turret;
    private boolean rewinding;
    private double rewindTargetDegrees;
    private double rewindStartAngleDegrees;
    private long rewindStartNanos;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        telemetry.addLine("Point turret straight forward BEFORE INIT; this resets to 0 deg.");
        telemetry.addLine("Gamepad 1 left stick X: turn turret; at +/-180 deg: auto unwind.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double angle = turret.getAngleDegrees();
        double stick = Math.abs(gamepad1.left_stick_x) > STICK_DEADBAND
                ? gamepad1.left_stick_x : 0.0;
        double min = TurretConstants.minDegrees();
        double max = TurretConstants.maxDegrees();

        if (rewinding) {
            double direction = Math.signum(rewindTargetDegrees - rewindStartAngleDegrees);
            long rewindElapsedNanos = System.nanoTime() - rewindStartNanos;
            if (direction * (rewindTargetDegrees - angle) <= REWIND_COMPLETE_MARGIN_DEGREES) {
                turret.stop();
                rewinding = false;
            } else if (rewindElapsedNanos < REWIND_BURST_NANOS) {
                turret.rotate(direction);
            } else {
                turret.aimAtPhysicalDegrees(rewindTargetDegrees, REWIND_PID_MIN_POWER,
                        REWIND_PID_MAX_POWER, REWIND_PID_KP);
            }
        } else if (stick > 0 && angle >= max - REWIND_TRIGGER_MARGIN_DEGREES
                && TurretConstants.travelDegrees >= 360.0) {
            beginRewind(min);
        } else if (stick < 0 && angle <= min + REWIND_TRIGGER_MARGIN_DEGREES
                && TurretConstants.travelDegrees >= 360.0) {
            beginRewind(max);
        } else {
            turret.rotate(stick * TurretConstants.manualPower);
        }

        telemetry.addData("Mode", rewinding
                ? (System.nanoTime() - rewindStartNanos < REWIND_BURST_NANOS
                        ? "AUTO UNWIND FULL POWER" : "AUTO UNWIND PID") : "MANUAL");
        telemetry.addData("Turret angle deg", "%.2f", turret.getAngleDegrees());
        telemetry.addData("Motor encoder ticks", turret.getEncoderTicks());
        telemetry.addData("Rewind target deg", rewinding ? rewindTargetDegrees : "--");
        telemetry.addData("Rewind elapsed sec", rewinding
                ? String.format("%.2f", (System.nanoTime() - rewindStartNanos) / 1e9) : "--");
        telemetry.addData("Min / Max deg", "%.1f / %.1f", min, max);
        telemetry.update();
    }

    private void beginRewind(double targetDegrees) {
        rewindTargetDegrees = targetDegrees;
        rewindStartAngleDegrees = turret.getAngleDegrees();
        rewindStartNanos = System.nanoTime();
        rewinding = true;
        turret.rotate(Math.signum(rewindTargetDegrees - rewindStartAngleDegrees));
    }

    @Override
    public void stop() {
        if (turret != null) turret.stop();
    }
}
