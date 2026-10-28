package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.utils.TurretMath;

/** Only the turret motor is initialized. Gamepad 1 controls target angle. */
@TeleOp(name = "Turret Limit Test", group = "Test")
public class TurretLimitTest extends OpMode {
    private Turret turret;
    private double targetDegrees;
    private int lastDirection;
    private long nextRepeatNanos;
    private String status = "HOLD";
    private static final double STEP_DEGREES = 30;
    private static final long INITIAL_REPEAT_NS = 500_000_000L;
    private static final long REPEAT_NS = 250_000_000L;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        telemetry.addLine("Point turret forward BEFORE INIT. This position is zero.");
        telemetry.addLine("Gamepad 1: D-pad left +30 / right -30; hold to repeat.");
        telemetry.update();
    }

    @Override
    public void start() {
        targetDegrees = 0;
        lastDirection = 0;
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        // Both buttons pressed: no steps. Angle positive is counterclockwise.
        int direction = gamepad1.dpad_left == gamepad1.dpad_right ? 0
                : gamepad1.dpad_left ? 1 : -1;
        if (direction != 0) {
            if (direction != lastDirection) {
                step(direction);
                nextRepeatNanos = now + INITIAL_REPEAT_NS;
            } else if (now >= nextRepeatNanos) {
                step(direction);
                nextRepeatNanos = now + REPEAT_NS;
            }
        }
        lastDirection = direction;
        // Keep holding the last accepted target after button release.
        boolean reachable = turret.aimAtDegrees(targetDegrees);
        telemetry.addData("State", reachable ? status : "UNREACHABLE / STOPPED");
        telemetry.addData("Current degrees", turret.getAngleDegrees());
        telemetry.addData("Target degrees", turret.getTargetDegrees());
        telemetry.addData("Error degrees", turret.getTargetDegrees() - turret.getAngleDegrees());
        telemetry.addData("Encoder ticks", turret.getTurretPos());
        telemetry.addData("Travel degrees", TurretConstants.travelDegrees);
        telemetry.addData("Min / Max", "%.1f / %.1f",
                TurretConstants.minDegrees(), TurretConstants.maxDegrees());
        telemetry.update();
    }

    private void step(int direction) {
        double requested = targetDegrees + direction * STEP_DEGREES;
        double equivalent = TurretMath.reachableDegrees(requested, turret.getAngleDegrees(),
                TurretConstants.minDegrees(), TurretConstants.maxDegrees());
        if (!Double.isFinite(equivalent)) {
            status = "UNREACHABLE STEP: " + requested + " (holding last target)";
            return;
        }
        status = Math.abs(equivalent - requested) > 1e-6 ? "REWIND" : "HOLD";
        targetDegrees = equivalent;
    }

    @Override
    public void stop() {
        if (turret != null) turret.stop();
    }
}
