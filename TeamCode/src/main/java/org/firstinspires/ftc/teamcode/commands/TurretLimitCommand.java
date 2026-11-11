package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

/** Manual turret control and full-turn cable unwind at the software limits. */
public class TurretLimitCommand extends CommandBase {
    private static final double STICK_DEADBAND = 0.08;
    private static final double TRIGGER_MARGIN_DEGREES = 1.0;
    private static final double COMPLETE_MARGIN_DEGREES = 1.0;
    private static final long BURST_NANOS = 215_000_000L;
    private static final double PID_MIN_POWER = 0.20;
    private static final double PID_MAX_POWER = 0.75;
    private static final double PID_KP = 0.03;

    private final Turret turret;
    private final Gamepad gamepad;
    private boolean rewinding;
    private double rewindTargetDegrees;
    private double rewindStartAngleDegrees;
    private long rewindStartNanos;

    public TurretLimitCommand(Turret turret, Gamepad gamepad) {
        this.turret = turret;
        this.gamepad = gamepad;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double angle = turret.getAngleDegrees();
        double stick = Math.abs(gamepad.left_stick_x) > STICK_DEADBAND
                ? gamepad.left_stick_x : 0.0;
        double min = TurretConstants.minDegrees();
        double max = TurretConstants.maxDegrees();
        if (rewinding) {
            double direction = Math.signum(rewindTargetDegrees - rewindStartAngleDegrees);
            long elapsed = System.nanoTime() - rewindStartNanos;
            if (direction * (rewindTargetDegrees - angle) <= COMPLETE_MARGIN_DEGREES) {
                turret.stop();
                rewinding = false;
            } else if (elapsed < BURST_NANOS) {
                turret.rotate(direction);
            } else {
                turret.aimAtPhysicalDegrees(rewindTargetDegrees, PID_MIN_POWER,
                        PID_MAX_POWER, PID_KP);
            }
        } else if (stick > 0 && angle >= max - TRIGGER_MARGIN_DEGREES
                && TurretConstants.travelDegrees >= 360.0) {
            beginRewind(min);
        } else if (stick < 0 && angle <= min + TRIGGER_MARGIN_DEGREES
                && TurretConstants.travelDegrees >= 360.0) {
            beginRewind(max);
        } else {
            turret.rotate(stick * TurretConstants.manualPower);
        }
    }

    private void beginRewind(double targetDegrees) {
        rewindTargetDegrees = targetDegrees;
        rewindStartAngleDegrees = turret.getAngleDegrees();
        rewindStartNanos = System.nanoTime();
        rewinding = true;
        turret.rotate(Math.signum(rewindTargetDegrees - rewindStartAngleDegrees));
    }

    public String getMode() {
        if (!rewinding) return "MANUAL";
        return System.nanoTime() - rewindStartNanos < BURST_NANOS
                ? "AUTO UNWIND FULL POWER" : "AUTO UNWIND PID";
    }
    public boolean isRewinding() { return rewinding; }
    public double getRewindTargetDegrees() { return rewindTargetDegrees; }
    public double getRewindElapsedSeconds() {
        return (System.nanoTime() - rewindStartNanos) / 1e9;
    }

    @Override
    public void end(boolean interrupted) { turret.stop(); }
}
