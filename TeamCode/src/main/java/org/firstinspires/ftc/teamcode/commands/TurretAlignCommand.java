package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.turret.AimingConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.AimingInput;

/** Points at the allied CELL in the robot's current audience/opposite half-field. */
public class TurretAlignCommand extends CommandBase {
    private final Turret turret;
    private final AimingInput input;
    private String status = "WAITING";
    private double distanceMm = Double.NaN;
    private AimingInput.Cell selectedCell = AimingInput.Cell.UNKNOWN;

    public TurretAlignCommand(Turret turret, AimingInput input) {
        this.turret = turret;
        this.input = input;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        AimingInput.Observation o = input.getObservation();
        distanceMm = Double.NaN;
        selectedCell = AimingInput.Cell.UNKNOWN;
        long age = o == null ? -1 : System.nanoTime() - o.captureNanos;
        if (o == null || !o.valid || age < 0
                || age / 1e6 > AimingConstants.maxObservationAgeMs
                || !Double.isFinite(o.xMm) || !Double.isFinite(o.yMm)
                || !Double.isFinite(o.headingRadians)) {
            reject("NO_FRESH_POSE");
            return;
        }
        AimingConstants.Alliance alliance = AimingConstants.alliance;
        selectedCell = AimingConstants.cellForPosition(o.yMm);
        double[] target = AimingConstants.target(alliance, selectedCell);
        if (target == null || !Double.isFinite(target[0]) || !Double.isFinite(target[1])) {
            reject("UNKNOWN_CELL_OR_UNCONFIGURED_TARGET");
            return;
        }
        double dx = target[0] - o.xMm, dy = target[1] - o.yMm;
        distanceMm = Math.hypot(dx, dy);
        if (distanceMm < 1) {
            reject("TARGET_TOO_CLOSE");
            return;
        }
        double relativeDegrees = Math.toDegrees(Math.atan2(dy, dx) - o.headingRadians);
        status = turret.aimAtDegrees(relativeDegrees) ? "TRACKING" : "UNREACHABLE";
    }

    private void reject(String reason) {
        status = reason;
        turret.stop();
    }

    public String getStatus() { return status; }
    public AimingInput.Cell getSelectedCell() { return selectedCell; }
    public double getDistanceMm() { return distanceMm; }

    @Override
    public void end(boolean interrupted) { turret.stop(); }
}
