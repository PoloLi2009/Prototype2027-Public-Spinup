package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

/** Holds the turret at the encoder angle measured when this command starts. */
public class TurretStartupHoldCommand extends CommandBase {
    private static final double MAX_LOCK_ERROR_DEGREES = 2.0;
    private final Turret turret;
    private double startupAngleDegrees = Double.NaN;
    private boolean holding;

    public TurretStartupHoldCommand(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // Capture the current output-shaft angle once; no chassis/field aiming is applied.
        startupAngleDegrees = turret.getAngleDegrees();
        holding = false;
    }

    @Override
    public void execute() {
        holding = turret.holdAtPhysicalDegrees(startupAngleDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        holding = false;
        turret.stop();
    }

    public double getStartupAngleDegrees() { return startupAngleDegrees; }
    public boolean isOnTarget() {
        return holding && Math.abs(turret.getAngleDegrees() - startupAngleDegrees)
                <= MAX_LOCK_ERROR_DEGREES;
    }
}
