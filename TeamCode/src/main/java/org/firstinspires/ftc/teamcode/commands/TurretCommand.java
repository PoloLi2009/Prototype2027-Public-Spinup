package org.firstinspires.ftc.teamcode.commands;

import java.util.function.DoubleSupplier;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

/** Default manual turret command; later vision/field alignment commands replace it by requiring Turret. */
public class TurretCommand extends CommandBase {
    private final Turret turret;
    private final DoubleSupplier power;

    public TurretCommand(Turret turret, GamepadEx gamepadEx) {
        this(turret, () -> gamepadEx.getButton(GamepadKeys.Button.DPAD_LEFT)
                ? -TurretConstants.manualPower
                : gamepadEx.getButton(GamepadKeys.Button.DPAD_RIGHT) ? TurretConstants.manualPower : 0.0);
    }

    /** Continuous bounded turret power; cancelled commands stop the turret. */
    public TurretCommand(Turret turret, DoubleSupplier power) {
        this.turret = turret;
        this.power = power;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double requested = power.getAsDouble();
        if (requested != 0.0) turret.rotate(requested);
        else turret.stop();
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
