package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

/** Default manual turret command; later vision/field alignment commands replace it by requiring Turret. */
public class TurretCommand extends CommandBase {
    private final Turret turret;
    private final GamepadEx gamepadEx;

    public TurretCommand(Turret turret, GamepadEx gamepadEx) {
        this.turret = turret;
        this.gamepadEx = gamepadEx;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (gamepadEx.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            turret.rotate(-TurretConstants.manualPower);
        } else if (gamepadEx.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            turret.rotate(TurretConstants.manualPower);
        } else {
            turret.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
