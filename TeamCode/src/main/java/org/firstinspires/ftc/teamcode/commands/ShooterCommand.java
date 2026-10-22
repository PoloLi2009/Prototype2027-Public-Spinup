package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/** Default shooter command: held presets and independently selected pitch. */
public class ShooterCommand extends CommandBase {
    private final Shooter shooter;
    private final GamepadEx gamepadEx;

    public ShooterCommand(Shooter shooter, GamepadEx gamepadEx) {
        this.shooter = shooter;
        this.gamepadEx = gamepadEx;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (gamepadEx.getButton(GamepadKeys.Button.Y)) {
            shooter.setShooterState(Shooter.ShooterState.HIGH);
        } else if (gamepadEx.getButton(GamepadKeys.Button.X)) {
            shooter.setShooterState(Shooter.ShooterState.LOW);
        } else {
            shooter.stop();
        }

        if (gamepadEx.getButton(GamepadKeys.Button.DPAD_UP)) {
            shooter.setPitchState(Shooter.PitchState.HIGH);
        } else if (gamepadEx.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            shooter.setPitchState(Shooter.PitchState.LOW);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
