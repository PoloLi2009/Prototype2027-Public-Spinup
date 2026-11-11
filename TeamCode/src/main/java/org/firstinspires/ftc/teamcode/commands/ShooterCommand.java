package org.firstinspires.ftc.teamcode.commands;

import java.util.function.Supplier;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/** Default shooter command: held presets and independently selected pitch. */
public class ShooterCommand extends CommandBase {
    private final Shooter shooter;
    private final Supplier<Shooter.ShooterState> speed;
    private final Supplier<Shooter.PitchState> pitch;

    public ShooterCommand(Shooter shooter, GamepadEx gamepadEx) {
        this(shooter,
                () -> gamepadEx.getButton(GamepadKeys.Button.Y) ? Shooter.ShooterState.HIGH
                        : gamepadEx.getButton(GamepadKeys.Button.X) ? Shooter.ShooterState.LOW : null,
                () -> gamepadEx.getButton(GamepadKeys.Button.DPAD_UP) ? Shooter.PitchState.HIGH
                        : gamepadEx.getButton(GamepadKeys.Button.DPAD_DOWN) ? Shooter.PitchState.LOW : null);
    }

    /** Null speed stops flywheels; null pitch preserves the current position. */
    public ShooterCommand(Shooter shooter, Supplier<Shooter.ShooterState> speed,
                          Supplier<Shooter.PitchState> pitch) {
        this.shooter = shooter;
        this.speed = speed;
        this.pitch = pitch;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        Shooter.ShooterState requestedSpeed = speed.get();
        if (requestedSpeed != null) shooter.setShooterState(requestedSpeed);
        else shooter.stop();
        Shooter.PitchState requestedPitch = pitch.get();
        if (requestedPitch != null) shooter.setPitchState(requestedPitch);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
