package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/** Runs the paired flywheel motors at full power. */
public class ShooterMotorTestCommand extends CommandBase {
    private final Shooter shooter;

    public ShooterMotorTestCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() { shooter.setShooterState(Shooter.ShooterState.TEST_FULL); }

    @Override
    public void end(boolean interrupted) { shooter.stop(); }
}
