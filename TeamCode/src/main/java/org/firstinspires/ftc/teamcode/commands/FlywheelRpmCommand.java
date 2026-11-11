package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/** Continuous PIDF velocity control in RPM; release/cancel stops both flywheels. */
public class FlywheelRpmCommand extends CommandBase {
    private final Shooter shooter;
    private final DoubleSupplier rpm;
    public FlywheelRpmCommand(Shooter shooter, double rpm) { this(shooter, () -> rpm); }
    public FlywheelRpmCommand(Shooter shooter, DoubleSupplier rpm) {
        this.shooter = shooter;
        this.rpm = Objects.requireNonNull(rpm);
        addRequirements(shooter);
    }
    @Override public void execute() { shooter.startFlywheelsAtTargetRpm(rpm.getAsDouble()); }
    @Override public void end(boolean interrupted) { shooter.stop(); }
}
