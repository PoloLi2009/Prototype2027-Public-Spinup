package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

/** Shared intake motor action. Feeding prerequisites are owned by ShootCommand/TransitCommand. */
public class RunIntakeCommand extends CommandBase {
    private final Intake intake;
    private final Supplier<Intake.IntakeState> state;
    public RunIntakeCommand(Intake intake, Intake.IntakeState state) { this(intake, () -> state); }
    public RunIntakeCommand(Intake intake, Supplier<Intake.IntakeState> state) {
        this.intake = intake;
        this.state = Objects.requireNonNull(state);
        addRequirements(intake);
    }
    @Override public void execute() { intake.setIntakeState(state.get()); }
    @Override public void end(boolean interrupted) { intake.stop(); }
}
