package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/** Manual unjam command; keeps the turret entry safely closed while reversing. */
public class ReverseIntakeCommand extends CommandBase {
    private final Intake intake;
    private final Transit transit;
    private final RunIntakeCommand motor;

    public ReverseIntakeCommand(Intake intake, Transit transit) {
        this(intake, transit, Intake.IntakeState.REVERSED);
    }

    public ReverseIntakeCommand(Intake intake, Transit transit, Intake.IntakeState state) {
        motor = new RunIntakeCommand(intake, Objects.requireNonNull(state));
        this.intake = intake;
        this.transit = transit;
        addRequirements(intake, transit);
    }

    @Override
    public void initialize() {
        transit.setGateState(Transit.GateState.CLOSED);
    }

    @Override
    public void execute() {
        motor.execute();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
