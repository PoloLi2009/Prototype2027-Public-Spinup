package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/** Manual unjam command; keeps the turret entry safely closed while reversing. */
public class ReverseIntakeCommand extends CommandBase {
    private final Intake intake;
    private final Transit transit;

    public ReverseIntakeCommand(Intake intake, Transit transit) {
        this.intake = intake;
        this.transit = transit;
        addRequirements(intake, transit);
    }

    @Override
    public void initialize() {
        transit.setGateState(Transit.GateState.CLOSED);
        transit.setRampState(Transit.RampState.DOWN);
    }

    @Override
    public void execute() {
        intake.setIntakeState(Intake.IntakeState.REVERSED);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
