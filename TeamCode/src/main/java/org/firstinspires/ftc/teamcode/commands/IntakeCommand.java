package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/** Holds collected balls below the turret while the driver intakes. */
public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final Transit transit;

    public IntakeCommand(Intake intake, Transit transit) {
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
        intake.setIntakeState(Intake.IntakeState.FORWARD);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
