package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/**
 * Feeds a ball only while a flywheel preset is active.  A future version should
 * replace isRunning() with encoder-velocity-at-setpoint after ball testing.
 */
public class TransitCommand extends CommandBase {
    private final Shooter shooter;
    private final Transit transit;
    private final Intake intake;

    public TransitCommand(Shooter shooter, Transit transit, Intake intake) {
        this.shooter = shooter;
        this.transit = transit;
        this.intake = intake;
        addRequirements(intake, transit);
    }

    @Override
    public void initialize() {
        transit.setGateState(Transit.GateState.CLOSED);
        transit.setRampState(Transit.RampState.DOWN);
    }

    @Override
    public void execute() {
        if (shooter.isRunning()) {
            transit.setRampState(Transit.RampState.UP);
            transit.setGateState(Transit.GateState.OPEN);
            intake.setIntakeState(Intake.IntakeState.FEED);
        } else {
            intake.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        transit.setGateState(Transit.GateState.CLOSED);
        transit.setRampState(Transit.RampState.DOWN);
    }
}
