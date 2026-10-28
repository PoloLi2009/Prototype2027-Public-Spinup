package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

/**
 * Feeds a ball only while a flywheel preset is active.  A future version should
 * replace isRunning() with encoder-velocity-at-setpoint after ball testing.
 */
public class TransitCommand extends CommandBase {
    private enum FeedState {
        SAFE,
        RETRACTING_GATE,
        RAISING_RAMP,
        FEEDING
    }

    private final Shooter shooter;
    private final Transit transit;
    private final Intake intake;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private FeedState feedState = FeedState.SAFE;

    public TransitCommand(Shooter shooter, Transit transit, Intake intake) {
        this.shooter = shooter;
        this.transit = transit;
        this.intake = intake;
        addRequirements(intake, transit);
    }

    @Override
    public void initialize() {
        moveToSafeState();
    }

    @Override
    public void execute() {
        if (!shooter.isRunning()) {
            moveToSafeState();
            return;
        }

        switch (feedState) {
            case SAFE:
                // Step 1: move the blocking rod away while the ramp remains down.
                transit.setGateState(Transit.GateState.OPEN);
                transit.setRampState(Transit.RampState.DOWN);
                intake.stop();
                stateTimer.reset();
                feedState = FeedState.RETRACTING_GATE;
                break;

            case RETRACTING_GATE:
                if (stateTimer.milliseconds() >= TransitConstants.gateRetractDelayMs) {
                    // Step 2 starts only after the blocking rod has had time to clear.
                    transit.setRampState(Transit.RampState.UP);
                    stateTimer.reset();
                    feedState = FeedState.RAISING_RAMP;
                }
                break;

            case RAISING_RAMP:
                if (stateTimer.milliseconds() >= TransitConstants.rampRaiseDelayMs) {
                    // Step 3: now it is safe for the shared motor to feed a ball.
                    intake.setIntakeState(Intake.IntakeState.FEED);
                    feedState = FeedState.FEEDING;
                }
                break;

            case FEEDING:
                intake.setIntakeState(Intake.IntakeState.FEED);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        moveToSafeState();
    }

    private void moveToSafeState() {
        intake.stop();
        transit.setGateState(Transit.GateState.CLOSED);
        transit.setRampState(Transit.RampState.DOWN);
        feedState = FeedState.SAFE;
    }
}
