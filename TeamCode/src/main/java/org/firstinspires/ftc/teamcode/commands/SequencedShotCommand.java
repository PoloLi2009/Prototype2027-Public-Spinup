package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import java.util.function.BooleanSupplier;

/** Manual intake and an RPM-qualified gate-before-feed shot. */
public class SequencedShotCommand extends CommandBase {
    public enum ShotState { IDLE, AIMING, FLYWHEEL_TEST, SPINNING_UP, OPENING_GATE, FEEDING }
    private static final double RPM_STEP = 100.0;
    private static final long READY_HOLD_MS = 250;
    private static final long GATE_OPEN_DELAY_MS = 500;

    private final Intake intake;
    private final Shooter shooter;
    private final Transit transit;
    private final Gamepad gamepad;
    private final BooleanSupplier aimReady;
    private final BooleanSupplier flywheelTestRequested;
    private final boolean spinWhileAiming;
    private final Intake.IntakeState manualIntakeState;
    private final Intake.IntakeState manualReverseState;
    private final double rpmStep;
    private final ElapsedTime readyTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();
    private ShotState shotState = ShotState.IDLE;
    private double targetRpm;
    private boolean lastUp, lastDown, lastLeft, lastRight;

    public SequencedShotCommand(Intake intake, Shooter shooter, Transit transit, Gamepad gamepad) {
        this(intake, shooter, transit, gamepad, 3000.0);
    }

    public SequencedShotCommand(Intake intake, Shooter shooter, Transit transit, Gamepad gamepad,
                                double startRpm) {
        this(intake, shooter, transit, gamepad, startRpm, () -> true,
                Intake.IntakeState.FORWARD, Intake.IntakeState.REVERSED);
    }

    public SequencedShotCommand(Intake intake, Shooter shooter, Transit transit, Gamepad gamepad,
                                double startRpm, BooleanSupplier aimReady,
                                Intake.IntakeState manualIntakeState,
                                Intake.IntakeState manualReverseState) {
        this(intake, shooter, transit, gamepad, startRpm, aimReady,
                manualIntakeState, manualReverseState, RPM_STEP, () -> false, false);
    }

    public SequencedShotCommand(Intake intake, Shooter shooter, Transit transit, Gamepad gamepad,
                                double startRpm, BooleanSupplier aimReady,
                                Intake.IntakeState manualIntakeState,
                                Intake.IntakeState manualReverseState, double rpmStep,
                                BooleanSupplier flywheelTestRequested, boolean spinWhileAiming) {
        this.intake = intake;
        this.shooter = shooter;
        this.transit = transit;
        this.gamepad = gamepad;
        this.aimReady = aimReady;
        this.flywheelTestRequested = flywheelTestRequested;
        this.spinWhileAiming = spinWhileAiming;
        this.manualIntakeState = manualIntakeState;
        this.manualReverseState = manualReverseState;
        this.rpmStep = rpmStep;
        targetRpm = Math.max(0.0, Math.min(ShooterConstants.maxRpm, startRpm));
        addRequirements(intake, shooter, transit);
    }

    @Override
    public void execute() {
        if (gamepad.dpad_up && !lastUp) shooter.adjustPitch(ShooterConstants.pitchStep);
        if (gamepad.dpad_down && !lastDown) shooter.adjustPitch(-ShooterConstants.pitchStep);
        double oldRpm = targetRpm;
        if (gamepad.dpad_right && !lastRight) {
            targetRpm = Math.min(ShooterConstants.maxRpm, targetRpm + rpmStep);
        }
        if (gamepad.dpad_left && !lastLeft) targetRpm = Math.max(0.0, targetRpm - rpmStep);
        lastUp = gamepad.dpad_up;
        lastDown = gamepad.dpad_down;
        lastRight = gamepad.dpad_right;
        lastLeft = gamepad.dpad_left;
        if (targetRpm != oldRpm && shotState != ShotState.IDLE) {
            shotState = ShotState.SPINNING_UP;
            readyTimer.reset();
        }

        boolean shootHeld = gamepad.right_trigger >= 0.5 && targetRpm > 0.0;
        if (!gamepad.left_bumper && targetRpm > 0.0 && flywheelTestRequested.getAsBoolean()) {
            shotState = ShotState.FLYWHEEL_TEST;
            transit.setGateState(Transit.GateState.CLOSED);
            intake.stop();
            shooter.startFlywheelsAtTargetRpm(targetRpm);
            return;
        }
        if (!shootHeld || gamepad.left_bumper) {
            shotState = ShotState.IDLE;
            transit.setGateState(Transit.GateState.CLOSED);
            if (gamepad.left_bumper) intake.setIntakeState(manualReverseState);
            else if (gamepad.left_trigger >= 0.5) intake.setIntakeState(manualIntakeState);
            else intake.stop();
            shooter.stop();
            return;
        }

        boolean aimed = aimReady.getAsBoolean();
        if (aimed || spinWhileAiming) {
            shooter.startFlywheelsAtTargetRpm(targetRpm);
        } else {
            shooter.stop();
        }
        boolean atSpeed = shooter.isAtTargetRpm();
        if (!atSpeed || shotState == ShotState.IDLE || shotState == ShotState.FLYWHEEL_TEST
                || (shotState == ShotState.AIMING && !spinWhileAiming)) {
            readyTimer.reset();
        }

        // Spin-up may overlap aiming, but losing aim always closes the gate and stops feed.
        if (!aimed) {
            shotState = ShotState.AIMING;
            transit.setGateState(Transit.GateState.CLOSED);
            intake.stop();
            return;
        }

        if (shotState == ShotState.IDLE || shotState == ShotState.AIMING
                || shotState == ShotState.FLYWHEEL_TEST) {
            shotState = ShotState.SPINNING_UP;
        }
        switch (shotState) {
            case SPINNING_UP:
                transit.setGateState(Transit.GateState.CLOSED);
                intake.stop();
                if (atSpeed && readyTimer.milliseconds() >= READY_HOLD_MS) {
                    transit.setGateState(Transit.GateState.OPEN);
                    gateTimer.reset();
                    shotState = ShotState.OPENING_GATE;
                }
                break;
            case OPENING_GATE:
                transit.setGateState(Transit.GateState.OPEN);
                intake.stop();
                if (gateTimer.milliseconds() >= GATE_OPEN_DELAY_MS) {
                    intake.setIntakeState(Intake.IntakeState.FEED);
                    shotState = ShotState.FEEDING;
                }
                break;
            case FEEDING:
                transit.setGateState(Transit.GateState.OPEN);
                intake.setIntakeState(Intake.IntakeState.FEED);
                break;
            default:
                break;
        }
    }

    public ShotState getShotState() { return shotState; }
    public double getTargetRpm() { return targetRpm; }
    public long getGateElapsedMs() {
        return shotState == ShotState.OPENING_GATE
                ? Math.min((long) gateTimer.milliseconds(), GATE_OPEN_DELAY_MS) : 0;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        shooter.stop();
        transit.setGateState(Transit.GateState.CLOSED);
        shotState = ShotState.IDLE;
    }
}
