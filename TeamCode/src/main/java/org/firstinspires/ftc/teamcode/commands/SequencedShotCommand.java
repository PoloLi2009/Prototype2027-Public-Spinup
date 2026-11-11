package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import java.util.function.BooleanSupplier;

/** Manual input adapter; all qualified shooting is delegated to ShootCommand. */
public class SequencedShotCommand extends CommandBase {
    public enum ShotState { IDLE, AIMING, FLYWHEEL_TEST, SPINNING_UP, OPENING_GATE, FEEDING }
    private static final double RPM_STEP = 100.0;

    private final FlywheelRpmCommand spin;
    private final Intake intake;
    private final Shooter shooter;
    private final Transit transit;
    private final Gamepad gamepad;
    private final Gamepad pitchPresetGamepad;
    private final BooleanSupplier flywheelTestRequested;
    private final Intake.IntakeState manualIntakeState;
    private final Intake.IntakeState manualReverseState;
    private final double rpmStep;
    private final ShootCommand shot;
    private final SetPitchCommand highPitch, lowPitch, increasePitch, decreasePitch;
    private boolean flywheelTestActive;
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
        this(intake, shooter, transit, gamepad, startRpm, aimReady, manualIntakeState,
                manualReverseState, rpmStep, flywheelTestRequested, spinWhileAiming, null);
    }

    /** Provide a second gamepad to use the same pitch presets as Spinup TeleOp. */
    public SequencedShotCommand(Intake intake, Shooter shooter, Transit transit, Gamepad gamepad,
                                double startRpm, BooleanSupplier aimReady,
                                Intake.IntakeState manualIntakeState,
                                Intake.IntakeState manualReverseState, double rpmStep,
                                BooleanSupplier flywheelTestRequested, boolean spinWhileAiming,
                                Gamepad pitchPresetGamepad) {
        this.intake = intake;
        this.shooter = shooter;
        this.transit = transit;
        this.gamepad = gamepad;
        this.pitchPresetGamepad = pitchPresetGamepad;
        this.flywheelTestRequested = flywheelTestRequested;
        this.manualIntakeState = manualIntakeState;
        this.manualReverseState = manualReverseState;
        this.rpmStep = rpmStep;
        targetRpm = Math.max(0.0, Math.min(ShooterConstants.maxRpm, startRpm));
        shot = new ShootCommand(intake, shooter, transit, () -> targetRpm, aimReady, spinWhileAiming);
        spin = new FlywheelRpmCommand(shooter, () -> targetRpm);
        highPitch = new SetPitchCommand(shooter, Shooter.PitchState.HIGH);
        lowPitch = new SetPitchCommand(shooter, Shooter.PitchState.LOW);
        increasePitch = new SetPitchCommand(shooter, () -> ShooterConstants.pitchStep);
        decreasePitch = new SetPitchCommand(shooter, () -> -ShooterConstants.pitchStep);
        addRequirements(intake, shooter, transit);
    }

    @Override
    public void execute() {
        double oldPitch = shooter.getPitchPosition();
        updatePitch();
        double oldRpm = targetRpm;
        if (gamepad.dpad_right && !lastRight) {
            targetRpm = Math.min(ShooterConstants.maxRpm, targetRpm + rpmStep);
        }
        if (gamepad.dpad_left && !lastLeft) targetRpm = Math.max(0.0, targetRpm - rpmStep);
        lastRight = gamepad.dpad_right;
        lastLeft = gamepad.dpad_left;
        if ((targetRpm != oldRpm || shooter.getPitchPosition() != oldPitch)
                && getShotState() != ShotState.IDLE) {
            flywheelTestActive = false;
            shot.restartReadiness();
        }

        boolean shootHeld = gamepad.right_trigger >= 0.5 && targetRpm > 0.0;
        if (!gamepad.left_bumper && targetRpm > 0.0 && flywheelTestRequested.getAsBoolean()) {
            flywheelTestActive = true;
            shot.resetState();
            transit.setGateState(Transit.GateState.CLOSED);
            intake.stop();
            spin.execute();
            return;
        }
        if (!shootHeld || gamepad.left_bumper) {
            flywheelTestActive = false;
            shot.resetState();
            transit.setGateState(Transit.GateState.CLOSED);
            if (gamepad.left_bumper) intake.setIntakeState(manualReverseState);
            else if (gamepad.left_trigger >= 0.5) intake.setIntakeState(manualIntakeState);
            else intake.stop();
            shooter.stop();
            return;
        }

        flywheelTestActive = false;
        shot.execute();
    }

    private void updatePitch() {
        if (pitchPresetGamepad != null) {
            if (pitchPresetGamepad.dpad_up) {
                highPitch.initialize();
            } else if (pitchPresetGamepad.dpad_down) {
                lowPitch.initialize();
            }
        } else {
            if (gamepad.dpad_up && !lastUp) increasePitch.initialize();
            if (gamepad.dpad_down && !lastDown) decreasePitch.initialize();
            lastUp = gamepad.dpad_up;
            lastDown = gamepad.dpad_down;
        }
    }

    public void setReadyHoldMilliseconds(long milliseconds) { shot.setReadyHoldMilliseconds(milliseconds); }

    public ShotState getShotState() {
        return flywheelTestActive ? ShotState.FLYWHEEL_TEST : ShotState.valueOf(shot.getShotState().name());
    }
    public double getTargetRpm() { return targetRpm; }
    public long getGateElapsedMs() { return shot.getGateElapsedMs(); }

    @Override
    public void end(boolean interrupted) {
        shot.end(interrupted);
        flywheelTestActive = false;
    }
}
