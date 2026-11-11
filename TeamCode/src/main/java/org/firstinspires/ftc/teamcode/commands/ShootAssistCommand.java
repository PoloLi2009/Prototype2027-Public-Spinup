package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.PositionHoldController;
import org.firstinspires.ftc.teamcode.subsystems.drive.ShootAssistDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.ShootAssistIntake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShootAssistShooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.ShootAssistTransit;
import org.firstinspires.ftc.teamcode.subsystems.transit.ShootAssistTransit.FiringState;
import org.firstinspires.ftc.teamcode.subsystems.vision.ShootAssistVision;

/**
 * Overtune ShootAssistCommand, line for line. Hold-to-shoot: initialize on RT press, execute while
 * held, end on release. Only the inputs are suppliers (Solo's button map) and the devices are the
 * Spinup ports of Overtune's Drive / Shooter / Intake / Transit / Vision APIs.
 *
 * <p>Solo runs its loop explicitly (no scheduler), so no FTCLib requirements are declared; Solo
 * guarantees that nothing else writes drive/shooter/gate/motor while this command is active.</p>
 */
public class ShootAssistCommand extends CommandBase {
    private final ShootAssistDrive drive;
    private final ShootAssistShooter shooter;
    private final ShootAssistIntake intake;
    private final ShootAssistTransit transit;
    private final ShootAssistVision vision;
    private final AutoAimController autoAim;
    private final PositionHoldController positionHold = new PositionHoldController();
    private final DoubleSupplier forwardInput, strafeInput, turnInput;
    private final BooleanSupplier intakeInput, jamInput, transitForwardInput;
    private final boolean autoAimEnabled;

    private boolean canPrepareToFire;
    private boolean manualTranslation;

    /**
     * @param forwardInput  OT gamepadEx.getLeftY() (up positive)
     * @param strafeInput   OT gamepadEx.getLeftX()
     * @param turnInput     OT gamepadEx.getRightX() (only used when auto aim is disabled)
     * @param intakeInput   OT LT
     * @param jamInput      OT RB (jam: reverse and cancel the shot)
     * @param transitForwardInput OT LB (transit forward while intake reverses)
     */
    public ShootAssistCommand(
            ShootAssistDrive drive,
            ShootAssistShooter shooter,
            ShootAssistIntake intake,
            ShootAssistTransit transit,
            ShootAssistVision vision,
            AutoAimController autoAim,
            DoubleSupplier forwardInput, DoubleSupplier strafeInput, DoubleSupplier turnInput,
            BooleanSupplier intakeInput, BooleanSupplier jamInput, BooleanSupplier transitForwardInput,
            boolean autoAimEnabled
    ) {
        this.drive = Objects.requireNonNull(drive);
        this.shooter = Objects.requireNonNull(shooter);
        this.intake = Objects.requireNonNull(intake);
        this.transit = Objects.requireNonNull(transit);
        this.vision = Objects.requireNonNull(vision);
        this.autoAim = Objects.requireNonNull(autoAim);
        this.forwardInput = Objects.requireNonNull(forwardInput);
        this.strafeInput = Objects.requireNonNull(strafeInput);
        this.turnInput = Objects.requireNonNull(turnInput);
        this.intakeInput = Objects.requireNonNull(intakeInput);
        this.jamInput = Objects.requireNonNull(jamInput);
        this.transitForwardInput = Objects.requireNonNull(transitForwardInput);
        this.autoAimEnabled = autoAimEnabled;
    }

    @Override
    public void initialize() {
        shooter.stop();
        autoAim.reset();
        positionHold.reset();
        manualTranslation = false;
        resetFiringSequence(FiringState.WAITING);
    }

    @Override
    public void execute() {
        double forward = applyDeadband(forwardInput.getAsDouble());
        double strafe = applyDeadband(strafeInput.getAsDouble());
        manualTranslation = forward != 0.0 || strafe != 0.0;
        if (!autoAimEnabled || !vision.isAimCalibrationReady() || manualTranslation) {
            positionHold.reset();
        } else {
            positionHold.update(drive.getPose(), drive.isFieldPoseInitialized(),
                    drive.getTranslationSpeedInchesPerSecond());
            forward = positionHold.getForwardPower();
            strafe = positionHold.getStrafePower();
        }

        boolean intakeRequested = intakeInput.getAsBoolean();
        boolean jamRequested = jamInput.getAsBoolean();
        intake.updateInputs(intakeRequested, jamRequested, true);
        double turnPower = updateShootRequest(jamRequested);
        if (transitForwardInput.getAsBoolean() && !jamRequested) {
            transit.forward();
        }
        if (autoAimEnabled && !manualTranslation) {
            drive.moveRobot(forward, strafe, turnPower);
        } else {
            // Manual translation stays headless during RT; position hold already returns robot-space powers.
            drive.moveFieldCentric(forward, strafe,
                    autoAimEnabled ? turnPower : applyDeadband(turnInput.getAsDouble()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        autoAim.reset();
        positionHold.reset();
        manualTranslation = false;
        resetFiringSequence(FiringState.IDLE);
        shooter.stop();
        intake.suspend();
        drive.stop();
    }

    public boolean canFire() {
        return transit.getFiringState() == FiringState.FEEDING;
    }

    public boolean canPrepareToFire() {
        return canPrepareToFire;
    }

    public FiringState getFiringState() {
        return transit.getFiringState();
    }

    public boolean isManualTranslation() {
        return manualTranslation;
    }

    public boolean isPositionHoldLocked() {
        return positionHold.isLocked();
    }

    private double updateShootRequest(boolean jamRequested) {
        if (jamRequested) {
            shooter.stop();
            autoAim.reset();
            resetFiringSequence(FiringState.WAITING);
            transit.reverse();
            return 0.0;
        }

        boolean validAimingTarget = vision.hasValidAimingTarget();
        double autoAimTurn = 0.0;
        if (autoAimEnabled) {
            if (vision.wasCorrectionApplied()) autoAim.reset();
            autoAimTurn = autoAim.calculateTurnPower(
                    validAimingTarget && !vision.shouldPauseForAimCalibration(), vision.getYawErrorDegrees());
        }

        double distanceCm = validAimingTarget ? DistanceUnit.INCH.toCm(
                vision.getTargetHorizontalDistanceInches()) : Double.NaN;
        if (!shooter.setDistanceTarget(distanceCm)) {
            // Invalid targeting also cancels an already committed feed.
            resetFiringSequence(FiringState.WAITING);
            return validAimingTarget ? autoAimTurn : 0.0;
        }

        canPrepareToFire = (!autoAimEnabled || vision.isAimCalibrationReady())
                && shooter.isReady()
                && (!autoAimEnabled || autoAim.isLocked())
                && shooter.isPitchReady();
        transit.requestFeed(canPrepareToFire);

        return validAimingTarget ? autoAimTurn : 0.0;
    }

    private void resetFiringSequence(FiringState resetState) {
        if (transit.getState() == ShootAssistTransit.TransitState.OPEN) {
            intake.clearBallCount();
        }
        canPrepareToFire = false;
        transit.cancelFeeding();
        if (resetState != FiringState.IDLE) {
            transit.requestFeed(false);
        }
    }

    private double applyDeadband(double input) {
        return Math.abs(input) > DriveConstants.joystickDeadband ? input : 0.0;
    }
}
