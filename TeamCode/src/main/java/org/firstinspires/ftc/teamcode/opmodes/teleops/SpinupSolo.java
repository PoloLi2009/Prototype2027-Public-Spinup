package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.BlueManualDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ShootAssistCommand;
import org.firstinspires.ftc.teamcode.commands.TurretStartupHoldCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.ShootAssistConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.ShootAssistDrive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.ShootAssistIntake;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShootAssistShooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.ShootAssistTransit;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.vision.ShootAssistVision;

/**
 * Single-driver TeleOp running Overtune's TeleOpBase flow: Blue headless drive, intake default,
 * distance pre-spin default and the ported Overtune ShootAssistCommand (chassis auto-aim, vision
 * calibration, position hold, speed/lock/pitch-gated feed) on RT. The turret stays locked at its
 * startup angle; Spinup shoots out of the chassis front (shooterYawDegrees = 0).
 */
@TeleOp(name = "solo", group = "TeleOp")
public class SpinupSolo extends LinearOpMode {
    private static final double INITIAL_TICKS_PER_SECOND = 900.0;
    private static final double RPM_STEP = 100.0;
    private static final double TRIGGER_THRESHOLD = 0.1; // Overtune TeleOpConstants.triggerThreshold
    private static final double OD_POLL_SECONDS = 0.05;
    private static final double OD_FAULT_SECONDS = 3.0;
    private final SoloControlSafety safety = new SoloControlSafety();
    private final ElapsedTime odClock = new ElapsedTime();
    private Drive drive;
    private BlueManualDriveCommand driveCommand;
    private Shooter shooter;
    private Intake intake;
    private Transit transit;
    private Turret turret;
    private TurretStartupHoldCommand turretHold;
    private Pinpoint pinpoint;
    private LimelightVision limelight;
    private ShootAssistDrive shootDrive;
    private ShootAssistShooter shootShooter;
    private ShootAssistTransit shootTransit;
    private ShootAssistIntake shootIntake;
    private ShootAssistVision vision;
    private AutoAimController autoAim;
    private ShootAssistCommand shootAssist;
    private boolean shootAssistActive;
    private boolean odEnabled, odReady, lastHeadingReset, lastAlliance, lastRight, lastLeft;
    private boolean started, aimRequested, stopLatched;
    private double targetTicksPerSecond = INITIAL_TICKS_PER_SECOND;
    private double headingDegrees;
    private double nextOdPoll, invalidOdSince = Double.NaN;
    private String odStatus = "Waiting for Pinpoint";

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        try {
            drive = new Drive(hardwareMap);
            intake = new Intake(hardwareMap);
            shooter = new Shooter(hardwareMap);
            transit = new Transit(hardwareMap);
            turret = new Turret(hardwareMap);
            turretHold = new TurretStartupHoldCommand(turret);
            turretHold.initialize();
            limelight = new LimelightVision(hardwareMap);
            GamepadEx gamepadEx = new GamepadEx(gamepad1);
            driveCommand = new BlueManualDriveCommand(drive, gamepadEx,
                    () -> Math.toRadians(headingDegrees), () -> odReady, turn -> turn);
            shootDrive = new ShootAssistDrive(drive, () -> odEnabled ? pinpoint : null,
                    () -> odReady, driveCommand);
            shootShooter = new ShootAssistShooter(shooter, () -> targetTicksPerSecond);
            shootTransit = new ShootAssistTransit(transit, intake);
            shootIntake = new ShootAssistIntake(shootTransit);
            // Camera rides on the locked turret: the solver uses the startup (locked) angle.
            vision = new ShootAssistVision(limelight, turretHold::getStartupAngleDegrees);
            autoAim = new AutoAimController();
            shootAssist = new ShootAssistCommand(shootDrive, shootShooter, shootIntake, shootTransit,
                    vision, autoAim,
                    gamepadEx::getLeftY, gamepadEx::getLeftX, gamepadEx::getRightX,
                    () -> gamepad1.left_trigger > TRIGGER_THRESHOLD,  // OT LT intake
                    () -> gamepad1.right_bumper,                      // OT RB jam
                    () -> false,                                      // OT LB transit-forward: one motor on Spinup
                    true);
            stopMechanisms();
            telemetry.setMsTransmissionInterval(100);

            if (isStopRequested()) return;
            // Holding Back at INIT skips Pinpoint when troubleshooting the I2C connection.
            if (gamepad1.back) disableOdometry("Disabled by driver");
            else enableOdometry();
            while (!isStarted() && !isStopRequested()) {
                if (gamepad1.back && odEnabled) disableOdometry("Disabled by driver");
                updateAlliance();
                updateSensors();
                vision.updateAimTarget(shootDrive, gamepad1.b,
                        gamepad1.right_trigger > TRIGGER_THRESHOLD);
                telemetry.addLine("Spinup solo: Overtune shoot assist, chassis aim, locked turret");
                telemetry.addLine("INIT: align turret forward. X toggles alliance. Back disables Pinpoint.");
                telemetry.addLine("RT aim+shoot | LT intake | LB reverse | RB jam | B stop | A pre-spin");
                telemetry.addLine("D-pad L/R flywheel speed | Gamepad 2 D-pad up/down pitch | L3 heading reset");
                showTelemetry();
                sleep(20);
            }
            if (isStopRequested()) return;
            updateSensors();
            driveCommand.initialize();
            lastHeadingReset = gamepad1.left_stick_button;
            started = true;

            // Explicit loop in Overtune scheduler order: sensors, buttons, then commands, then outputs.
            while (opModeIsActive()) {
                if (gamepad1.back && odEnabled) disableOdometry("Disabled by driver");
                aimRequested = gamepad1.right_trigger > TRIGGER_THRESHOLD;
                // B applies motor stop before any optional I2C transaction.
                if (gamepad1.b) {
                    safety.requireNeutral();
                    cancelShootAssist();
                    stopMechanisms();
                    drive.stop();
                } else {
                    updateSensors();
                }
                if (gamepad1.left_stick_button && !lastHeadingReset) {
                    driveCommand.resetDriverHeading();
                }
                lastHeadingReset = gamepad1.left_stick_button;
                updateAlliance();
                updateTargetSpeedAndPitch();
                stopLatched = safety.update(gamepad1.b, controlsNeutral(), odReady);

                turretHold.execute();
                turret.periodic();
                shootDrive.periodic();
                vision.update();
                vision.updateAimTarget(shootDrive, stopLatched || gamepad1.b, aimRequested);
                if (stopLatched) {
                    cancelShootAssist();
                    autoAim.reset();
                    stopMechanisms();
                } else {
                    shootIntake.setReverseRequested(gamepad1.left_bumper);
                    if (aimRequested) {
                        if (!shootAssistActive) {
                            shootAssist.initialize();
                            shootAssistActive = true;
                        }
                        shootAssist.execute();
                    } else {
                        cancelShootAssist();
                        runDefaults();
                    }
                }
                shootShooter.periodic();
                transit.periodic();
                intake.periodic();
                showTelemetry();
                sleep(10);
            }
        } finally {
            // Attempt every output even if one device has gone offline.
            cleanup(this::cancelShootAssist);
            cleanup(() -> { if (drive != null) drive.stop(); });
            cleanup(() -> { if (shootTransit != null) shootTransit.cancelFeeding(); });
            cleanup(() -> { if (shootIntake != null) shootIntake.stop(); });
            cleanup(() -> { if (intake != null) { intake.stop(); intake.periodic(); } });
            cleanup(() -> { if (shootShooter != null) shootShooter.stop(); });
            cleanup(() -> { if (shooter != null) { shooter.stop(); shooter.periodic(); } });
            cleanup(() -> { if (transit != null) {
                transit.setGateState(Transit.GateState.CLOSED); transit.periodic();
            } });
            cleanup(() -> { if (turret != null) turret.shutdown(); });
            cleanup(() -> { if (vision != null) vision.stop(); });
            CommandScheduler.getInstance().reset();
        }
    }

    /** Overtune default commands: DriveCommand, IntakeCommand and the distance pre-spin RunCommand. */
    private void runDefaults() {
        driveCommand.execute();
        boolean intakeRequested = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        boolean jamRequested = gamepad1.right_bumper;
        shootIntake.updateInputs(intakeRequested, jamRequested);
        shootTransit.stopMotor(); // Spinup's single motor follows the intake request.
        if (gamepad1.a && !jamRequested) {
            shootShooter.setTargetVelocity(targetTicksPerSecond); // Solo manual pre-spin
        } else if (jamRequested || !vision.hasValidAimingTarget()) {
            shootShooter.stop();
        } else {
            shootShooter.setDistanceTarget(
                    DistanceUnit.INCH.toCm(vision.getTargetHorizontalDistanceInches()));
        }
    }

    private void cancelShootAssist() {
        if (shootAssistActive && shootAssist != null) {
            shootAssistActive = false;
            shootAssist.end(true);
        }
    }

    private void updateAlliance() {
        if (gamepad1.x && !lastAlliance) limelight.toggleAlliance();
        lastAlliance = gamepad1.x;
    }

    private void updateTargetSpeedAndPitch() {
        double step = RPM_STEP * ShooterConstants.rightEncoderTicksPerRevolution / 60.0;
        double max = ShooterConstants.maxRpm * ShooterConstants.rightEncoderTicksPerRevolution / 60.0;
        if (gamepad1.dpad_right && !lastRight) targetTicksPerSecond = Math.min(max, targetTicksPerSecond + step);
        if (gamepad1.dpad_left && !lastLeft) targetTicksPerSecond = Math.max(0.0, targetTicksPerSecond - step);
        lastRight = gamepad1.dpad_right;
        lastLeft = gamepad1.dpad_left;
        if (gamepad2.dpad_up) shooter.setPitchState(Shooter.PitchState.HIGH);
        else if (gamepad2.dpad_down) shooter.setPitchState(Shooter.PitchState.LOW);
    }

    private void showTelemetry() {
        boolean aligned = started && !stopLatched && aimRequested && vision.hasValidAimingTarget()
                && vision.isAimCalibrationReady() && autoAim.isLocked();
        String aimState = !started ? "Waiting for START" : stopLatched ? "Stopped (release controls)"
                : !aimRequested ? "Idle (hold RT to aim)"
                : !vision.isAimCalibrationReady() ? vision.getRtCalibrationStatus()
                : !vision.hasValidAimingTarget() ? "No valid aim target: " + vision.getTargetStatus()
                : aligned ? "Aligned and stable" : "Turning / not yet stable";
        if (started && aimRequested && !stopLatched && vision.hasValidAimingTarget()) {
            aimState += String.format(Locale.US, " (error %.1f deg / lock +/-%.1f)",
                    vision.getYawErrorDegrees(), ShootAssistConstants.aimLockToleranceDegrees);
        }
        telemetry.addData("Alliance / CELL", "%s / %s", vision.isBlueAlliance() ? "BLUE" : "RED",
                vision.getSelectedTarget().map(Enum::name).orElse("unknown"));
        telemetry.addData("Aim", aimState);
        boolean allowed = started && !stopLatched && aimRequested && shootAssist.canFire();
        telemetry.addData("Shoot allowed", allowed ? "YES" : "no: " + shootingBlockReason());
        telemetry.addData("Flywheel ticks/s actual / target", "%.0f / %s (%s)",
                shooter.getFlywheelVelocityTicksPerSecond(),
                shootShooter.hasTargetVelocity()
                        ? String.format(Locale.US, "%.0f", shootShooter.getTargetVelocity()) : "off",
                shootShooter.isReady() ? "ready" : "not ready");
        telemetry.addData("Solo target ticks/s", "%.0f (ready +/-%.0f)", targetTicksPerSecond,
                ShootAssistConstants.readyEnterToleranceTicksPerSecond);
        telemetry.addData("Pinpoint", odStatus);
        telemetry.addData("Localization", shootDrive.getLocalizationStatus());
        telemetry.addData("Vision", vision.getPoseStatus());
        telemetry.addData("Calibration", vision.getCalibrationStatus());
        double distanceIn = vision.getTargetHorizontalDistanceInches();
        telemetry.addData("Shot distance cm", Double.isFinite(distanceIn)
                ? String.format(Locale.US, "%.1f", DistanceUnit.INCH.toCm(distanceIn)) : "n/a");
        if (aimRequested) {
            telemetry.addData("Turn output", "%+.3f", autoAim.getRawTurnPower());
            telemetry.addData("Position hold", shootAssist.isPositionHoldLocked() ? "locked" : "moving/off");
        }
        telemetry.addData("Firing / gate / motor", "%s / %s / %s", shootTransit.getFiringState(),
                transit.getGateState(), intake.getIntakeState());
        telemetry.addData("Pitch preset / position", "%s / %.2f",
                shooter.getPitchState(), shooter.getPitchPosition());
        telemetry.addData("Turret startup / actual deg", "%.1f / %.1f",
                turretHold.getStartupAngleDegrees(), turret.getAngleDegrees());
        telemetry.update();
    }

    private String shootingBlockReason() {
        if (!started) return "waiting for START";
        if (stopLatched) return "B stop latched";
        if (!aimRequested) return "RT not held";
        if (gamepad1.right_bumper) return "jam clearing";
        if (!vision.isAimCalibrationReady()) return vision.getRtCalibrationStatus();
        if (!vision.hasValidAimingTarget()) return "localization or aim target invalid";
        if (shootAssist.getFiringState() == ShootAssistTransit.FiringState.REVERSING)
            return "released, reversing before feed";
        if (!shootShooter.isReady()) return "flywheel not at speed";
        if (!autoAim.isLocked()) return "not aligned yet";
        if (!shootShooter.isPitchReady()) return "pitch not settled";
        if (!shootTransit.areLimitPositionsCalibrated()) return "gate positions invalid";
        return "waiting for firing state";
    }

    private boolean controlsNeutral() {
        return deadband(gamepad1.left_stick_x) == 0.0 && deadband(gamepad1.left_stick_y) == 0.0
                && deadband(gamepad1.right_stick_x) == 0.0
                && gamepad1.left_trigger < 0.5 && gamepad1.right_trigger < 0.5
                && !gamepad1.a && !gamepad1.left_bumper && !gamepad1.right_bumper;
    }

    private double deadband(double value) {
        return Math.abs(value) > DriveConstants.joystickDeadband ? value : 0.0;
    }

    private void stopMechanisms() {
        if (shootTransit != null) shootTransit.cancelFeeding();
        if (shootIntake != null) shootIntake.stop();
        if (shootShooter != null) shootShooter.stop();
        else shooter.stop();
        intake.periodic(); shooter.periodic(); transit.periodic();
    }

    private void enableOdometry() {
        try {
            pinpoint = new Pinpoint(hardwareMap);
            odEnabled = true;
            invalidOdSince = odClock.seconds();
            odStatus = "Initializing / calibrating";
        } catch (RuntimeException failure) {
            disableOdometry("INIT failed: " + failure.getClass().getSimpleName());
        }
    }

    private boolean pollOdometry() {
        if (!odEnabled) return false;
        double now = odClock.seconds();
        if (now < nextOdPoll) return false;
        nextOdPoll = now + OD_POLL_SECONDS;
        try {
            pinpoint.periodic();
            headingDegrees = pinpoint.getHeadingDegrees();
            odReady = pinpoint.isReady() && Double.isFinite(headingDegrees);
            if (odReady) {
                invalidOdSince = Double.NaN;
                odStatus = "READY";
            } else {
                odStatus = "NOT READY; robot-centric fallback";
                if (!Double.isFinite(invalidOdSince)) invalidOdSince = now;
                if (now - invalidOdSince >= OD_FAULT_SECONDS) {
                    disableOdometry("Fault latched; no more reads until next INIT");
                }
            }
        } catch (RuntimeException failure) {
            disableOdometry("Read failed: " + failure.getClass().getSimpleName());
        }
        return odEnabled;
    }

    private void disableOdometry(String reason) {
        odEnabled = false;
        odReady = false;
        odStatus = reason;
    }

    private void updateSensors() { pollOdometry(); }

    private void cleanup(Runnable action) {
        try { action.run(); }
        catch (RuntimeException failure) {
            telemetry.addData("STOP cleanup", failure.toString());
            telemetry.update();
        }
    }
}
