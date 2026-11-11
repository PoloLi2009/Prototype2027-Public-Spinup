package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.SequencedShotCommand;
import org.firstinspires.ftc.teamcode.commands.TurretStartupHoldCommand;
import org.firstinspires.ftc.teamcode.commands.VisionCellChassisAimCommand;
import org.firstinspires.ftc.teamcode.commands.VisionReadCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.localization.PinpointConstants;
import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;

@TeleOp(name = "Vision Cell Chassis Aim Test", group = "Test")
public class VisionCellChassisAimTest extends CommandOpMode {
    private static final double START_FLYWHEEL_RPM = 3000.0;
    private static final double FLYWHEEL_STEP_TICKS_PER_SECOND = 50.0;
    private Drive drive;
    private Pinpoint pinpoint;
    private Turret turret;
    private LimelightVision vision;
    private VisionLocalization localization;
    private TurretStartupHoldCommand turretHold;
    private VisionCellChassisAimCommand chassisAim;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private SequencedShotCommand shotCommand;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        pinpoint = new Pinpoint(hardwareMap);
        turret = new Turret(hardwareMap);
        vision = new LimelightVision(hardwareMap);
        localization = new VisionLocalization();
        // Register drive after vision and before intake so the shot sees this loop's aim result.
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transit = new Transit(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(pinpoint, intake, shooter, transit);
        shooter.setPitchState(Shooter.PitchState.LOW);
        shooter.periodic();
        intake.periodic();
        transit.periodic();

        turretHold = new TurretStartupHoldCommand(turret);
        turret.setDefaultCommand(turretHold);
        vision.setDefaultCommand(new VisionReadCommand(vision, pinpoint, localization,
                gamepad1, true, turretHold::getStartupAngleDegrees));
        chassisAim = new VisionCellChassisAimCommand(drive, pinpoint, localization,
                vision, turretHold, gamepad1);
        drive.setDefaultCommand(chassisAim);
        double rpmStep = FLYWHEEL_STEP_TICKS_PER_SECOND * 60.0
                / ShooterConstants.rightEncoderTicksPerRevolution;
        shotCommand = new SequencedShotCommand(intake, shooter, transit, gamepad1,
                START_FLYWHEEL_RPM, chassisAim::isOnTarget,
                Intake.IntakeState.FAST_FORWARD, Intake.IntakeState.FAST_REVERSED, rpmStep,
                () -> gamepad1.a && gamepad1.right_trigger < 0.5, /* spinWhileAiming= */ true);
        intake.setDefaultCommand(shotCommand);

        telemetry.addLine("Align the turret with chassis forward before INIT.");
        telemetry.addLine("X: alliance. LT: collect. LB: eject. RT: aim + spin, then shoot.");
        telemetry.addLine("Hold A with RT released to test the flywheel.");
        telemetry.addLine("D-pad up/down: pitch. Left/right: flywheel -/+50 ticks/s.");
        telemetry.addData("OD start X/Y mm / heading deg", "%.0f / %.0f / %.1f",
                PinpointConstants.testStartFieldXmm, PinpointConstants.testStartFieldYmm,
                PinpointConstants.testStartFieldHeadingDegrees);
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        Pose2d fieldPose = chassisAim.getAimPose();
        double visionAge = localization.getLastVisionAgeSeconds();
        String poseSource = localization.hasFieldPose()
                ? String.format("LL %.1fs", visionAge) : "OD start";
        telemetry.addData("Pose", fieldPose == null ? "Waiting for Pinpoint"
                : String.format("Field %.2f/%.2f %.0f° | Drive %.2f/%.2f m | %s",
                        fieldPose.getX(), fieldPose.getY(), fieldPose.getRotation().getDegrees(),
                        chassisAim.getDriverXmm() / 1000.0, chassisAim.getDriverYmm() / 1000.0,
                        poseSource));
        telemetry.addData("Aim", "%s/%s | %s | err %.1f° pwr %.2f",
                vision.isBlueAlliance() ? "B" : "R",
                chassisAim.getTarget() == null ? "--" : (chassisAim.getTarget().upperHalf ? "UP" : "LOW"),
                chassisAim.getAimStatus(), chassisAim.getHeadingErrorDegrees(), chassisAim.getTurnPower());
        telemetry.addData("Shot", "%s | gate %s | intake %s",
                shotCommand.getShotState(), transit.getGateState(), intake.getIntakeState());
        double targetTicksPerSecond = shotCommand.getTargetRpm()
                * ShooterConstants.rightEncoderTicksPerRevolution / 60.0;
        telemetry.addData("Flywheel ticks/s", "%.0f / %.0f | pwr %.2f",
                shooter.getFlywheelVelocityTicksPerSecond(), targetTicksPerSecond, shooter.getCommandedFlywheelPower());
        double leftPitch = shooter.getPitchPosition();
        telemetry.addData("Pitch L/R", "%.2f / %.2f | turret %s",
                leftPitch, 1.0 - leftPitch, turretHold.isOnTarget() ? "locked" : "off angle");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            super.runOpMode();
        } finally {
            CommandScheduler.getInstance().reset();
            if (shotCommand != null) shotCommand.end(true);
            if (intake != null) intake.periodic();
            if (shooter != null) shooter.periodic();
            if (transit != null) transit.periodic();
            if (drive != null) drive.stop();
            if (turret != null) turret.shutdown();
            if (vision != null) vision.stop();
        }
    }
}
