package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDriveCommand;
import org.firstinspires.ftc.teamcode.commands.SequencedShotCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/** Field-centric drive and sequenced shot test wiring and telemetry. */
@TeleOp(name = "Field Centric Transit Shooter Test", group = "Test")
public class FieldCentricTransitShooterTest extends CommandOpMode {
    private Drive drive;
    private Pinpoint pinpoint;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private FieldCentricDriveCommand driveCommand;
    private SequencedShotCommand shotCommand;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        drive = new Drive(hardwareMap);
        pinpoint = new Pinpoint(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transit = new Transit(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(pinpoint, intake, shooter, transit);
        shooter.setPitchState(Shooter.PitchState.LOW);
        shooter.periodic();
        intake.periodic();
        transit.periodic();
        driveCommand = new FieldCentricDriveCommand(drive, pinpoint, gamepad1, false);
        drive.setDefaultCommand(driveCommand);
        shotCommand = new SequencedShotCommand(intake, shooter, transit, gamepad1);
        intake.setDefaultCommand(shotCommand);
        telemetry.addLine("Left stick: field drive; right stick X: turn; left stick press: heading zero.");
        telemetry.addLine("D-pad up/down: pitch +/-0.01; left/right: target RPM -/+100.");
        telemetry.addLine("Left trigger: intake; left bumper: eject; right trigger: sequenced shot.");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        double targetRpm = shotCommand.getTargetRpm();
        telemetry.addData("OD", pinpoint.isReady() ? "READY" : "WAITING");
        telemetry.addData("Driver heading deg", "%.1f", driveCommand.getDriverHeadingDegrees());
        telemetry.addData("Pitch left / right", "%.2f / %.2f",
                shooter.getPitchPosition(), 1.0 - shooter.getPitchPosition());
        telemetry.addData("Shooter target RPM", "%.0f", targetRpm);
        telemetry.addData("Shooter left RPM", "%.0f", shooter.getLeftFlywheelRpm());
        telemetry.addData("Shooter RPM error / tolerance", "%.0f / %.0f",
                targetRpm - shooter.getLeftFlywheelRpm(), ShooterConstants.readyToleranceRpm);
        telemetry.addData("Shooter at target", shooter.isAtTargetRpm());
        telemetry.addData("Shooter commanded power", "%.2f", shooter.getCommandedFlywheelPower());
        telemetry.addData("Shot state", shotCommand.getShotState());
        telemetry.addData("Gate delay ms", shotCommand.getGateElapsedMs());
        telemetry.addData("Gate / transit", "%s / %s",
                transit.getGateState(), intake.getIntakeState());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            super.runOpMode();
        } finally {
            CommandScheduler.getInstance().reset();
            if (drive != null) drive.stop();
            if (shotCommand != null) shotCommand.end(true);
            if (intake != null) intake.periodic();
            if (shooter != null) shooter.periodic();
            if (transit != null) transit.periodic();
        }
    }
}
