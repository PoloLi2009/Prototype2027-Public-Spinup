package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDriveCommand;
import org.firstinspires.ftc.teamcode.commands.OdCellAimCommand;
import org.firstinspires.ftc.teamcode.commands.SequencedShotCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.field.CellTarget;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.localization.PinpointConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

/** Field drive, OD CELL aim and RPM-qualified shooting without vision. */
@TeleOp(name = "Field Centric OD Cell Shooter Test", group = "Test")
public class FieldCentricOdCellShooterTest extends CommandOpMode {
    private Drive drive;
    private Pinpoint pinpoint;
    private Turret turret;
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private FieldCentricDriveCommand driveCommand;
    private OdCellAimCommand aimCommand;
    private SequencedShotCommand shotCommand;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        drive = new Drive(hardwareMap);
        pinpoint = new Pinpoint(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transit = new Transit(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(pinpoint, intake, shooter, transit);
        shooter.setPitchState(Shooter.PitchState.LOW);
        shooter.periodic();
        intake.periodic();
        transit.periodic();

        driveCommand = new FieldCentricDriveCommand(drive, pinpoint, gamepad1);
        drive.setDefaultCommand(driveCommand);
        aimCommand = new OdCellAimCommand(turret, pinpoint, gamepad1,
                PinpointConstants.testStartFieldXmm, PinpointConstants.testStartFieldYmm,
                PinpointConstants.testStartFieldHeadingDegrees,
                true, () -> gamepad1.right_trigger >= 0.5);
        turret.setDefaultCommand(aimCommand);
        shotCommand = new SequencedShotCommand(intake, shooter, transit, gamepad1,
                3000.0, () -> aimCommand.isOnTarget() && pinpoint.isReady(),
                Intake.IntakeState.FAST_FORWARD, Intake.IntakeState.FAST_REVERSED);
        intake.setDefaultCommand(shotCommand);

        telemetry.addLine("Point turret forward BEFORE INIT; its encoder zero is set now.");
        telemetry.addLine("Set PinpointConstants testStartField pose to the real START pose.");
        telemetry.addData("Configured START pose mm/deg", "%.1f / %.1f / %.1f",
                PinpointConstants.testStartFieldXmm, PinpointConstants.testStartFieldYmm,
                PinpointConstants.testStartFieldHeadingDegrees);
        telemetry.addLine("Y: BLUE/RED; left trigger: intake 0.85; left bumper: eject 0.85.");
        telemetry.addLine("Right trigger: aim, spin to RPM, open gate, then feed at 1.0.");
        telemetry.addLine("D-pad up/down: pitch; left/right: RPM -/+100.");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        CellTarget target = aimCommand.getTarget();
        telemetry.addData("OD", pinpoint.isReady() ? "READY" : "WAITING");
        telemetry.addData("Alliance / CELL", "%s / %s",
                aimCommand.isBlueAlliance() ? "BLUE" : "RED",
                target == null ? "UNAVAILABLE" : target.name());
        telemetry.addData("OD X / Y / heading", "%.1f / %.1f / %.1f",
                pinpoint.getXmm(), pinpoint.getYmm(), pinpoint.getHeadingDegrees());
        telemetry.addData("Turret angle / target", "%.1f / %.1f",
                turret.getAngleDegrees(), aimCommand.getTargetDegrees());
        telemetry.addData("Turret on target", aimCommand.isOnTarget());
        telemetry.addData("Shooter target / left RPM", "%.0f / %.0f",
                shotCommand.getTargetRpm(), shooter.getLeftFlywheelRpm());
        telemetry.addData("Shooter tolerance", "%.0f", ShooterConstants.readyToleranceRpm);
        telemetry.addData("Shooter at target", shooter.isAtTargetRpm());
        telemetry.addData("Shooter power", "%.2f", shooter.getCommandedFlywheelPower());
        telemetry.addData("Shot state / gate delay ms", "%s / %d",
                shotCommand.getShotState(), shotCommand.getGateElapsedMs());
        telemetry.addData("Gate / transit", "%s / %s",
                transit.getGateState(), intake.getIntakeState());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try { super.runOpMode(); }
        finally {
            CommandScheduler.getInstance().reset();
            if (drive != null) drive.stop();
            if (aimCommand != null) aimCommand.end(true);
            if (shotCommand != null) shotCommand.end(true);
            if (intake != null) intake.periodic();
            if (shooter != null) shooter.periodic();
            if (transit != null) transit.periodic();
        }
    }
}
