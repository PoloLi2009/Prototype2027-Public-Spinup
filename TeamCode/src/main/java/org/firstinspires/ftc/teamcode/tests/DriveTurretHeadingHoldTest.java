package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TurretHeadingHoldCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

/** Hardware wiring and telemetry for field-centric drive and turret heading hold. */
@TeleOp(name = "Drive Turret Heading Hold Test", group = "Test")
public class DriveTurretHeadingHoldTest extends CommandOpMode {
    private Drive drive;
    private Turret turret;
    private Pinpoint pinpoint;
    private FieldCentricDriveCommand driveCommand;
    private TurretHeadingHoldCommand holdCommand;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        drive = new Drive(hardwareMap);
        turret = new Turret(hardwareMap);
        pinpoint = new Pinpoint(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(pinpoint);

        driveCommand = new FieldCentricDriveCommand(drive, pinpoint, gamepad1);
        drive.setDefaultCommand(driveCommand);
        holdCommand = new TurretHeadingHoldCommand(turret, pinpoint, gamepad1);
        turret.setDefaultCommand(holdCommand);

        telemetry.addLine("Point turret forward before INIT; encoder zero is set at INIT.");
        telemetry.addLine("Left stick: field-relative drive; right stick X: turn chassis.");
        telemetry.addLine("Left stick press: zero driver heading; right stick press: aim forward and hold.");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        double targetDegrees = holdCommand.getTargetDegrees();
        telemetry.addData("OD", pinpoint.isReady() ? "READY" : "WAITING");
        telemetry.addData("OD / driver heading deg", "%.2f / %.2f",
                pinpoint.getHeadingDegrees(), driveCommand.getDriverHeadingDegrees());
        telemetry.addData("Turret angle deg", "%.2f", turret.getAngleDegrees());
        telemetry.addData("Turret target deg", "%.2f", targetDegrees);
        telemetry.addData("Turret error deg", "%.2f", targetDegrees - turret.getAngleDegrees());
        telemetry.addData("Turret encoder ticks", turret.getEncoderTicks());
        telemetry.addData("Heading hold", holdCommand.isTracking() ? "ON" : "WAITING / UNREACHABLE");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            super.runOpMode();
        } finally {
            CommandScheduler.getInstance().reset();
            if (drive != null) drive.stop();
            if (turret != null) turret.stop();
        }
    }
}
