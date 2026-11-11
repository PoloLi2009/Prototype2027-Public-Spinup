package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.FieldCentricDriveCommand;
import org.firstinspires.ftc.teamcode.commands.OdCellAimCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.field.CellTarget;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.localization.PinpointConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

/** Hardware wiring and telemetry for the OD CELL aim commands. */
@TeleOp(name = "OD Cell Turret Aim Test", group = "Test")
public class OdCellTurretAimTest extends CommandOpMode {
    private Drive drive;
    private Turret turret;
    private Pinpoint pinpoint;
    private FieldCentricDriveCommand driveCommand;
    private OdCellAimCommand aimCommand;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        drive = new Drive(hardwareMap);
        turret = new Turret(hardwareMap);
        pinpoint = new Pinpoint(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(pinpoint);

        driveCommand = new FieldCentricDriveCommand(drive, pinpoint, gamepad1);
        drive.setDefaultCommand(driveCommand);
        aimCommand = new OdCellAimCommand(turret, pinpoint, gamepad1,
                PinpointConstants.testStartFieldXmm, PinpointConstants.testStartFieldYmm,
                PinpointConstants.testStartFieldHeadingDegrees);
        turret.setDefaultCommand(aimCommand);

        telemetry.addLine("Point turret forward before INIT. Keep robot still until START.");
        telemetry.addLine("Set PinpointConstants testStartField pose to the real START pose.");
        telemetry.addData("START pose mm/deg", "x=%.1f y=%.1f h=%.1f",
                PinpointConstants.testStartFieldXmm, PinpointConstants.testStartFieldYmm,
                PinpointConstants.testStartFieldHeadingDegrees);
        telemetry.addLine("X: BLUE/RED; left stick press: reset driver heading.");
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
        telemetry.addData("OD pose mm/deg", "x=%.1f y=%.1f h=%.1f",
                pinpoint.getXmm(), pinpoint.getYmm(), pinpoint.getHeadingDegrees());
        telemetry.addData("Driver heading deg", "%.1f",
                driveCommand.getDriverHeadingDegrees());
        telemetry.addData("CELL target mm", target == null ? "UNAVAILABLE"
                : String.format("x=%.1f y=%.1f", target.xMm, target.yMm));
        telemetry.addData("Turret angle / target", "%.1f / %.1f",
                turret.getAngleDegrees(), aimCommand.getTargetDegrees());
        telemetry.addData("Turret error deg", "%.1f",
                aimCommand.getTargetDegrees() - turret.getAngleDegrees());
        telemetry.addData("Auto aim", aimCommand.isTracking() ? "ON" : "WAITING / UNREACHABLE");
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
