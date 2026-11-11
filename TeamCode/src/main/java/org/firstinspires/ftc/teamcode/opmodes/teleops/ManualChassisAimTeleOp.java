package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.ManualChassisAimCommand;
import org.firstinspires.ftc.teamcode.commands.SequencedShotCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.utils.FieldTargetConstants;

/** Basic chassis CELL aim and mechanism test wiring and telemetry. */
@TeleOp(name = "Manual Chassis Aim Test", group = "Test")
public class ManualChassisAimTeleOp extends CommandOpMode {
    private Drive drive;
    private Intake intake;
    private Transit transit;
    private Shooter shooter;
    private Pinpoint pinpoint;
    private ManualChassisAimCommand aimCommand;
    private SequencedShotCommand shotCommand;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        transit = new Transit(hardwareMap);
        shooter = new Shooter(hardwareMap);
        pinpoint = new Pinpoint(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(pinpoint, intake, transit, shooter);
        aimCommand = new ManualChassisAimCommand(drive, pinpoint, gamepad1);
        drive.setDefaultCommand(aimCommand);
        shotCommand = new SequencedShotCommand(intake, shooter, transit, gamepad1,
                ShooterConstants.targetRpm);
        intake.setDefaultCommand(shotCommand);
        transit.periodic();
        telemetry.addLine("A: set current robot pose to field center, heading 0 degrees.");
        telemetry.addLine("X: switch BLUE/RED. Release right stick X for chassis auto-aim.");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        double[] target = aimCommand.getTarget();
        telemetry.addData("Alliance", aimCommand.getAlliance());
        telemetry.addData("Pinpoint X / Y mm", "%.1f / %.1f",
                pinpoint.getXmm(), pinpoint.getYmm());
        telemetry.addData("Pinpoint heading", "%.1f deg", pinpoint.getHeadingDegrees());
        telemetry.addData("Half / auto aim", "%s / %s",
                FieldTargetConstants.halfName(pinpoint.getYmm()),
                aimCommand.isAutoAiming() ? "ON" : "MANUAL");
        telemetry.addData("Target X / Y mm", target == null ? "UNCONFIGURED"
                : String.format("%.1f / %.1f", target[0], target[1]));
        telemetry.addData("Shot state", shotCommand.getShotState());
        telemetry.addData("Flywheel target / left RPM", "%.0f / %.0f",
                shotCommand.getTargetRpm(), shooter.getRightFlywheelRpm());
        telemetry.addData("Flywheel ready", shooter.isAtTargetRpm());
        telemetry.addData("Pitch command", "%.2f", shooter.getPitchPosition());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try { super.runOpMode(); }
        finally {
            CommandScheduler.getInstance().reset();
            if (aimCommand != null) aimCommand.end(true);
            if (shotCommand != null) shotCommand.end(true);
            if (intake != null) intake.periodic();
            if (transit != null) transit.periodic();
            if (shooter != null) shooter.periodic();
        }
    }
}
