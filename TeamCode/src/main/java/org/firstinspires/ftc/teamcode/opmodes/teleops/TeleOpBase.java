package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ReverseIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.TurretCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

/**
 * Shared TeleOp wiring, following the DECODE project's OpMode -> Commands ->
 * Subsystems structure. Concrete TeleOps can later add alliance-specific
 * vision and autonomous handoff without duplicating hardware construction.
 */
public abstract class TeleOpBase extends CommandOpMode {
    public Drive drive;
    public Intake intake;
    public Transit transit;
    public Turret turret;
    public Shooter shooter;
    public GamepadEx gamepadEx1;
    public GamepadEx gamepadEx2;

    @Override
    public void initialize() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        transit = new Transit(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        drive.setDefaultCommand(new DriveCommand(drive, gamepadEx1));
        // Manual D-pad override; release resumes the automatic default command.
        new FunctionalButton(() -> gamepad2.dpad_left || gamepad2.dpad_right)
                .whenHeld(new TurretCommand(turret, gamepadEx2));
        shooter.setDefaultCommand(new ShooterCommand(shooter, gamepadEx2));

        new FunctionalButton(() -> gamepadEx2.getTrigger(
                com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.5)
                .whenHeld(new IntakeCommand(intake, transit));

        new FunctionalButton(() -> gamepadEx2.getTrigger(
                com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5)
                .whenHeld(new TransitCommand(shooter, transit, intake));

        new FunctionalButton(() -> gamepadEx2.getButton(
                com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER))
                .whenHeld(new ReverseIntakeCommand(intake, transit));

        telemetry.addLine("BIOBUZZ Spinup initialized");
        telemetry.addLine("Verify servo directions and motor directions before enabling.");
        telemetry.update();
    }

    @Override
    public void run() {
        // Keep the explicit scheduler call to match the established DECODE TeleOpBase pattern.
        CommandScheduler.getInstance().run();

        telemetry.addLine("----- Drive -----");
        telemetry.addData("State", drive.getDriveState());
        telemetry.addLine("----- Intake / Transit -----");
        telemetry.addData("Intake", intake.getIntakeState());
        telemetry.addData("Gate", transit.getGateState());
        telemetry.addLine("----- Shooter -----");
        telemetry.addData("State", shooter.getShooterState());
        telemetry.addData("Pitch", shooter.getPitchState());
        telemetry.addData("Velocity ticks/s", shooter.getAverageVelocity());
        telemetry.addLine("----- Turret -----");
        telemetry.addData("Encoder ticks", turret.getTurretPos());
        telemetry.addData("Turret degrees", turret.getAngleDegrees());
        telemetry.addData("Target degrees", turret.getTargetDegrees());
        telemetry.update();
    }
}
