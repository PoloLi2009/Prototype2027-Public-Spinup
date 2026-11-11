package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.TurretLimitCommand;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

/** Turret unwind test wiring and telemetry. */
@TeleOp(name = "Turret Limit Test", group = "Test")
public class TurretLimitTest extends CommandOpMode {
    private Turret turret;
    private TurretLimitCommand command;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        turret = new Turret(hardwareMap);
        command = new TurretLimitCommand(turret, gamepad1);
        turret.setDefaultCommand(command);
        telemetry.addLine("Point turret straight forward BEFORE INIT; this resets to 0 deg.");
        telemetry.addLine("Gamepad 1 left stick X: turn turret; at +/-180 deg: auto unwind.");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("Mode", command.getMode());
        telemetry.addData("Turret angle deg", "%.2f", turret.getAngleDegrees());
        telemetry.addData("Motor encoder ticks", turret.getEncoderTicks());
        telemetry.addData("Rewind target deg", command.isRewinding()
                ? command.getRewindTargetDegrees() : "--");
        telemetry.addData("Rewind elapsed sec", command.isRewinding()
                ? String.format("%.2f", command.getRewindElapsedSeconds()) : "--");
        telemetry.addData("Min / Max deg", "%.1f / %.1f",
                TurretConstants.minDegrees(), TurretConstants.maxDegrees());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try { super.runOpMode(); }
        finally {
            CommandScheduler.getInstance().reset();
            if (turret != null) turret.shutdown();
        }
    }
}
