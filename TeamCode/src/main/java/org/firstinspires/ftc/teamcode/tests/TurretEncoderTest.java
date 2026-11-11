package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.TurretEncoderTestCommand;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@TeleOp(name = "Turret Encoder Test", group = "Test")
public class TurretEncoderTest extends CommandOpMode {
    private Turret turret;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        turret = new Turret(hardwareMap);
        turret.setDefaultCommand(new TurretEncoderTestCommand(turret, gamepad1));
        telemetry.addLine("Point turret forward before INIT: encoder zero is set in init.");
        telemetry.addLine("Gamepad 1 left stick X controls turret power.");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("Motor Encoder Ticks", turret.getEncoderTicks());
        telemetry.addData("Turret Angle", "%.2f deg", turret.getAngleDegrees());
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
