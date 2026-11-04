package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@TeleOp(name = "Turret Encoder Test", group = "Test")
public class TurretEncoderTest extends OpMode {
    private Turret turret;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        telemetry.addLine("Point turret forward before INIT: encoder zero is set in init.");
        telemetry.addLine("Gamepad 1 left stick X controls turret power.");
        telemetry.update();
    }

    @Override
    public void loop() {
        turret.setPower(gamepad1.left_stick_x);
        telemetry.addData("Motor Encoder Ticks", turret.getEncoderTicks());
        telemetry.addData("Turret Angle", "%.2f deg", turret.getAngleDegrees());
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
    }
}
