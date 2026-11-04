package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Shooter Motor Test", group = "Test")
public class ShooterMotorTest extends OpMode {
    private DcMotorEx leftShooterMotor;
    private DcMotorEx rightShooterMotor;

    @Override
    public void init() {
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftFlywheelMotor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightFlywheelMotor");

        leftShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Press START: both shooter motors run at full power.");
        telemetry.update();
    }

    @Override
    public void loop() {
        leftShooterMotor.setPower(1.0);
        rightShooterMotor.setPower(1.0);
    }

    @Override
    public void stop() {
        leftShooterMotor.setPower(0.0);
        rightShooterMotor.setPower(0.0);
    }
}
