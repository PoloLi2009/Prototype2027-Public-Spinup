package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

@TeleOp(name = "Limit Servo Test", group = "Test")
public class LimitServoTest extends OpMode {
    private static final double STEP = 0.01;

    private Servo limitServo;
    private double position;
    private boolean wasDpadUpPressed;
    private boolean wasDpadDownPressed;

    @Override
    public void init() {
        limitServo = hardwareMap.get(Servo.class, TransitConstants.gateServoName);
        position = TransitConstants.gateClosedPosition;
        limitServo.setPosition(position);
        telemetry.addLine("D-pad up: +0.01 | D-pad down: -0.01");
        telemetry.addData("Servo hardware name", TransitConstants.gateServoName);
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !wasDpadUpPressed) {
            position = clamp(position + STEP);
        }
        if (gamepad1.dpad_down && !wasDpadDownPressed) {
            position = clamp(position - STEP);
        }
        wasDpadUpPressed = gamepad1.dpad_up;
        wasDpadDownPressed = gamepad1.dpad_down;

        limitServo.setPosition(position);
        telemetry.addData("Limit servo position", "%.2f", position);
        telemetry.addData("Position range", "0.00 to 1.00");
        telemetry.update();
    }

    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
