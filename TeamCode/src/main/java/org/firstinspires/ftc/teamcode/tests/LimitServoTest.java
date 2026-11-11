package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.LimitServoTrimCommand;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

@TeleOp(name = "Limit Servo Test", group = "Test")
public class LimitServoTest extends CommandOpMode {
    private Transit transit;
    private LimitServoTrimCommand command;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        transit = new Transit(hardwareMap);
        command = new LimitServoTrimCommand(transit, gamepad1);
        transit.setDefaultCommand(command);
        transit.periodic();
        telemetry.addLine("D-pad up: +0.01 | D-pad down: -0.01");
        telemetry.addData("Servo hardware name", TransitConstants.limitServoName);
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("Limit servo position", "%.2f", command.getPosition());
        telemetry.addData("Position range", "0.00 to 1.00");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try { super.runOpMode(); }
        finally { CommandScheduler.getInstance().reset(); }
    }
}
