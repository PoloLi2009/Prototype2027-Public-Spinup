package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.PitchServoTrimCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.PitchServos;

@TeleOp(name = "Pitch Servo Trim Test", group = "Test")
public class PitchServoTrimTest extends CommandOpMode {
    private PitchServos servos;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        servos = new PitchServos(hardwareMap);
        servos.setDefaultCommand(new PitchServoTrimCommand(servos, gamepad1));
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addLine("D-pad up/down: left +/-0.01; right/left: right +/-0.01");
        telemetry.addLine("X: left +0.01, right -0.01; Y: left -0.01, right +0.01");
        telemetry.addData("Left target (not feedback)", "%.2f", servos.getLeftPosition());
        telemetry.addData("Right target (not feedback)", "%.2f", servos.getRightPosition());
        telemetry.addData("Left SDK value", "%.2f", servos.getLeftSdkPosition());
        telemetry.addData("Right SDK value", "%.2f", servos.getRightSdkPosition());
        telemetry.addLine("Actual angles unavailable from 3-wire PWM servos");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try { super.runOpMode(); }
        finally { CommandScheduler.getInstance().reset(); }
    }
}
