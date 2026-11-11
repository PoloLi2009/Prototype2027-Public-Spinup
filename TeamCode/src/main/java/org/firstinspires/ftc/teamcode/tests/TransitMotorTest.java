package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.TransitMotorTestCommand;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;

/** Tests the motor shared by intake and transit. */
@TeleOp(name = "Transit Motor Test", group = "Test")
public class TransitMotorTest extends CommandOpMode {
    private Intake intake;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        intake = new Intake(hardwareMap);
        intake.setDefaultCommand(new TransitMotorTestCommand(intake, gamepad1));
        telemetry.addData("Hardware name", IntakeConstants.transitMotorName);
        telemetry.addLine("Hold X: forward; hold Y: reverse; release: stop.");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("Transit motor", intake.getIntakeState());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try { super.runOpMode(); }
        finally {
            CommandScheduler.getInstance().reset();
            if (intake != null) { intake.stop(); intake.periodic(); }
        }
    }
}
