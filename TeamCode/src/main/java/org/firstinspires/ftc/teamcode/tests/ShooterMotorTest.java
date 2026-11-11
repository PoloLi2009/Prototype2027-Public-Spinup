package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.ShooterMotorTestCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

@TeleOp(name = "Shooter Motor Test", group = "Test")
public class ShooterMotorTest extends CommandOpMode {
    private Shooter shooter;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        shooter = new Shooter(hardwareMap, false);
        shooter.setDefaultCommand(new ShooterMotorTestCommand(shooter));
        telemetry.addLine("Press START: both shooter motors run at full power.");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try { super.runOpMode(); }
        finally {
            CommandScheduler.getInstance().reset();
            if (shooter != null) { shooter.stop(); shooter.periodic(); }
        }
    }
}
