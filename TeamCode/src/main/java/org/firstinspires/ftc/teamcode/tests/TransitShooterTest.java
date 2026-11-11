package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.TransitShooterManualCommand;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/** Independent motor, flywheel and gate test wiring and telemetry. */
@TeleOp(name = "Transit Shooter Test", group = "Test")
public class TransitShooterTest extends CommandOpMode {
    private Intake intake;
    private Shooter shooter;
    private Transit transit;
    private TransitShooterManualCommand command;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, false);
        transit = new Transit(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(shooter, transit);
        transit.periodic();
        command = new TransitShooterManualCommand(intake, shooter, transit, gamepad1);
        intake.setDefaultCommand(command);
        telemetry.addLine("Right trigger: shooter; X: intake +0.65; Y: eject -0.65; A: feed +1.00.");
        telemetry.addLine("Left trigger: open gate; release: close gate.");
        telemetry.addLine("D-pad up/down: target RPM +/-100; buttons can be held together.");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("Transit", intake.getIntakeState());
        telemetry.addData("Gate", transit.getGateState());
        telemetry.addData("Shooter target RPM", "%.0f", command.getTargetRpm());
        telemetry.addData("Shooter left encoder RPM", "%.0f", shooter.getLeftFlywheelRpm());
        telemetry.addData("Shooter at target", shooter.isAtTargetRpm());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try { super.runOpMode(); }
        finally {
            CommandScheduler.getInstance().reset();
            if (command != null) command.end(true);
            if (intake != null) intake.periodic();
            if (shooter != null) shooter.periodic();
            if (transit != null) transit.periodic();
        }
    }
}
