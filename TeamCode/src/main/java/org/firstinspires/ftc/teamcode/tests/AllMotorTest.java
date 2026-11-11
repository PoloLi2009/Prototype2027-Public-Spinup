package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.commands.AllMotorTestCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@TeleOp(name = "All Motor Test", group = "Test")
public class AllMotorTest extends CommandOpMode {
    private AllMotorTestCommand motorTest;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        Drive drive = new Drive(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        DcMotor leftFlywheel = configureMotor(ShooterConstants.leftShooterName,
                DcMotorSimple.Direction.FORWARD);
        DcMotor rightFlywheel = configureMotor(ShooterConstants.rightShooterName,
                DcMotorSimple.Direction.REVERSE);
        DcMotor transit = configureMotor(IntakeConstants.transitMotorName,
                DcMotorSimple.Direction.FORWARD);
        motorTest = new AllMotorTestCommand(drive, turret, leftFlywheel, rightFlywheel, transit, gamepad1);
        drive.setDefaultCommand(motorTest);

        telemetry.addLine("D-pad left/right: choose group. Hold A: pulse selected group.");
        telemetry.addLine("Hold both triggers: pulse all eight motors. Release to repeat.");
        telemetry.update();
    }

    private DcMotor configureMotor(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setPower(0.0);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("Selected", motorTest.getSelectedGroup());
        telemetry.addData("Running", motorTest.getActiveGroup());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            super.runOpMode();
        } finally {
            CommandScheduler.getInstance().reset();
            if (motorTest != null) motorTest.end(true);
        }
    }
}
