package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

/** Isolates the four drivetrain motors for a manual hardware check. */
@TeleOp(name = "Drive Only Test", group = "Test")
public class DriveOnlyTest extends CommandOpMode {
    private Drive drive;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        drive = new Drive(hardwareMap);
        drive.setDefaultCommand(new DriveCommand(drive, new GamepadEx(gamepad1)));
        telemetry.addLine("Drive motors initialized. Left stick: move. Right X: turn.");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("Drive", drive.getDriveState());
        telemetry.addData("Motor power LF/RF/LB/RB", "%.2f / %.2f / %.2f / %.2f",
                drive.leftFrontMotor.getPower(), drive.rightFrontMotor.getPower(),
                drive.leftBackMotor.getPower(), drive.rightBackMotor.getPower());
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            super.runOpMode();
        } finally {
            CommandScheduler.getInstance().reset();
            if (drive != null) drive.stop();
        }
    }
}
