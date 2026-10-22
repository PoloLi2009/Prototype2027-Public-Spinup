package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

/** Default drive command, deliberately named and shaped like the DECODE counterpart. */
public class DriveCommand extends CommandBase {
    private final Drive drive;
    private final GamepadEx gamepadEx;

    public DriveCommand(Drive drive, GamepadEx gamepadEx) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double forward = -gamepadEx.getLeftY();
        double strafe = gamepadEx.getLeftX();
        double turn = gamepadEx.getRightX();
        boolean active = Math.abs(forward) > DriveConstants.joystickDeadband
                || Math.abs(strafe) > DriveConstants.joystickDeadband
                || Math.abs(turn) > DriveConstants.joystickDeadband;

        if (active) {
            drive.setDriveState(Drive.DriveState.TELEOP);
            drive.moveRobot(forward, strafe, turn);
        } else {
            drive.setDriveState(Drive.DriveState.STOP);
            drive.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
