package org.firstinspires.ftc.teamcode.commands;

import java.util.function.DoubleSupplier;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

/** Default drive command, deliberately named and shaped like the DECODE counterpart. */
public class DriveCommand extends CommandBase {
    private final Drive drive;
    private final DoubleSupplier forward, strafe, turn;

    public DriveCommand(Drive drive, GamepadEx gamepadEx) {
        this(drive, () -> -gamepadEx.getLeftY(), gamepadEx::getLeftX, gamepadEx::getRightX);
    }

    /** Robot-relative inputs in the same signs as Drive.moveRobot. */
    public DriveCommand(Drive drive, DoubleSupplier forward,
                        DoubleSupplier strafe, DoubleSupplier turn) {
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double forward = this.forward.getAsDouble();
        double strafe = this.strafe.getAsDouble();
        double turn = this.turn.getAsDouble();
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
