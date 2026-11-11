package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;

/** Drives in the field direction selected by the left stick. */
public class FieldCentricDriveCommand extends CommandBase {
    private final Drive drive;
    private final Pinpoint pinpoint;
    private final Gamepad gamepad;
    private final boolean requireFieldPose;
    private boolean headingZeroInitialized;
    private boolean lastLeftStickButton;
    private double driverHeadingZeroDegrees;
    private double driverHeadingDegrees;

    public FieldCentricDriveCommand(Drive drive, Pinpoint pinpoint, Gamepad gamepad) {
        this(drive, pinpoint, gamepad, true);
    }

    public FieldCentricDriveCommand(Drive drive, Pinpoint pinpoint, Gamepad gamepad,
                                    boolean requireFieldPose) {
        this.drive = drive;
        this.pinpoint = pinpoint;
        this.gamepad = gamepad;
        this.requireFieldPose = requireFieldPose;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (!pinpoint.isReady() || (requireFieldPose && !pinpoint.isFieldPoseInitialized())) {
            drive.stop();
            return;
        }
        double headingDegrees = pinpoint.getHeadingDegrees();
        if (!Double.isFinite(headingDegrees)) {
            drive.stop();
            return;
        }
        if (!headingZeroInitialized) {
            driverHeadingZeroDegrees = headingDegrees;
            lastLeftStickButton = gamepad.left_stick_button;
            headingZeroInitialized = true;
        }
        if (gamepad.left_stick_button && !lastLeftStickButton) {
            driverHeadingZeroDegrees = headingDegrees;
        }
        lastLeftStickButton = gamepad.left_stick_button;
        driverHeadingDegrees = headingDegrees - driverHeadingZeroDegrees;

        drive.moveFieldCentric(-deadband(gamepad.left_stick_y),
                deadband(gamepad.left_stick_x), -deadband(gamepad.right_stick_x),
                driverHeadingDegrees);
    }

    private double deadband(double value) {
        return Math.abs(value) > DriveConstants.joystickDeadband ? value : 0.0;
    }

    public double getDriverHeadingDegrees() {
        return driverHeadingDegrees;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
