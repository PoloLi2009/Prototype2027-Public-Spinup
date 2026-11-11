package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;

/** Drives in the field direction selected by the left stick. */
public class FieldCentricDriveCommand extends CommandBase {
    private final Drive drive;
    private final Pinpoint pinpoint;
    private final DoubleSupplier forwardAxis;
    private final DoubleSupplier strafeAxis;
    private final DoubleSupplier turnAxis;
    private final BooleanSupplier resetHeadingRequested;
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
        this(drive, pinpoint, () -> gamepad.left_stick_y, () -> gamepad.left_stick_x,
                () -> gamepad.right_stick_x, () -> gamepad.left_stick_button, requireFieldPose);
    }

    /** Live control inputs; axes retain the original command signs and deadbands. */
    public FieldCentricDriveCommand(Drive drive, Pinpoint pinpoint, DoubleSupplier forwardAxis,
            DoubleSupplier strafeAxis, DoubleSupplier turnAxis, BooleanSupplier resetHeadingRequested,
                                    boolean requireFieldPose) {
        this.drive = drive;
        this.pinpoint = pinpoint;
        this.forwardAxis = Objects.requireNonNull(forwardAxis);
        this.strafeAxis = Objects.requireNonNull(strafeAxis);
        this.turnAxis = Objects.requireNonNull(turnAxis);
        this.resetHeadingRequested = Objects.requireNonNull(resetHeadingRequested);
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
            lastLeftStickButton = resetHeadingRequested.getAsBoolean();
            headingZeroInitialized = true;
        }
        if (resetHeadingRequested.getAsBoolean() && !lastLeftStickButton) {
            driverHeadingZeroDegrees = headingDegrees;
        }
        lastLeftStickButton = resetHeadingRequested.getAsBoolean();
        driverHeadingDegrees = headingDegrees - driverHeadingZeroDegrees;

        drive.moveFieldCentric(-deadband(forwardAxis.getAsDouble()),
                deadband(strafeAxis.getAsDouble()), -deadband(turnAxis.getAsDouble()),
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
