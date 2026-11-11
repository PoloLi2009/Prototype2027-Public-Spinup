package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

/** Test-compatible field drive with optional aim turn override; only this command writes Drive. */
public class DriveWithAimCommand extends CommandBase {
    private final Drive drive;
    private final DoubleSupplier forwardAxis, strafeAxis, turnAxis, scale, headingDegrees;
    private final BooleanSupplier fieldActive;
    private final DoubleUnaryOperator aimTurn;

    public DriveWithAimCommand(Drive drive, DoubleSupplier forwardAxis, DoubleSupplier strafeAxis,
            DoubleSupplier turnAxis, DoubleSupplier scale, BooleanSupplier fieldActive,
            DoubleSupplier headingDegrees, DoubleUnaryOperator aimTurn) {
        this.drive = drive;
        this.forwardAxis = forwardAxis;
        this.strafeAxis = strafeAxis;
        this.turnAxis = turnAxis;
        this.scale = scale;
        this.fieldActive = fieldActive;
        this.headingDegrees = headingDegrees;
        this.aimTurn = aimTurn;
        addRequirements(drive);
    }

    @Override public void execute() {
        double powerScale = scale.getAsDouble();
        double forward = -deadband(forwardAxis.getAsDouble()) * powerScale;
        double strafe = deadband(strafeAxis.getAsDouble()) * powerScale;
        double turn = aimTurn.applyAsDouble(-deadband(turnAxis.getAsDouble()) * powerScale);
        if (fieldActive.getAsBoolean()) {
            // Exactly the VisionCellChassisAimCommand field axes and absolute OD heading.
            drive.moveFieldCentric(forward, strafe, turn, headingDegrees.getAsDouble());
        } else {
            drive.moveRobot(-forward, strafe, turn);
        }
    }

    private static double deadband(double value) {
        return Math.abs(value) > DriveConstants.joystickDeadband ? value : 0.0;
    }
    @Override public void end(boolean interrupted) { drive.stop(); }
}
