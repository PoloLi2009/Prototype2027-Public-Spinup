package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

/**
 * Overtune Blue manual-drive logic (local master 5045231), adapted only at the hardware boundary.
 * Keeps Spinup's names, motor directions, deadband and drive calibration. The optional turn
 * callback is Solo's existing aim control; with no aim request it returns the Blue manual turn.
 */
public class BlueManualDriveCommand extends CommandBase {
    private final Drive drive;
    private final DoubleSupplier forwardInput, strafeInput, turnInput, headingRadians;
    private final BooleanSupplier odometryReady;
    private final DoubleUnaryOperator aimTurn;
    private double driverHeadingReferenceRadians = Double.NaN;

    public BlueManualDriveCommand(Drive drive, GamepadEx gamepad,
            DoubleSupplier headingRadians, BooleanSupplier odometryReady, DoubleUnaryOperator aimTurn) {
        this(drive, gamepad::getLeftY, gamepad::getLeftX, gamepad::getRightX,
                headingRadians, odometryReady, aimTurn);
    }

    public BlueManualDriveCommand(Drive drive, DoubleSupplier forwardInput,
            DoubleSupplier strafeInput, DoubleSupplier turnInput, DoubleSupplier headingRadians,
            BooleanSupplier odometryReady, DoubleUnaryOperator aimTurn) {
        this.drive = drive;
        this.forwardInput = forwardInput;
        this.strafeInput = strafeInput;
        this.turnInput = turnInput;
        this.headingRadians = headingRadians;
        this.odometryReady = odometryReady;
        this.aimTurn = aimTurn;
        addRequirements(drive);
    }

    @Override public void initialize() { resetDriverHeading(); }

    @Override public void execute() {
        // Equivalent to Blue Drive.periodic's reference initialization/recovery.
        if (!odometryReady.getAsBoolean()) driverHeadingReferenceRadians = Double.NaN;
        else if (!Double.isFinite(driverHeadingReferenceRadians)) resetDriverHeading();
        double forward = applyDeadband(forwardInput.getAsDouble());
        double strafe = applyDeadband(strafeInput.getAsDouble());
        double turn = applyDeadband(turnInput.getAsDouble());
        moveFieldCentric(forward, strafe, aimTurn.applyAsDouble(turn));
    }

    public void resetDriverHeading() {
        driverHeadingReferenceRadians = odometryReady.getAsBoolean()
                ? headingRadians.getAsDouble() : Double.NaN;
    }

    public double getDriverHeadingRadians() {
        if (!odometryReady.getAsBoolean() || !Double.isFinite(driverHeadingReferenceRadians)) {
            return Double.NaN;
        }
        return Math.IEEEremainder(headingRadians.getAsDouble() - driverHeadingReferenceRadians, 2.0 * Math.PI);
    }

    /** Overtune Drive.updateVisionPose: a field-heading correction must not move the driver's frame. */
    public void shiftDriverHeadingReference(double correctionRadians) {
        if (Double.isFinite(driverHeadingReferenceRadians) && Double.isFinite(correctionRadians)) {
            driverHeadingReferenceRadians = Math.IEEEremainder(
                    driverHeadingReferenceRadians + correctionRadians, 2.0 * Math.PI);
        }
    }

    /** Public for the ported Overtune ShootAssistCommand (Overtune Drive.moveFieldCentric). */
    public void moveFieldCentric(double forward, double strafe, double turn) {
        if (!Double.isFinite(forward) || !Double.isFinite(strafe) || !Double.isFinite(turn)) {
            drive.stop();
            return;
        }
        double heading = getDriverHeadingRadians();
        if (!Double.isFinite(heading)) {
            moveRobot(forward, strafe, turn);
            return;
        }
        // Copied from Overtune Blue's Drive.moveFieldCentric.
        double robotForward = forward * Math.cos(heading) - strafe * Math.sin(heading);
        double robotStrafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        moveRobot(robotForward, robotStrafe, turn);
    }

    /** Public for the ported Overtune ShootAssistCommand (Overtune Drive.moveRobot). */
    public void moveRobot(double forward, double strafe, double turn) {
        // Spinup's existing lateral calibration is the only mixer parameter adaptation.
        double rotX = strafe * DriveConstants.strafingBalance;
        double rotY = forward;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double leftFrontPower = (rotY + rotX + turn) / denominator;
        double leftBackPower = (rotY - rotX + turn) / denominator;
        double rightFrontPower = (rotY - rotX - turn) / denominator;
        double rightBackPower = (rotY + rotX - turn) / denominator;
        drive.setWheelPowers(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    private double applyDeadband(double input) {
        return Math.abs(input) > DriveConstants.joystickDeadband ? input : 0.0;
    }

    @Override public void end(boolean interrupted) { drive.stop(); }
}
