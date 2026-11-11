package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.utils.FieldTargetConstants;

/** Basic robot-centric chassis drive with OD turning toward the selected CELL. */
public class ManualChassisAimCommand extends CommandBase {
    private final Drive drive;
    private final Pinpoint pinpoint;
    private final DoubleSupplier forwardAxis;
    private final DoubleSupplier strafeAxis;
    private final DoubleSupplier turnAxis;
    private final BooleanSupplier toggleAllianceRequested;
    private final BooleanSupplier resetPoseRequested;
    private FieldTargetConstants.Alliance alliance = FieldTargetConstants.Alliance.BLUE;
    private double[] target;
    private boolean autoAiming;
    private boolean lastX, lastA;

    public ManualChassisAimCommand(Drive drive, Pinpoint pinpoint, Gamepad gamepad) {
        this(drive, pinpoint, () -> gamepad.left_stick_y, () -> gamepad.left_stick_x,
                () -> gamepad.right_stick_x, () -> gamepad.x, () -> gamepad.a);
    }

    /** Live control inputs; axes retain the original command signs and deadbands. */
    public ManualChassisAimCommand(Drive drive, Pinpoint pinpoint, DoubleSupplier forwardAxis,
            DoubleSupplier strafeAxis, DoubleSupplier turnAxis, BooleanSupplier toggleAllianceRequested,
            BooleanSupplier resetPoseRequested) {
        this.drive = drive;
        this.pinpoint = pinpoint;
        this.forwardAxis = Objects.requireNonNull(forwardAxis);
        this.strafeAxis = Objects.requireNonNull(strafeAxis);
        this.turnAxis = Objects.requireNonNull(turnAxis);
        this.toggleAllianceRequested = Objects.requireNonNull(toggleAllianceRequested);
        this.resetPoseRequested = Objects.requireNonNull(resetPoseRequested);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (toggleAllianceRequested.getAsBoolean() && !lastX) {
            alliance = alliance == FieldTargetConstants.Alliance.BLUE
                    ? FieldTargetConstants.Alliance.RED : FieldTargetConstants.Alliance.BLUE;
        }
        if (resetPoseRequested.getAsBoolean() && !lastA) pinpoint.resetFieldPose();
        lastX = toggleAllianceRequested.getAsBoolean();
        lastA = resetPoseRequested.getAsBoolean();

        target = FieldTargetConstants.target(alliance, pinpoint.getYmm());
        double turn = turnAxis.getAsDouble();
        autoAiming = Math.abs(turn) <= DriveConstants.joystickDeadband
                && target != null && pinpoint.isReady()
                && Double.isFinite(pinpoint.getXmm())
                && Double.isFinite(pinpoint.getYmm())
                && Double.isFinite(pinpoint.getHeadingDegrees());
        if (autoAiming) {
            double heading = Math.toDegrees(Math.atan2(target[1] - pinpoint.getYmm(),
                    target[0] - pinpoint.getXmm()));
            double error = normalizeDegrees(heading - pinpoint.getHeadingDegrees());
            turn = Range.clip(error * FieldTargetConstants.chassisAimKp,
                    -FieldTargetConstants.maxChassisAimPower,
                    FieldTargetConstants.maxChassisAimPower);
        }
        drive.moveRobot(-forwardAxis.getAsDouble(), strafeAxis.getAsDouble(), turn);
    }

    private double normalizeDegrees(double degrees) {
        double wrapped = degrees % 360.0;
        return wrapped > 180.0 ? wrapped - 360.0
                : wrapped < -180.0 ? wrapped + 360.0 : wrapped;
    }

    public void setAlliance(FieldTargetConstants.Alliance alliance) {
        this.alliance = Objects.requireNonNull(alliance);
    }

    public FieldTargetConstants.Alliance getAlliance() { return alliance; }
    public double[] getTarget() { return target; }
    public boolean isAutoAiming() { return autoAiming; }

    @Override
    public void end(boolean interrupted) { drive.stop(); }
}
