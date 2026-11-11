package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drive.ShootAssistConstants;
import org.firstinspires.ftc.teamcode.subsystems.field.CellTarget;

/**
 * Port of Overtune AutoHiveTarget: observes one CELL without writing vision corrections into
 * Pedro's Pinpoint localizer. Spinup substitutions: the HIVE observation is Spinup's
 * {@link HiveFrameSolver} result held by {@link ShootAssistVision}; the CELL point is Spinup's
 * {@link CellTarget}; the shooter offset is {@link ShootAssistConstants#shooterYawDegrees}.
 */
public class AutoHiveTarget {
    private final ShootAssistVision vision;
    private final CellTarget cell;
    private ShootAssistVision.HiveState state = ShootAssistVision.HiveState.UNKNOWN;
    private double distanceCm = Double.NaN;
    private double yawErrorDegrees = Double.NaN;
    private String status = "Waiting for a HIVE observation";

    public AutoHiveTarget(ShootAssistVision vision, CellTarget cell) {
        if (cell == null || cell.blueAlliance != vision.isBlueAlliance()) {
            throw new IllegalArgumentException("The CELL must belong to the vision alliance");
        }
        this.vision = vision;
        this.cell = cell;
    }

    public void update(Pose pedroPose) {
        state = ShootAssistVision.HiveState.UNKNOWN;
        distanceCm = Double.NaN;
        yawErrorDegrees = Double.NaN;
        status = vision.getPoseStatus();
        if (pedroPose == null || !Double.isFinite(pedroPose.x())
                || !Double.isFinite(pedroPose.y()) || !Double.isFinite(pedroPose.heading())) {
            status = "Invalid Pedro pose";
            return;
        }
        Pose2D fieldPose = toHiveFieldPose(pedroPose);
        double[] target = fieldTarget();
        HiveFrameSolver.Observation correction = vision.getCurrentObservation().orElse(null);
        if (correction != null && target != null) {
            // Read HIVE tilt independently of Pinpoint translation drift.
            double headingDifference = Math.abs(Math.IEEEremainder(
                    correction.headingDegrees - fieldPose.getHeading(AngleUnit.DEGREES), 360.0));
            if (headingDifference < ShootAssistConstants.maximumVisionHeadingDifferenceDegrees) {
                double hiveAngleDegrees = correction.hiveTiltDegrees;
                state = Math.abs(hiveAngleDegrees) <= ShootAssistConstants.highHiveTiltThresholdDegrees
                        ? ShootAssistVision.HiveState.MIDDLE
                        : -Math.sin(Math.toRadians(hiveAngleDegrees)) * target[0] > 0.0
                        ? ShootAssistVision.HiveState.HIGH : ShootAssistVision.HiveState.LOW;
            } else {
                status = "HIVE heading disagrees with Pedro pose";
            }
        }
        // Aim and shot calibration share the CELL point; odometry supplies the yaw error.
        if (target != null) {
            yawErrorDegrees = ShootAssistVision.coordinateYawErrorDegrees(fieldPose,
                    target[0], target[1], ShootAssistConstants.shooterYawDegrees);
            distanceCm = DistanceUnit.INCH.toCm(Math.hypot(
                    target[0] - fieldPose.getX(DistanceUnit.INCH),
                    target[1] - fieldPose.getY(DistanceUnit.INCH)));
        }
    }

    private double[] fieldTarget() {
        if (!Double.isFinite(cell.xMm) || !Double.isFinite(cell.yMm)) return null;
        return new double[]{DistanceUnit.MM.toInches(cell.xMm), DistanceUnit.MM.toInches(cell.yMm)};
    }

    /** Pedro (0..144 in) to the field-center frame shared by the HIVE map and CellTarget. */
    public static Pose2D toHiveFieldPose(Pose pose) {
        double c = ShootAssistConstants.pedroFieldCenterInches;
        return new Pose2D(DistanceUnit.INCH, pose.x() - c, pose.y() - c,
                AngleUnit.RADIANS, pose.heading());
    }

    public double getYawErrorDegrees() { return yawErrorDegrees; }
    public ShootAssistVision.HiveState getState() { return state; }
    public double getDistanceCm() { return distanceCm; }
    public String getStatus() { return status; }
    public CellTarget getCell() { return cell; }
}
