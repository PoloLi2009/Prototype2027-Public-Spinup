package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Spinup's flat-HIVE correction of one Limelight MT1 frame (same call path as VisionLocalization).
 * Used by the Overtune Vision port in place of Overtune VisionMath.solveHiveFlatPose, because
 * Spinup's camera rides on the turret and its solver needs the turret angle.
 */
public final class HiveFrameSolver {
    public static final class Observation {
        public final double xMm, yMm, headingDegrees, hiveTiltDegrees, timestamp;
        public final long captureNanos;
        public Observation(double xMm, double yMm, double headingDegrees, double hiveTiltDegrees,
                           double timestamp, long captureNanos) {
            this.xMm = xMm; this.yMm = yMm; this.headingDegrees = headingDegrees;
            this.hiveTiltDegrees = hiveTiltDegrees; this.timestamp = timestamp;
            this.captureNanos = captureNanos;
        }
    }

    private HiveFrameSolver() { }

    /** Returns null when the frame fails Spinup's HIVE geometry / floor / level checks. */
    public static Observation solve(LimelightVision.Frame frame, double turretClockwiseDegrees) {
        if (frame == null || frame.rawPose == null) return null;
        Position p = frame.rawPose.getPosition().toUnit(DistanceUnit.METER);
        HiveFlatPoseSolver.Result result = HiveFlatPoseSolver.solve(p.x, p.y, p.z,
                frame.rawPose.getOrientation().getYaw(AngleUnit.RADIANS),
                frame.rawPose.getOrientation().getPitch(AngleUnit.RADIANS),
                frame.rawPose.getOrientation().getRoll(AngleUnit.RADIANS), turretClockwiseDegrees);
        if (result == null) return null;
        return new Observation(result.chassisPose.getX() * 1000.0, result.chassisPose.getY() * 1000.0,
                result.chassisPose.getRotation().getDegrees(), result.hiveTiltDegrees,
                frame.cameraTimestamp, frame.approximateCaptureNanos);
    }
}
