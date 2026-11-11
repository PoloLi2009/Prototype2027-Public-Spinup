package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

/** Corrects one alliance HIVE's rigid rotation about its horizontal field-Y pivot.
 * Input botpose must already use the measured camera-to-turret-center transform in Limelight.
 * The turret-center frame has its origin on the floor and turns with the turret.
 */
public final class HiveFlatPoseSolver {
    public static final class Result {
        public final Pose2d chassisPose;
        public final double hiveTiltDegrees, floorErrorMeters;
        Result(Pose2d pose, double tilt, double floorError) {
            chassisPose = pose; hiveTiltDegrees = tilt; floorErrorMeters = floorError;
        }
    }
    public static Result solve(double x, double y, double z, double yaw, double pitch,
                               double roll, double turretClockwiseDegrees) {
        double[] input = {x,y,z,yaw,pitch,roll,turretClockwiseDegrees};
        for (double v : input) if (!Double.isFinite(v)) return null;
        // R = Rz(yaw) Ry(pitch) Rx(roll); use the full up vector, including at yaw +/-90 deg.
        double r00 = Math.cos(yaw)*Math.cos(pitch), r10 = Math.sin(yaw)*Math.cos(pitch);
        double r20 = -Math.sin(pitch);
        double ux = Math.cos(yaw)*Math.sin(pitch)*Math.cos(roll)+Math.sin(yaw)*Math.sin(roll);
        double uy = Math.sin(yaw)*Math.sin(pitch)*Math.cos(roll)-Math.cos(yaw)*Math.sin(roll);
        double uz = Math.cos(pitch)*Math.cos(roll);
        double phi = Math.atan2(ux, uz);
        if (Math.abs(Math.toDegrees(phi)) > VisionConstants.maxHiveTiltDegrees
                || Math.abs(uy) > Math.sin(Math.toRadians(VisionConstants.maxRobotLevelErrorDegrees)))
            return null;
        double c = Math.cos(phi), s = Math.sin(phi);
        // Apply Ry(-phi) about (pivotX, any Y, pivotHeight), not about the field origin.
        double dx = x - VisionConstants.hivePivotXmeters;
        double dz = z - VisionConstants.hivePivotHeightMeters;
        double correctedX = c*dx-s*dz+VisionConstants.hivePivotXmeters;
        double correctedZ = s*dx+c*dz+VisionConstants.hivePivotHeightMeters;
        if (Math.abs(correctedZ) > VisionConstants.maxFloorErrorMeters) return null;
        double turretYaw = Math.atan2(r10, c*r00-s*r20);
        // Encoder CW is positive, while the field yaw is CCW-positive.
        double chassisYaw = turretYaw + Math.toRadians(turretClockwiseDegrees);
        return new Result(new Pose2d(correctedX, y, new Rotation2d(chassisYaw)),
                -Math.toDegrees(phi), correctedZ);
    }
    private HiveFlatPoseSolver() { }
}
