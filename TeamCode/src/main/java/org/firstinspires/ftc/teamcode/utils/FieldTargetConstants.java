package org.firstinspires.ftc.teamcode.utils;

/**
 * Preliminary CELL opening-center floor projections in millimeters.
 * Frame: field center is (0,0); +Y is the upper half; heading 0 points +X.
 * Verify against the assembled field before using outside this basic test TeleOp.
 */
public final class FieldTargetConstants {
    private FieldTargetConstants() { }

    public enum Alliance { RED, BLUE }

    // CELL opening projection: tag-cluster center plus the 30-degree CELL opening offset.
    public static double upperHiveXmm = 458.6;
    public static double lowerHiveXmm = -458.6;
    public static double blueCellYmm = 321.4;
    public static double redCellYmm = -321.4;
    public static double chassisAimKp = 0.040;
    public static double maxChassisAimPower = 0.85;

    /** Upper / lower is chosen from the robot's current Pinpoint Y coordinate. */
    public static double[] target(Alliance alliance, double robotYmm) {
        if (alliance == null || !Double.isFinite(robotYmm)) return null;
        double hiveX = robotYmm >= 0.0 ? upperHiveXmm : lowerHiveXmm;
        double cellY = alliance == Alliance.BLUE ? blueCellYmm : redCellYmm;
        return new double[]{hiveX, cellY};
    }

    public static String halfName(double robotYmm) {
        return robotYmm >= 0.0 ? "UPPER" : "LOWER";
    }
}
