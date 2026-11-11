package org.firstinspires.ftc.teamcode.subsystems.vision;

/** Limelight acquisition settings. Geometry must match the actual uploaded fmaps. */
public final class VisionConstants {
    private VisionConstants() { }
    public static String hardwareName = "limelight";
    public static int bluePipeline = 0;
    public static int redPipeline = 1;
    public static int pollRateHz = 50;
    public static long maxFrameAgeMs = 200;
    // Expected values already entered in the Limelight web UI, relative to the rotating
    // turret frame with its origin at robot-center floor level. Do NOT apply these twice.
    public static final double[] cameraWebPose = {0.117, 0.0, 0.278, 180.0, 50.0, 0.0};
    // BIOBUZZ manual V0.9 section 9.6.1: pivot axis 43.95 inches above the mat.
    public static double hivePivotHeightMeters = 43.95 * 0.0254;
    public static double hivePivotXmeters = 0.0;
    public static double maxHiveTiltDegrees = 35.0;
    public static double maxRobotLevelErrorDegrees = 10.0;
    public static double maxFloorErrorMeters = 0.15;
    public static int minimumTagCount = 2;
    public static double maxTurretSpeedDegreesPerSecond = 180.0;
    public static double minimumVisionXyStdMeters = 0.08;
    public static double visionDistanceNoiseCoefficient = 0.04;
    public static double visionHeadingStdDegrees = 20.0;
}
