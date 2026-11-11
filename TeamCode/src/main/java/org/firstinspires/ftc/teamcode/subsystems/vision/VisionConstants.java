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
    // Order is the Limelight UI "robot space" order: {forward m, right m, up m, roll, pitch, yaw deg}.
    //   forward 0.117 m, centered, 0.278 m above the floor, pitched 50 deg, yaw 0 (looks along
    //   the turret's forward = the shooter direction, since Spinup shoots out of the front).
    // NOTE (2026-11-11): the Limelight is mounted UPSIDE DOWN. That inversion is represented only
    //   by roll = 180 here / in the Limelight web UI; no code flips the image or the pose. If the
    //   camera's roll in the web UI is not 180, MegaTag poses will be wrong; nothing else compensates.
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
