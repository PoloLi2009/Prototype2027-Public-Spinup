package org.firstinspires.ftc.teamcode.subsystems.turret;

/** Constants for the 5:2 Yellow Jacket motor and its timing-belt turret. */
public final class TurretConstants {
    private TurretConstants() { }

    public static String turretMotorName = "turret";
    public static double manualPower = 0.35;

    /*
     * These ticks come from the motor's integrated encoder, not a separate turret
     * encoder. Measure zero and both mechanical stops before enabling. Software
     * limits do not make an overextending mechanism legal; physical stops are
     * still needed.
     */
    public static boolean useSoftLimits = true;
    // 145.1 already includes the 5.2:1 gearbox; do not multiply by 5.2 again.
    public static double motorOutputTicksPerRevolution = 145.1;
    public static double motorPulleyTeeth = 20;
    public static double turretPulleyTeeth = 142;
    /** Total travel centered on the forward zero: 360 -> +/-180; 270 -> +/-135. */
    public static double travelDegrees = 360;
    public static double minDegrees() { return -travelDegrees / 2.0; }
    public static double maxDegrees() { return travelDegrees / 2.0; }
    // Verify on blocks: positive power/encoder motion must turn counterclockwise.
    public static int encoderSign = 1;
    public static int powerSign = 1;
    public static double kP = 0.01; // power per degree; initial value, tune on robot
    public static double kI = 0;
    public static double kD = 0;
    public static double integralLimitDegreeSeconds = 50;
    public static double maxAimPower = 0.25;
    public static double angleToleranceDegrees = 1.0;
}
