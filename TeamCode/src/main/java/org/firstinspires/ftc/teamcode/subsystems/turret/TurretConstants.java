package org.firstinspires.ftc.teamcode.subsystems.turret;

/** Turret drive and software-limit constants. */
public final class TurretConstants {
    private TurretConstants() { }

    public static String turretMotorName = "turretMotor";
    public static double manualPower = 0.50;

    /* Measure zero and both mechanical stops before enabling software limits. */
    public static boolean useSoftLimits = true;
    /** Total travel centered on the forward zero: 360 -> +/-180; 270 -> +/-135. */
    public static double travelDegrees = 360;
    public static double minDegrees() { return -travelDegrees / 2.0; }
    public static double maxDegrees() { return travelDegrees / 2.0; }
    public static int powerSign = -1;
    public static double kP = 0.01; // power per degree; initial value, tune on robot
    public static double kI = 0;
    public static double kD = 0;
    public static double integralLimitDegreeSeconds = 50;
    public static double maxAimPower = 0.25;
    public static double angleToleranceDegrees = 1.0;
}
