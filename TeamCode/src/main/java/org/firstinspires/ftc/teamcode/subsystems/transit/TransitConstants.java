package org.firstinspires.ftc.teamcode.subsystems.transit;

/** Constants for the turret-entry blocking rod. */
public final class TransitConstants {
    private TransitConstants() { }

    public static String limitServoName = "limitServo";

    // Measured limit-servo positions: open at 0.15, closed at 0.32.
    public static double gateClosedPosition = 0.32;
    public static double gateOpenPosition = 0.15;

    /** Wait for the blocking rod to clear before starting the shared intake/transit motor. */
    public static long gateRetractDelayMs = 250;
}
