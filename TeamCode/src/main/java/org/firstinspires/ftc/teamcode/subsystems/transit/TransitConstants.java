package org.firstinspires.ftc.teamcode.subsystems.transit;

/** Constants for the turret-entry blocking rod. */
public final class TransitConstants {
    private TransitConstants() { }

    public static String gateServoName = "ballGate";

    // Tune on the real mechanism before allowing the flywheels to run.
    public static double gateClosedPosition = 0.15;
    public static double gateOpenPosition = 0.72;

    /** Wait for the blocking rod to clear before starting the shared intake/transit motor. */
    public static long gateRetractDelayMs = 250;
}
