package org.firstinspires.ftc.teamcode.subsystems.transit;

/** Constants for the turret-entry ball gate and the ramp/blocker. */
public final class TransitConstants {
    private TransitConstants() { }

    public static String gateServoName = "ballGate";
    public static String rampServoName = "ballRamp";

    // Tune on the real mechanism before allowing the flywheels to run.
    public static double gateClosedPosition = 0.15;
    public static double gateOpenPosition = 0.72;
    public static double rampUpPosition = 0.18;
    public static double rampDownPosition = 0.78;
}
