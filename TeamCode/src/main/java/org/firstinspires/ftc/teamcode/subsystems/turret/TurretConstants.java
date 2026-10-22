package org.firstinspires.ftc.teamcode.subsystems.turret;

/** Constants for the 5:2 Yellow Jacket motor and its timing-belt turret. */
public final class TurretConstants {
    private TurretConstants() { }

    public static String turretMotorName = "turret";
    public static double manualPower = 0.35;

    /*
     * Measure zero and both mechanical stops before enabling. Software limits do
     * not make an overextending mechanism legal; physical stops are still needed.
     */
    public static boolean useSoftLimits = false;
    public static int minTicks = -1800;
    public static int maxTicks = 1800;
}
