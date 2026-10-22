package org.firstinspires.ftc.teamcode.subsystems.shooter;

/** Names and conservative initial values for the 1:1 two-motor three-wheel shooter. */
public final class ShooterConstants {
    private ShooterConstants() { }

    public static String leftShooterName = "flywheelLeft";
    public static String rightShooterName = "flywheelRight";
    public static String pitchServoName = "shooterPitch";

    // Start with open-loop power; move to velocity PIDF after real ball testing.
    public static double lowPower = 0.65;
    public static double highPower = 0.90;

    public static double pitchLowPosition = 0.22;
    public static double pitchHighPosition = 0.62;
}
