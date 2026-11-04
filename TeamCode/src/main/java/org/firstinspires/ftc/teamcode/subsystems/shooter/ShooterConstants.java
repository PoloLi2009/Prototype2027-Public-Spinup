package org.firstinspires.ftc.teamcode.subsystems.shooter;

/** Names and conservative initial values for the 1:1 two-motor three-wheel shooter. */
public final class ShooterConstants {
    private ShooterConstants() { }

    public static String leftShooterName = "Left Flywheel Motor";
    public static String rightShooterName = "Right Flywheel Motor";
    public static String leftPitchServoName = "Left Pitch Servo";
    public static String rightPitchServoName = "Right Pitch Servo";

    // Start with open-loop power; move to velocity PIDF after real ball testing.
    public static double lowPower = 0.65;
    public static double highPower = 0.90;
    public static double targetRpm = 6000.0;
    public static double readyToleranceRpm = 100.0;
    public static double rightEncoderTicksPerRevolution = 28.0;
    public static double leftFlywheelFollowPower = 1.0;

    // Temporary neutral point. Calibrate limits before using the shooter on the field.
    public static double pitchLowPosition = 0.50;
    public static double pitchHighPosition = 0.50;
    public static double pitchStep = 0.01;
}
