package org.firstinspires.ftc.teamcode.subsystems.shooter;

/** Names and conservative initial values for the 1:1 two-motor three-wheel shooter. */
public final class ShooterConstants {
    private ShooterConstants() { }

    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";
    public static String leftPitchServoName = "leftPitchServo";
    public static String rightPitchServoName = "rightPitchServo";

    // Open-loop presets remain available; RPM mode uses the left encoder for feedback.
    public static double lowPower = 0.65;
    public static double highPower = 0.90;
    public static double targetRpm = 6000.0;
    public static double readyToleranceRpm = 100.0;
    // Left motor encoder counts per motor revolution; confirm this for the installed motor.
    public static double leftEncoderTicksPerRevolution = 28.0;
    public static double maxRpm = 6000.0;
    public static double flywheelKpPowerPerRpm = 0.0002;
    public static double flywheelKiPowerPerRpmSecond = 0.00030;

    // Temporary neutral point. Calibrate limits before using the shooter on the field.
    public static double pitchLowPosition = 0.50;
    public static double pitchHighPosition = 0.50;
    public static double pitchStep = 0.01;
}
