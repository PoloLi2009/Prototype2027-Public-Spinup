package org.firstinspires.ftc.teamcode.subsystems.shooter;

/** Names and conservative initial values for the 1:1 two-motor three-wheel shooter. */
public final class ShooterConstants {
    private ShooterConstants() { }

    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";
    public static String leftPitchServoName = "leftPitchServo";
    public static String rightPitchServoName = "rightPitchServo";

    // Open-loop presets remain available. RPM mode uses the right motor encoder.
    public static double lowPower = 0.65;
    public static double highPower = 0.90;
    public static double targetRpm = 6000.0;
    // Readiness follows the requested target, as in Overtune; 900 TPS is Solo
    // default target, not an independent minimum that defeats downward adjustment.
    public static double readyRelativeTolerance = 0.06;
    // Right motor encoder counts per motor revolution.
    public static double rightEncoderTicksPerRevolution = 28.0;
    public static double maxRpm = 6000.0;
    // 2026-11-11: user-approved verified Shooter PID AutoTune result (RPM units).
    public static double flywheelKpPowerPerRpm = 0.00020000;
    public static double flywheelKiPowerPerRpmSecond = 0.00060000;

    // Nonzero D candidates were tested; this verified run selected D=0.
    // All production velocity-control callers read this same parameter set.
    public static double flywheelKfPowerPerRpm = 0.00029167;
    public static double flywheelKdPowerPerRpmPerSecond = 0.0;
    public static double flywheelDerivativeFilterSeconds = 0.05;

    // Temporary neutral point. Calibrate limits before using the shooter on the field.
    public static double pitchLowPosition = 0.50;
    public static double pitchHighPosition = 0.50;
    public static double pitchStep = 0.01;
}
