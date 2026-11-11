package org.firstinspires.ftc.teamcode.subsystems.intake;

/** Constants for the shared intake/transit motor. */
public final class IntakeConstants {
    private IntakeConstants() { }

    public static String transitMotorName = "transitMotor";
    public static double intakePower = 0.65;
    public static double reversePower = -0.65;
    // Combined CELL shooting tests select these stronger manual modes.
    public static double fastIntakePower = 0.85;
    public static double fastReversePower = -0.85;
    public static double feedPower = 1.0;
}
