package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Tunable drivetrain values and Driver Station hardware names. */
public final class DriveConstants {
    private DriveConstants() { }

    public static String leftFrontMotorName = "leftFrontMotor";
    public static String leftBackMotorName = "leftBackMotor";
    public static String rightFrontMotorName = "rightFrontMotor";
    public static String rightBackMotorName = "rightBackMotor";

    // 2026-11-11 field report: all six Solo motion directions were inverted.
    // Reverse all four hardware directions, keeping the Blue mixer and OD signs unchanged.
    public static DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction leftBackMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightBackMotorDirection = DcMotorSimple.Direction.REVERSE;

    public static double strafingBalance = 1.00;
    public static double teleOpScale = 1.00;
    public static double joystickDeadband = 0.04;
}
