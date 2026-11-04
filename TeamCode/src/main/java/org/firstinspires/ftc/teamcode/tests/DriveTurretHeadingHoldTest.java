package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.utils.TurretMath;

/** Drive with the turret holding the field direction it faced at START. */
@TeleOp(name = "Drive Turret Heading Hold Test", group = "Test")
public class DriveTurretHeadingHoldTest extends OpMode {
    // Pinpoint heading is positive counterclockwise; measured turret angle is positive clockwise.
    private static final double HEADING_TO_TURRET_SIGN = 1.0;
    private static final double HOLD_MIN_POWER = 0.0;
    private static final double HOLD_MAX_POWER = 0.40;
    private static final double HOLD_KP = 0.012;

    private Drive drive;
    private Turret turret;
    private Pinpoint pinpoint;
    private double headingZeroDegrees;
    private double holdWorldDegrees;
    private boolean lastLeftStickButton;
    private boolean lastRightStickButton;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        turret = new Turret(hardwareMap);
        pinpoint = new Pinpoint(hardwareMap);
        telemetry.addLine("Point turret forward before INIT; encoder zero is set at INIT.");
        telemetry.addLine("Left stick: drive/strafe; right stick X: turn chassis.");
        telemetry.addLine("Left stick press: zero heading; right stick press: hold current aim.");
        telemetry.update();
    }

    @Override
    public void start() {
        pinpoint.periodic();
        headingZeroDegrees = pinpoint.getHeadingDegrees();
        holdWorldDegrees = turret.getAngleDegrees();
        lastLeftStickButton = gamepad1.left_stick_button;
        lastRightStickButton = gamepad1.right_stick_button;
    }

    @Override
    public void loop() {
        pinpoint.periodic();
        double rawHeadingDegrees = pinpoint.getHeadingDegrees();
        double headingDegrees = rawHeadingDegrees - headingZeroDegrees;
        if (gamepad1.left_stick_button && !lastLeftStickButton) {
            // Re-zero the working heading without changing the turret's field target.
            holdWorldDegrees += HEADING_TO_TURRET_SIGN * headingDegrees;
            headingZeroDegrees = rawHeadingDegrees;
            headingDegrees = 0.0;
        }
        if (gamepad1.right_stick_button && !lastRightStickButton) {
            // The current physical turret direction becomes the new field target.
            holdWorldDegrees = turret.getAngleDegrees()
                    - HEADING_TO_TURRET_SIGN * headingDegrees;
        }
        lastLeftStickButton = gamepad1.left_stick_button;
        lastRightStickButton = gamepad1.right_stick_button;

        double desiredDegrees = holdWorldDegrees
                + HEADING_TO_TURRET_SIGN * headingDegrees;
        double targetDegrees = TurretMath.reachableDegrees(desiredDegrees,
                turret.getAngleDegrees(), TurretConstants.minDegrees(),
                TurretConstants.maxDegrees());
        boolean tracking = turret.aimAtPhysicalDegrees(targetDegrees,
                HOLD_MIN_POWER, HOLD_MAX_POWER, HOLD_KP);

        drive.moveRobot(deadband(gamepad1.left_stick_y),
                deadband(gamepad1.left_stick_x), -deadband(gamepad1.right_stick_x));

        telemetry.addData("Chassis heading deg", "%.2f", headingDegrees);
        telemetry.addData("Turret angle deg", "%.2f", turret.getAngleDegrees());
        telemetry.addData("Turret target deg", "%.2f", targetDegrees);
        telemetry.addData("Turret encoder ticks", turret.getEncoderTicks());
        telemetry.addData("Heading hold", tracking ? "ON" : "TARGET OUT OF RANGE");
        telemetry.update();
    }

    private double deadband(double value) {
        return Math.abs(value) > DriveConstants.joystickDeadband ? value : 0.0;
    }

    @Override
    public void stop() {
        if (drive != null) drive.stop();
        if (turret != null) turret.stop();
    }
}
