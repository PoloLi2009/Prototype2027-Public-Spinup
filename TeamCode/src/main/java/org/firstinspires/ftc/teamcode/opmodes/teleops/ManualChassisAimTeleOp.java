package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.utils.FieldTargetConstants;

/** Basic driver test: Pinpoint pose, chassis auto-aim, intake, shooter, gate, and pitch. */
@TeleOp(name = "Manual Chassis Aim Test", group = "Test")
public class ManualChassisAimTeleOp extends OpMode {
    private Drive drive;
    private Intake intake;
    private Transit transit;
    private Shooter shooter;
    private Pinpoint pinpoint;
    private FieldTargetConstants.Alliance alliance = FieldTargetConstants.Alliance.BLUE;
    private double targetRpm = ShooterConstants.targetRpm;
    private boolean lastX, lastA, lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;
    private String shotState = "IDLE";

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        transit = new Transit(hardwareMap);
        shooter = new Shooter(hardwareMap);
        pinpoint = new Pinpoint(hardwareMap);
        telemetry.addLine("A: set current robot pose to field center, heading 0 degrees.");
        telemetry.addLine("X: switch BLUE/RED. Release right stick X for chassis auto-aim.");
        telemetry.update();
    }

    @Override
    public void loop() {
        pinpoint.periodic();
        updateButtons();

        double[] target = FieldTargetConstants.target(alliance, pinpoint.getYmm());
        double turn = gamepad1.right_stick_x;
        boolean autoAiming = Math.abs(turn) <= DriveConstants.joystickDeadband && target != null;
        if (autoAiming) {
            turn = autoAimTurn(target[0], target[1]);
        }
        drive.moveRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x, turn);

        boolean shootHeld = gamepad1.right_trigger >= 0.5;
        if (shootHeld) {
            shooter.startFlywheelsAtTargetRpm(targetRpm);
        } else {
            shooter.stop();
        }

        if (gamepad1.left_bumper) {
            transit.setGateState(Transit.GateState.CLOSED);
            intake.setIntakeState(Intake.IntakeState.REVERSED);
            shotState = "REVERSE";
        } else if (shootHeld && shooter.isAtTargetRpm()) {
            transit.setGateState(Transit.GateState.OPEN);
            intake.setIntakeState(Intake.IntakeState.FEED);
            shotState = "FEEDING";
        } else if (gamepad1.left_trigger >= 0.5) {
            transit.setGateState(Transit.GateState.CLOSED);
            intake.setIntakeState(Intake.IntakeState.FORWARD);
            shotState = "INTAKING";
        } else {
            transit.setGateState(Transit.GateState.CLOSED);
            intake.stop();
            shotState = shootHeld ? "SPINNING UP" : "IDLE";
        }

        shooter.periodic();
        intake.periodic();
        transit.periodic();
        publishTelemetry(target, autoAiming);
    }

    private void updateButtons() {
        if (gamepad1.x && !lastX) {
            alliance = alliance == FieldTargetConstants.Alliance.BLUE
                    ? FieldTargetConstants.Alliance.RED : FieldTargetConstants.Alliance.BLUE;
        }
        if (gamepad1.a && !lastA) {
            pinpoint.resetFieldPose();
        }
        if (gamepad1.dpad_up && !lastDpadUp) {
            shooter.adjustPitch(ShooterConstants.pitchStep);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            shooter.adjustPitch(-ShooterConstants.pitchStep);
        }
        // D-pad left/right changes the commanded flywheel speed by 100 RPM.
        if (gamepad1.dpad_right && !lastDpadRight) {
            targetRpm = Math.min(ShooterConstants.targetRpm, targetRpm + 100.0);
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            targetRpm = Math.max(0.0, targetRpm - 100.0);
        }
        lastX = gamepad1.x;
        lastA = gamepad1.a;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
    }

    private double autoAimTurn(double targetXmm, double targetYmm) {
        double targetHeading = Math.toDegrees(Math.atan2(targetYmm - pinpoint.getYmm(),
                targetXmm - pinpoint.getXmm()));
        double error = normalizeDegrees(targetHeading - pinpoint.getHeadingDegrees());
        return Range.clip(error * FieldTargetConstants.chassisAimKp,
                -FieldTargetConstants.maxChassisAimPower,
                FieldTargetConstants.maxChassisAimPower);
    }

    private double normalizeDegrees(double degrees) {
        double wrapped = degrees % 360.0;
        return wrapped > 180.0 ? wrapped - 360.0 : wrapped < -180.0 ? wrapped + 360.0 : wrapped;
    }

    private void publishTelemetry(double[] target, boolean autoAiming) {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pinpoint X / Y mm", "%.1f / %.1f", pinpoint.getXmm(), pinpoint.getYmm());
        telemetry.addData("Pinpoint heading", "%.1f deg", pinpoint.getHeadingDegrees());
        telemetry.addData("Half / auto aim", "%s / %s",
                FieldTargetConstants.halfName(pinpoint.getYmm()), autoAiming ? "ON" : "MANUAL");
        telemetry.addData("Target X / Y mm", target == null ? "UNCONFIGURED"
                : String.format("%.1f / %.1f", target[0], target[1]));
        telemetry.addData("Shot state", shotState);
        telemetry.addData("Flywheel target / right RPM", "%.0f / %.0f", targetRpm,
                shooter.getRightFlywheelRpm());
        telemetry.addData("Flywheel ready", shooter.isAtTargetRpm());
        telemetry.addData("Pitch command", "%.2f", shooter.getPitchPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        intake.stop();
        transit.setGateState(Transit.GateState.CLOSED);
        shooter.stop();
        intake.periodic();
        transit.periodic();
        shooter.periodic();
    }
}
