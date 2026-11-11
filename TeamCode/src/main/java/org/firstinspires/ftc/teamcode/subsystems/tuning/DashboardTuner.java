package org.firstinspires.ftc.teamcode.subsystems.tuning;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

/** Generic four-channel Dashboard tuning hardware, isolated from competition subsystems. */
@Config
public class DashboardTuner extends SubsystemBase {
    private final HardwareMap hardwareMap;
    public static String[] motorName = {"", "", "", ""};
    public static String[] servoName = {"", "", "", ""};
    public static double[] motorTarget = new double[4];
    public static double[] servoTarget = {-1, -1, -1, -1};
    public static int[] slaveTo = {-1, -1, -1, -1};
    public static double[] slavePowerScale = {1, 1, 1, 1};
    public static boolean[] isPositionCloseLoop = new boolean[4];
    public static boolean[] isVelocityCloseLoop = new boolean[4];

    // PIDF parameters are edited live through FTC Dashboard.
    public static double[] kP = {0, 0, 0, 0};
    public static double[] kI = {0, 0, 0, 0};
    public static double[] kD = {0, 0, 0, 0};
    public static double[] kF = {0, 0, 0, 0};
    public static String[] colorSensorName = {"", "", "", ""};

    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final PIDFController[] pidfControllers = {
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0)
    };

    private final String[] boundMotorNames = new String[4];
    private final int[] previousMode = new int[4];
    private final double[] output = new double[4];
    private final float[] hsvValues = new float[3];
    private final FtcDashboard dashboard;
    private Turret lockedTurret;

    public DashboardTuner(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        dashboard = FtcDashboard.getInstance();
    }

    public void applyTargets() {
        TelemetryPacket packet = new TelemetryPacket();
        if (!Arrays.equals(boundMotorNames, motorName)) bindMotors();
        if (lockedTurret != null) lockedTurret.periodic();

        // Compute leaders first. A follower never runs its own encoder loop.
        Arrays.fill(output, 0.0);
        for (int i = 0; i < motors.length; i++) {
            if (isTunable(i) && slaveTo[i] == -1) tuneMotor(i, packet);
        }
        for (int i = 0; i < motors.length; i++) {
            if (isTunable(i) && slaveTo[i] != -1) followPower(i, packet);
            tuneServo(i, packet);
            reportSensor(i, packet);
        }
        if (dashboard != null) dashboard.sendTelemetryPacket(packet);
    }

    private void bindMotors() {
        stopMotors();
        Arrays.fill(motors, null);
        Arrays.fill(previousMode, -1);
        System.arraycopy(motorName, 0, boundMotorNames, 0, motors.length);
        for (int i = 0; i < motors.length; i++) {
            String name = motorName[i];
            if (name.isEmpty()) continue;
            if (TurretConstants.LOCK_STARTUP_ANGLE && name.equals(TurretConstants.turretMotorName)) {
                if (lockedTurret == null && hardwareMap.tryGet(DcMotorEx.class, name) != null) {
                    lockedTurret = new Turret(hardwareMap);
                }
                continue;
            }
            DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, name);
            if (motor == null || Arrays.asList(motors).contains(motor)) continue;
            motors[i] = motor;
            motor.setPower(0.0);
            boolean reversed = name.equals(DriveConstants.leftFrontMotorName)
                    || name.equals(DriveConstants.leftBackMotorName)
                    || name.equals(ShooterConstants.leftShooterName)
                    || name.equals(ShooterConstants.rightShooterName);
            motor.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }
    }

    private boolean isTunable(int index) {
        return index >= 0 && index < motors.length && motors[index] != null;
    }

    private boolean hasFollower(int index) {
        for (int i = 0; i < motors.length; i++) {
            if (i != index && isTunable(i) && slaveTo[i] == index) return true;
        }
        return false;
    }

    private void tuneMotor(int i, TelemetryPacket packet) {
        DcMotorEx motor = motors[i];
        boolean position = isPositionCloseLoop[i];
        boolean velocity = isVelocityCloseLoop[i];
        double target = motorTarget[i];
        if (!Double.isFinite(target) || (position && velocity)
                || ((position || velocity) && (!validGains(i)
                || boundMotorNames[i].equals(ShooterConstants.leftShooterName)))) {
            applyPower(i, 0, 0.0);
            packet.put("motorStatus " + i, "Check target, mode, PIDF or encoder");
            return;
        }

        double currentVelocity = motor.getVelocity();
        int currentPosition = motor.getCurrentPosition();
        if (velocity && !hasFollower(i) && target != 0.0) {
            selectMode(i, 3, DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(kP[i], kI[i], kD[i], kF[i]));
            motor.setVelocity(target);
        } else if (position || (velocity && target != 0.0)) {
            int mode = position ? 1 : 2;
            selectMode(i, mode, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            PIDFController controller = pidfControllers[i];
            controller.setPIDF(kP[i], kI[i], kD[i], kF[i]);
            double measured = position ? currentPosition : currentVelocity;
            output[i] = Double.isFinite(measured) ? clipPower(controller.calculate(measured, target)) : 0.0;
            motor.setPower(output[i]);
        } else {
            applyPower(i, 0, target);
        }

        if (velocity) packet.put("targetVelocity " + i, target);
        if (position) packet.put("targetPosition " + i, target);
        packet.put("currentVelocity " + i, currentVelocity);
        packet.put("currentPosition " + i, currentPosition);
        packet.put("currentPower " + i, motor.getPower());
    }

    private boolean validGains(int i) {
        return Double.isFinite(kP[i]) && Double.isFinite(kI[i])
                && Double.isFinite(kD[i]) && Double.isFinite(kF[i]);
    }

    private void followPower(int i, TelemetryPacket packet) {
        int leader = slaveTo[i];
        boolean valid = isTunable(leader) && leader != i && slaveTo[leader] == -1;
        applyPower(i, 4, valid ? output[leader] * slavePowerScale[i] : 0.0);
        packet.put("currentPower " + i, output[i]);
        if (!valid) packet.put("motorStatus " + i, "Invalid leader");
    }

    private void applyPower(int i, int mode, double power) {
        selectMode(i, mode, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        output[i] = clipPower(power);
        motors[i].setPower(output[i]);
    }

    private void selectMode(int i, int mode, DcMotor.RunMode runMode) {
        DcMotorEx motor = motors[i];
        if (previousMode[i] != mode) {
            motor.setPower(0.0);
            pidfControllers[i].reset();
            previousMode[i] = mode;
        }
        if (motor.getMode() != runMode) motor.setMode(runMode);
    }

    private static double clipPower(double power) {
        return Double.isFinite(power) ? Math.max(-1.0, Math.min(1.0, power)) : 0.0;
    }

    private void tuneServo(int i, TelemetryPacket packet) {
        if (motorName[i].equals(TurretConstants.turretMotorName) && TurretConstants.LOCK_STARTUP_ANGLE) {
            packet.put("motorStatus " + i, lockedTurret == null ? "Missing turret" : "Turret locked");
            if (lockedTurret != null) packet.put("turretAngle " + i, lockedTurret.getAngleDegrees());
        } else if (!motorName[i].isEmpty() && motors[i] == null) {
            packet.put("motorStatus " + i, "Missing or duplicate motor");
        }
        String name = servoName[i];
        if (name.isEmpty()) return;
        Servo servo = hardwareMap.tryGet(Servo.class, name);
        if (servo == null) {
            packet.put("servoStatus " + i, "Missing servo");
            return;
        }
        double target = servoTarget[i];
        if (Double.isFinite(target) && target >= 0.0 && target <= 1.0) {
            servo.setDirection(name.equals(ShooterConstants.rightPitchServoName)
                    ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            servo.setPosition(target);
            packet.put("servoPosition " + i, target);
        }
    }

    private void reportSensor(int i, TelemetryPacket packet) {
        String name = colorSensorName[i];
        if (name.isEmpty()) return;
        ColorSensor sensor = hardwareMap.tryGet(ColorSensor.class, name);
        if (sensor != null) {
            Color.RGBToHSV(sensor.red(), sensor.green(), sensor.blue(), hsvValues);
            packet.put("Alpha " + i, sensor.alpha());
            packet.put("Red " + i, sensor.red());
            packet.put("Green " + i, sensor.green());
            packet.put("Blue " + i, sensor.blue());
            packet.put("Hue " + i, hsvValues[0]);
        }
        DistanceSensor distance = hardwareMap.tryGet(DistanceSensor.class, name);
        if (distance != null) packet.put("Distance (cm) " + i, distance.getDistance(DistanceUnit.CM));
    }

    public void stop() {
        if (lockedTurret != null) {
            lockedTurret.shutdown();
            lockedTurret = null;
        }
        stopMotors();
    }

    private void stopMotors() {
        for (DcMotorEx motor : motors) if (motor != null) motor.setPower(0.0);
    }
}
