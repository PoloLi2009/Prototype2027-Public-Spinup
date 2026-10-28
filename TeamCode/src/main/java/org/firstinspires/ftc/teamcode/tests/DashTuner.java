package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Migrated from the 2026 DECODE codebase. Select this as a standalone TeleOp,
 * then configure hardware names and targets from FTC Dashboard before pressing
 * START. It deliberately does not use the competition subsystems.
 */
@TeleOp(name = "DashTuner", group = "Test")
@Config
public class DashTuner extends OpMode {
    public static String[] motorName = {"", "", "", ""};
    public static String[] servoName = {"", "", "", ""};
    public static double[] motorTarget = new double[4];
    public static double[] servoTarget = new double[4];
    public static int[] slaveTo = {-1, -1, -1, -1};
    public static boolean[] isPositionCloseLoop = new boolean[4];
    public static boolean[] isVelocityCloseLoop = new boolean[4];

    // PIDF parameters are edited live through FTC Dashboard.
    public static double[] kP = {0, 0, 0, 0};
    public static double[] kI = {0, 0, 0, 0};
    public static double[] kD = {0, 0, 0, 0};
    public static double[] kF = {0, 0, 0, 0};
    public static String[] colorSensorName = {"", "", "", ""};

    private final DcMotorEx[] motors = new DcMotorEx[4];
    private final Servo[] servos = new Servo[4];
    private final ColorSensor[] colorSensors = new ColorSensor[4];
    private final DistanceSensor[] distanceSensors = new DistanceSensor[4];
    private final PIDFController[] pidfControllers = {
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0)
    };

    private final float[] hsvValues = {0F, 0F, 0F};
    private FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

        for (int i = 0; i < 4; i++) {
            if (!motorName[i].isEmpty()) {
                motors[i] = hardwareMap.get(DcMotorEx.class, motorName[i]);
                if (isVelocityCloseLoop[i]) {
                    motors[i].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                            new PIDFCoefficients(kP[i], kI[i], kD[i], kF[i]));
                }
                if (isPositionCloseLoop[i]) {
                    pidfControllers[i].setPIDF(kP[i], kI[i], kD[i], kF[i]);
                }
            }
            if (!servoName[i].isEmpty()) {
                servos[i] = hardwareMap.get(Servo.class, servoName[i]);
            }
            if (!colorSensorName[i].isEmpty()) {
                colorSensors[i] = hardwareMap.get(ColorSensor.class, colorSensorName[i]);
                distanceSensors[i] = hardwareMap.get(DistanceSensor.class, colorSensorName[i]);
            }
        }
    }

    @Override
    public void loop() {
        for (int i = 0; i < 4; i++) {
            tuneMotor(i);
            tuneServo(i);
            reportSensor(i);
        }
    }

    private void tuneMotor(int index) {
        if (motors[index] == null) {
            return;
        }

        int leader = slaveTo[index];
        if (leader >= 0 && leader < motors.length) {
            if (motors[leader] != null && isVelocityCloseLoop[leader]
                    && !isPositionCloseLoop[leader]) {
                motors[index].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                        new PIDFCoefficients(kP[leader], kI[leader], kD[leader], kF[leader]));
                motors[index].setVelocity(-motorTarget[leader]);
            }
            return;
        }

        TelemetryPacket packet = new TelemetryPacket();
        if (isVelocityCloseLoop[index] && !isPositionCloseLoop[index]) {
            motors[index].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(kP[index], kI[index], kD[index], kF[index]));
            motors[index].setVelocity(motorTarget[index]);
            packet.put("targetVelocity " + index, motorTarget[index]);
            packet.put("currentVelocity " + index, motors[index].getVelocity());
        } else if (isPositionCloseLoop[index] && !isVelocityCloseLoop[index]) {
            pidfControllers[index].setPIDF(kP[index], kI[index], kD[index], kF[index]);
            double position = motors[index].getCurrentPosition();
            motors[index].setPower(pidfControllers[index].calculate(position, motorTarget[index]));
            packet.put("targetPosition " + index, motorTarget[index]);
            packet.put("currentPosition " + index, position);
        } else {
            motors[index].setPower(motorTarget[index]);
            packet.put("currentVelocity " + index, motors[index].getVelocity());
            packet.put("currentPosition " + index, motors[index].getCurrentPosition());
        }
        packet.put("currentPower " + index, motors[index].getPower());
        dashboard.sendTelemetryPacket(packet);
    }

    private void tuneServo(int index) {
        if (servos[index] != null) {
            servos[index].setPosition(servoTarget[index]);
        }
    }

    private void reportSensor(int index) {
        if (colorSensors[index] == null) {
            return;
        }

        Color.RGBToHSV((int) (colorSensors[index].red() * 255),
                (int) (colorSensors[index].green() * 255),
                (int) (colorSensors[index].blue() * 255), hsvValues);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Alpha " + index, colorSensors[index].alpha());
        packet.put("Red " + index, colorSensors[index].red());
        packet.put("Green " + index, colorSensors[index].green());
        packet.put("Blue " + index, colorSensors[index].blue());
        packet.put("Hue " + index, hsvValues[0]);
        packet.put("Distance (cm) " + index, String.format(Locale.US, "%.02f",
                distanceSensors[index].getDistance(DistanceUnit.CM)));
        dashboard.sendTelemetryPacket(packet);
    }
}
