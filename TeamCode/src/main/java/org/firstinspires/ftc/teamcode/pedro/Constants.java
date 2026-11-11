package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.localization.PinpointConstants;

import java.util.OptionalDouble;

public class Constants {
    // Motor names, directions and pod geometry are shared with the competition Drive.
    // Motor configuration comes from DriveConstants; pod geometry comes from PinpointConstants (mm).
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(DriveConstants.leftFrontMotorName);
        c.backLeftName.set(DriveConstants.leftBackMotorName);
        c.frontRightName.set(DriveConstants.rightFrontMotorName);
        c.backRightName.set(DriveConstants.rightBackMotorName);
        c.frontLeftDirection.set(DriveConstants.leftFrontMotorDirection);
        c.backLeftDirection.set(DriveConstants.leftBackMotorDirection);
        c.frontRightDirection.set(DriveConstants.rightFrontMotorDirection);
        c.backRightDirection.set(DriveConstants.rightBackMotorDirection);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(PinpointConstants.pinpointName);
        c.xPodOffset.set(PinpointConstants.xPodOffsetMm);
        c.yPodOffset.set(PinpointConstants.yPodOffsetMm);
        c.offsetUnits.set(DistanceUnit.MM);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.podType.set(PinpointConstants.podType);
        c.xPodDirection.set(PinpointConstants.xPodDirection);
        c.yPodDirection.set(PinpointConstants.yPodDirection);
    });

    // Spinup must supply its own Foresight Tuner export. Never use Overtune's measurements.
    public static ForesightConfig foresightConfig = null;

    public static Foresight createAlgorithm() {
        if (foresightConfig == null) {
            throw new IllegalStateException(
                    "Run AutoTune > Foresight Tuner, then paste its Java export into pedro/Constants.java");
        }
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap hardwareMap) {
        Foresight algorithm = createAlgorithm();
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                algorithm);
    }
}
