package org.firstinspires.ftc.teamcode.subsystems.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/** Measured Pinpoint setup values. Replace the zero offsets with this robot's measurements. */
public final class PinpointConstants {
    private PinpointConstants() { }

    public static String pinpointName = "od";
    public static double xPodOffsetMm = 0.0;
    public static double yPodOffsetMm = 0.0;
    public static GoBildaPinpointDriver.GoBildaOdometryPods podType =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static GoBildaPinpointDriver.EncoderDirection xPodDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection yPodDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
}
