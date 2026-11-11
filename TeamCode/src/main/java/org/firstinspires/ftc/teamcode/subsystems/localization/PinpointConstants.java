package org.firstinspires.ftc.teamcode.subsystems.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/** Pinpoint setup values measured from the turret/robot center. */
public final class PinpointConstants {
    private PinpointConstants() { }

    public static String pinpointName = "od";
    // OD CELL test: robot center's measured field pose when START is pressed.
    public static double testStartFieldXmm = 0.0;
    public static double testStartFieldYmm = 0.0;
    public static double testStartFieldHeadingDegrees = 0.0;
    // X pod is 101.3 mm right of center; left is positive.
    public static double xPodOffsetMm = -101.3;
    // Y pod is 154.4 mm behind center; forward is positive.
    public static double yPodOffsetMm = -154.4;
    public static GoBildaPinpointDriver.GoBildaOdometryPods podType =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static GoBildaPinpointDriver.EncoderDirection xPodDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection yPodDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
}
