package org.firstinspires.ftc.teamcode.subsystems.localization;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Minimal wrapper for the FTC SDK goBILDA Pinpoint driver. */
public class Pinpoint extends SubsystemBase {
    private final GoBildaPinpointDriver pinpoint;
    private boolean fieldPoseInitialized;

    public Pinpoint(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PinpointConstants.pinpointName);
        pinpoint.setOffsets(PinpointConstants.xPodOffsetMm, PinpointConstants.yPodOffsetMm,
                DistanceUnit.MM);
        pinpoint.setEncoderResolution(PinpointConstants.podType);
        pinpoint.setEncoderDirections(PinpointConstants.xPodDirection,
                PinpointConstants.yPodDirection);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void periodic() {
        pinpoint.update();
    }

    /** Sets the current physical robot pose as the field origin and forward heading. */
    public void resetFieldPose() {
        setFieldPose(0, 0, 0);
    }

    public void setFieldPose(double xMm, double yMm, double headingDegrees) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, xMm, yMm,
                AngleUnit.DEGREES, headingDegrees));
        fieldPoseInitialized = true;
    }

    public boolean isFieldPoseInitialized() {
        return fieldPoseInitialized;
    }

    public boolean isReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public double getXmm() {
        return pinpoint.getPosition().getX(DistanceUnit.MM);
    }

    public double getYmm() {
        return pinpoint.getPosition().getY(DistanceUnit.MM);
    }

    public double getHeadingDegrees() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    /** Translation speed from the last update(); used by the ported ShootAssist position hold. */
    public double getTranslationSpeedMmPerSecond() {
        return Math.hypot(pinpoint.getVelX(DistanceUnit.MM), pinpoint.getVelY(DistanceUnit.MM));
    }
}
