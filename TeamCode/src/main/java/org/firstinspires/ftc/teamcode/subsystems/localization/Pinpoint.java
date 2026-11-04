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
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
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
}
