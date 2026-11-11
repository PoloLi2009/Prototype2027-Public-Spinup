package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Auto's sole drivetrain/localizer owner. Do not also construct manual Drive or Pinpoint. */
public class PedroDrive extends SubsystemBase {
    private final Follower follower;
    private final GoBildaPinpointDriver pinpoint;

    public PedroDrive(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
    }
    @Override public void periodic() {
        follower.update();
        if (!isReady()) stop();
    }
    public boolean isReady() {
        Pose pose = follower.pose();
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                && pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }
    public void setPose(Pose pose) { follower.setPose(pose); }
    public Pose pose() { return follower.pose(); }
    /** Used by the ported Overtune AutoShootCommand localization check. */
    public com.pedropathing.math.Velocity velocity() { return follower.velocity(); }
    public void follow(Path path) { follower.follow(path); }
    public boolean isBusy() { return follower.isBusy(); }
    public void stop() { follower.stop(); follower.drivetrain.stop(); }
}
