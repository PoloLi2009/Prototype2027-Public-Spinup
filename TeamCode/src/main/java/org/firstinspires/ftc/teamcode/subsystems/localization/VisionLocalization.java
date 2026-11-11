package org.firstinspires.ftc.teamcode.subsystems.localization;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subsystems.localization.fusion.PoseHistory;
import org.firstinspires.ftc.teamcode.subsystems.localization.fusion.PinpointEkf;
import org.firstinspires.ftc.teamcode.subsystems.localization.fusion.WpiPinpointPoseEstimator;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveFlatPoseSolver;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

/** Sensor fusion only. Does not reset Pinpoint or send poses to drive/aim controls. */
public class VisionLocalization extends SubsystemBase {
    private final PoseHistory history = new PoseHistory();
    private final WpiPinpointPoseEstimator wpi = new WpiPinpointPoseEstimator();
    private final PinpointEkf ekf = new PinpointEkf();
    private Pose2d odometry = new Pose2d(), correctedVision;
    private String status = "Waiting for OD and vision";
    private double tilt = Double.NaN, lastTilt = Double.NaN, stableSince, lastFrameTime;
    private double lastAcceptedTime = Double.NaN;
    private int lastPipeline = -1, accepted, rejected;
    private double turretAtCapture = Double.NaN;
    private double rawVisionYawDegrees = Double.NaN;

    public void clear() {
        history.clear(); wpi.reset(); ekf.reset(); correctedVision = null;
        lastTilt = Double.NaN; lastAcceptedTime = Double.NaN;
        lastPipeline = -1; lastFrameTime = 0; stableSince = 0;
        rawVisionYawDegrees = Double.NaN; turretAtCapture = Double.NaN;
        status = "Fusion reset; waiting for pose";
    }
    public void update(double now, Pose2d odom, double turretDegrees, LimelightVision.Frame frame) {
        if (!Double.isFinite(odom.getX()) || !Double.isFinite(odom.getY())
                || !Double.isFinite(odom.getRotation().getRadians()) || !Double.isFinite(turretDegrees)) {
            clear(); return;
        }
        odometry = odom;
        history.add(now, odom, turretDegrees);
        wpi.updateWithTime(now, odom);
        ekf.updateWithTime(now, odom);
        if (frame == null) return;
        rawVisionYawDegrees = frame.rawPose.getOrientation().getYaw(AngleUnit.DEGREES);
        double capture = frame.approximateCaptureNanos / 1e9;
        PoseHistory.Sample past = history.sample(capture);
        if (past == null) { reject("Capture outside OD/turret history"); return; }
        PoseHistory.Sample earlier = history.sample(capture - 0.020);
        if (earlier == null) { reject("Waiting for turret history"); return; }
        if (Math.abs(past.turretDegrees-earlier.turretDegrees)/0.020
                > VisionConstants.maxTurretSpeedDegreesPerSecond) {
            reject("Turret too fast for reliable exposure"); return;
        }
        Position p = frame.rawPose.getPosition().toUnit(DistanceUnit.METER);
        HiveFlatPoseSolver.Result result = HiveFlatPoseSolver.solve(p.x, p.y, p.z,
                frame.rawPose.getOrientation().getYaw(AngleUnit.RADIANS),
                frame.rawPose.getOrientation().getPitch(AngleUnit.RADIANS),
                frame.rawPose.getOrientation().getRoll(AngleUnit.RADIANS), past.turretDegrees);
        if (result == null) { lastTilt=Double.NaN; reject("HIVE geometry / floor / level mismatch"); return; }
        tilt=result.hiveTiltDegrees; correctedVision=result.chassisPose; turretAtCapture=past.turretDegrees;
        // Do not use a frame captured during a HIVE flip. Pipeline changes restart this check.
        if (frame.pipeline != lastPipeline || !Double.isFinite(lastTilt)
                || capture-lastFrameTime > 0.2 || Math.abs(tilt-lastTilt)>2.0) stableSince=capture;
        lastPipeline=frame.pipeline; lastTilt=tilt; lastFrameTime=capture;
        if (capture-stableSince < 0.15) { reject("Waiting for stable HIVE tilt"); return; }
        double xyStd = VisionConstants.minimumVisionXyStdMeters
                + VisionConstants.visionDistanceNoiseCoefficient
                * frame.averageDistanceMeters * frame.averageDistanceMeters / Math.sqrt(frame.tagCount);
        double headingStd = Math.toRadians(VisionConstants.visionHeadingStdDegrees);
        // Both estimators see the same corrected raw measurement. Neither consumes the other's output.
        boolean wpiAccepted=wpi.addVisionMeasurement(result.chassisPose,capture,xyStd,headingStd);
        boolean ekfAccepted=ekf.addVisionMeasurement(result.chassisPose,capture,xyStd,headingStd,past.odometry);
        if (ekfAccepted) { accepted++; lastAcceptedTime=now; }
        else rejected++;
        status="WPI " + (wpiAccepted?"accepted":"rejected") + " / EKF " + ekf.getStatus();
    }
    private void reject(String reason) { status=reason; rejected++; }
    public Pose2d getEstimatedPosition() {return ekf.getEstimatedPosition();}
    public Pose2d getWpiPosition() {return wpi.getEstimatedPosition();}
    public Pose2d getOdometry() {return odometry;}
    public Pose2d getCorrectedVision() {return correctedVision;}
    public boolean hasFieldPose() {return ekf.isSeeded();}
    public double getLastVisionAgeSeconds() {
        return Double.isFinite(lastAcceptedTime)?System.nanoTime()/1e9-lastAcceptedTime:Double.NaN;
    }
    public String getStatus() {return status;}
    public double getHiveTiltDegrees() {return tilt;}
    public double getTurretAtCaptureDegrees() {return turretAtCapture;}
    public double getRawVisionYawDegrees() {return rawVisionYawDegrees;}
    public double getLastInnovationNis() {return ekf.getLastNis();}
    public double[] getStandardDeviations() {return ekf.getStandardDeviations();}
    public int getAcceptedCount() {return accepted;}
    public int getRejectedCount() {return rejected;}
}
