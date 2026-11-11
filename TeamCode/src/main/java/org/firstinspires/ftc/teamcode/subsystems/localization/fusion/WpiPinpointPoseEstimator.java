// Copyright (c) FIRST and other WPILib contributors.
// Adapted from WPILib v2025.3.2 PoseEstimator; see TeamCode/third-party/WPILib-LICENSE.md.
package org.firstinspires.ftc.teamcode.subsystems.localization.fusion;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Twist2d;

/** Android/Java-8 adaptation of WPILib's SE(2) correction and closed-form Kalman gain.
 * Pinpoint supplies odometry directly. Only chronological vision measurements are accepted.
 * This is a port, not an edu.wpi.first library dependency or an additional EKF stage.
 */
public class WpiPinpointPoseEstimator {
    private final PoseHistory history = new PoseHistory();
    private Pose2d raw = new Pose2d(), estimate = new Pose2d();
    private Pose2d anchorRaw, anchorEstimate;
    private double lastVisionTime = Double.NEGATIVE_INFINITY;
    public void reset() {
        history.clear(); anchorRaw = null; anchorEstimate = null;
        lastVisionTime = Double.NEGATIVE_INFINITY; raw = estimate = new Pose2d();
    }
    public void updateWithTime(double time, Pose2d odometry) {
        raw = odometry; history.add(time, odometry, 0);
        estimate = compensate(odometry);
    }
    private Pose2d compensate(Pose2d odometry) {
        return anchorRaw == null ? odometry : anchorEstimate.plus(odometry.minus(anchorRaw));
    }
    public boolean addVisionMeasurement(Pose2d vision, double time, double xyStd, double headingStd) {
        PoseHistory.Sample sample = history.sample(time);
        if (sample == null || time <= lastVisionTime) return false;
        Pose2d predicted = compensate(sample.odometry);
        if (anchorRaw == null) {
            anchorEstimate = vision;
        } else {
            Twist2d error = predicted.log(vision);
            if (Math.hypot(vision.getX()-predicted.getX(), vision.getY()-predicted.getY()) > 0.75
                    || Math.abs(error.dtheta) > Math.toRadians(45)) return false;
            // q/(q+sqrt(q*r)) == stateStd/(stateStd+measurementStd).
            double kxy = 0.10 / (0.10 + xyStd);
            double kh = Math.toRadians(2) / (Math.toRadians(2) + headingStd);
            anchorEstimate = predicted.exp(new Twist2d(error.dx * kxy, error.dy * kxy,
                    error.dtheta * kh));
        }
        anchorRaw = sample.odometry; lastVisionTime = time;
        estimate = compensate(raw);
        return true;
    }
    public Pose2d getEstimatedPosition() { return estimate; }
}
