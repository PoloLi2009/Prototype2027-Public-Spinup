package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/** Acquires each MT1 frame once. Raw fmap coordinates are not robot field coordinates yet. */
public class LimelightVision extends SubsystemBase {
    public static final class Frame {
        public final Pose3D rawPose;
        public final long approximateCaptureNanos;
        public final double cameraTimestamp;
        public final int pipeline;
        public final int tagCount;
        public final double averageDistanceMeters;
        Frame(Pose3D pose, long captureNanos, double timestamp, int pipeline, int tagCount,
              double averageDistanceMeters) {
            this.rawPose = pose;
            this.approximateCaptureNanos = captureNanos;
            this.cameraTimestamp = timestamp;
            this.pipeline = pipeline;
            this.tagCount = tagCount;
            this.averageDistanceMeters = averageDistanceMeters;
        }
    }

    private final Limelight3A camera;
    private boolean blueAlliance = true;
    private double lastTimestamp = Double.NaN;
    private Frame frame;
    private String status = "Waiting for frame";

    public LimelightVision(HardwareMap hardwareMap) {
        camera = hardwareMap.get(Limelight3A.class, VisionConstants.hardwareName);
        camera.setPollRateHz(VisionConstants.pollRateHz);
        camera.pipelineSwitch(getExpectedPipeline());
        camera.start();
    }

    public void toggleAlliance() {
        blueAlliance = !blueAlliance;
        frame = null;
        camera.pipelineSwitch(getExpectedPipeline());
        status = "Waiting for new pipeline frame";
    }

    @Override
    public void periodic() {
        frame = null;
        if (!camera.isConnected()) { status = "Disconnected"; return; }
        LLResult result = camera.getLatestResult();
        if (result == null || !result.isValid()) { status = "No valid result"; return; }
        if (result.getPipelineIndex() != getExpectedPipeline()) {
            status = "Waiting for pipeline"; return;
        }
        double timestamp = result.getTimestamp();
        if (!Double.isFinite(timestamp) || timestamp <= 0) {
            status = "Invalid camera timestamp"; return;
        }
        if (timestamp == lastTimestamp) { status = "No new frame"; return; }
        lastTimestamp = timestamp;
        long age = result.getStaleness();
        double latency = result.getCaptureLatency() + result.getTargetingLatency();
        if (age < 0 || age > VisionConstants.maxFrameAgeMs
                || !Double.isFinite(latency) || latency < 0
                || age + latency > VisionConstants.maxFrameAgeMs) {
            status = "Frame too old"; return;
        }
        Pose3D pose = result.getBotpose();
        if (result.getBotposeTagCount() < VisionConstants.minimumTagCount || pose == null
                || pose.getPosition() == null || pose.getPosition().unit == null
                || pose.getOrientation() == null) {
            status = "No MT1 pose"; return;
        }
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            status = "No tag IDs"; return;
        }
        // Red and blue HIVE structures tilt independently; never apply one correction to both.
        StringBuilder unknownTagIds = new StringBuilder();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (!(blueAlliance ? id >= 38 && id <= 45 : id >= 30 && id <= 37)) {
                if (unknownTagIds.length() > 0) unknownTagIds.append(",");
                unknownTagIds.append(id);
            }
        }
        if (unknownTagIds.length() > 0) {
            status = "Unexpected HIVE tag ID(s): " + unknownTagIds;
            return;
        }
        double distance = result.getBotposeAvgDist();
        if (!Double.isFinite(distance) || distance <= 0) {
            status = "Invalid target distance"; return;
        }
        // Camera time has a different clock origin; estimate host capture time from age + latency.
        long captureNanos = System.nanoTime() - (long) ((age + latency) * 1e6);
        frame = new Frame(pose, captureNanos, timestamp,
                result.getPipelineIndex(), result.getBotposeTagCount(), distance);
        status = "New MT1 frame";
    }

    /** Returns only a new frame from the most recent periodic call, otherwise null. */
    public Frame getNewFrame() { return frame; }
    public boolean isBlueAlliance() { return blueAlliance; }
    public int getExpectedPipeline() {
        return blueAlliance ? VisionConstants.bluePipeline : VisionConstants.redPipeline;
    }
    public String getStatus() { return status; }
    public void stop() { camera.stop(); frame = null; }
}
