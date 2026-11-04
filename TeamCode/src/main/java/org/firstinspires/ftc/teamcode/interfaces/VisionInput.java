package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Reserved integration boundary for a future vision program.  Nothing in the
 * current robot code reads this interface or starts a camera pipeline.
 */
public interface VisionInput {
    Observation getLatestObservation();

    final class Observation {
        public final double xMm;
        public final double yMm;
        public final double headingDegrees;
        public final long captureNanos;
        public final boolean valid;

        public Observation(double xMm, double yMm, double headingDegrees,
                           long captureNanos, boolean valid) {
            this.xMm = xMm;
            this.yMm = yMm;
            this.headingDegrees = headingDegrees;
            this.captureNanos = captureNanos;
            this.valid = valid;
        }
    }
}
