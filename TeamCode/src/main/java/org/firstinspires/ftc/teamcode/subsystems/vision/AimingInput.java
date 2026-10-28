package org.firstinspires.ftc.teamcode.subsystems.vision;

/**
 * Vision integration boundary. Coordinates: mm, origin at the blue/audience
 * inner corner; +X toward red, +Y away from audience. Heading: radians,
 * zero along +X, counterclockwise viewed from above.
 */
public interface AimingInput {
    enum Cell { A, B, UNKNOWN }

    /** Immutable coherent observation; capture time uses System.nanoTime(). */
    final class Observation {
        public final double xMm, yMm, headingRadians;
        public final Cell redRaisedCell, blueRaisedCell;
        public final long captureNanos;
        public final boolean valid;

        public Observation(double xMm, double yMm, double headingRadians,
                           Cell redRaisedCell, Cell blueRaisedCell,
                           long captureNanos, boolean valid) {
            this.xMm = xMm;
            this.yMm = yMm;
            this.headingRadians = headingRadians;
            this.redRaisedCell = redRaisedCell;
            this.blueRaisedCell = blueRaisedCell;
            this.captureNanos = captureNanos;
            this.valid = valid;
        }
    }

    Observation getObservation();
}
