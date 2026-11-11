package org.firstinspires.ftc.teamcode.subsystems.shooter;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.subsystems.drive.ShootAssistConstants;

/**
 * Overtune Shooter API used by ShootAssistCommand / AutoShootCommand
 * (setDistanceTarget / isReady / isPitchReady / stop), on top of Spinup's own {@link Shooter}
 * PIDF and right-encoder feedback.
 *
 * <p>setDistanceTarget is Overtune's adaptive shot, copied: interpolate {distance, ticks/s, pitch}
 * between the bracketing rows of near+far tables (clamped beyond the ends), optionally move the
 * pitch, then use the fixed override velocity when it is positive, otherwise table velocity x
 * scale. Tables live in {@link ShootAssistConstants}. Readiness keeps Overtune's enter/exit
 * hysteresis and stable-time logic with Spinup's +/-30 TPS window.</p>
 */
public class ShootAssistShooter {
    private final Shooter shooter;
    private static final int SHOT_POINTS_PER_RANGE = 5;
    private final DoubleSupplier distanceVelocityScale;
    // Zero keeps distance-based velocity; positive values override velocity only, not pitch.
    private final DoubleSupplier fixedDistanceTargetVelocity;
    private double targetVelocity = Double.NaN;
    private double velocitySettleReferenceTarget = Double.NaN;
    private boolean velocityControlActive;
    private boolean ready;
    private long withinToleranceSinceNanos = -1;
    private double pitchSettleReferencePosition;
    private long pitchChangedAtNanos;

    /** Distance table velocity and pitch (Overtune Shooter(hardwareMap)). */
    public ShootAssistShooter(Shooter shooter) {
        this(shooter, () -> 1.0, () -> 0.0);
    }

    /**
     * @param distanceVelocityScale  Overtune per-instance scale of table velocity, (0, 1]
     * @param fixedDistanceTargetVelocity positive ticks/s overrides table velocity (pitch still follows distance)
     */
    public ShootAssistShooter(Shooter shooter, DoubleSupplier distanceVelocityScale,
                              DoubleSupplier fixedDistanceTargetVelocity) {
        this.shooter = Objects.requireNonNull(shooter);
        this.distanceVelocityScale = Objects.requireNonNull(distanceVelocityScale);
        this.fixedDistanceTargetVelocity = Objects.requireNonNull(fixedDistanceTargetVelocity);
        pitchSettleReferencePosition = shooter.getPitchPosition();
        pitchChangedAtNanos = System.nanoTime();
    }

    public boolean setDistanceTarget(double distanceCm) {
        return setDistanceTarget(distanceCm, true);
    }

    /** Overtune Shooter.setDistanceTarget, line for line (Spinup readiness check in place of PID calibration). */
    public boolean setDistanceTarget(double distanceCm, boolean updatePitch) {
        double[][] points = getDistanceShotPoints();
        double scale = distanceVelocityScale.getAsDouble();
        if (points == null || !areReadyValuesValid()
                || !Double.isFinite(distanceCm) || distanceCm < 0.0
                || !Double.isFinite(scale) || scale <= 0.0 || scale > 1.0) {
            stop();
            return false;
        }
        double[] lower = null;
        double[] upper = null;
        for (double[] point : points) {
            if (point[0] <= distanceCm && (lower == null || point[0] > lower[0])) lower = point;
            if (point[0] >= distanceCm && (upper == null || point[0] < upper[0])) upper = point;
        }
        // Adjacent points also cover the gap between tables. Clamp beyond the outer endpoints.
        if (lower == null) lower = upper;
        if (upper == null) upper = lower;
        double fraction = lower == upper ? 0.0
                : (distanceCm - lower[0]) / (upper[0] - lower[0]);
        double velocity = (1.0 - fraction) * lower[1] + fraction * upper[1];
        if (updatePitch) {
            double pitch = (1.0 - fraction) * lower[2] + fraction * upper[2];
            setPitchPosition(pitch);
        }
        double fixedVelocity = fixedDistanceTargetVelocity.getAsDouble();
        setTargetVelocity(fixedVelocity > 0.0 ? fixedVelocity : velocity * scale);
        return hasTargetVelocity();
    }

    public static boolean areDistanceShotValuesValid() {
        return areReadyValuesValid() && getDistanceShotPoints() != null;
    }

    /** Returns the ordered near and far points, or null when the tables are invalid. */
    public static double[][] getDistanceShotPoints() {
        double[][] near = ShootAssistConstants.nearShotPoints;
        double[][] far = ShootAssistConstants.farShotPoints;
        if (near == null || far == null
                || near.length != SHOT_POINTS_PER_RANGE || far.length != SHOT_POINTS_PER_RANGE
                || !Double.isFinite(ShootAssistConstants.pitchMinPosition)
                || !Double.isFinite(ShootAssistConstants.pitchMaxPosition)
                || ShootAssistConstants.pitchMinPosition < 0.0
                || ShootAssistConstants.pitchMaxPosition > 1.0
                || ShootAssistConstants.pitchMinPosition > ShootAssistConstants.pitchMaxPosition) {
            return null;
        }
        double[][] points = new double[near.length + far.length][];
        System.arraycopy(near, 0, points, 0, near.length);
        System.arraycopy(far, 0, points, near.length, far.length);
        for (int i = 0; i < points.length; i++) {
            double[] point = points[i];
            if (point == null || point.length != 3
                    || !isDistancePointValid(point[0], point[1], point[2])
                    || (i > 0 && point[0] <= points[i - 1][0])) return null;
        }
        return points;
    }

    private static boolean isDistancePointValid(double distanceCm, double velocity, double pitch) {
        return Double.isFinite(distanceCm) && distanceCm > 0.0
                && Double.isFinite(velocity) && velocity > 0.0
                && Double.isFinite(pitch) && pitch >= ShootAssistConstants.pitchMinPosition
                && pitch <= ShootAssistConstants.pitchMaxPosition;
    }

    /** Overtune setPitchPosition onto Spinup's two pitch servos (Shooter clamps to 0..1). */
    public void setPitchPosition(double position) {
        if (!Double.isFinite(position)
                || !Double.isFinite(ShootAssistConstants.pitchMinPosition)
                || !Double.isFinite(ShootAssistConstants.pitchMaxPosition)
                || ShootAssistConstants.pitchMinPosition < 0.0
                || ShootAssistConstants.pitchMaxPosition > 1.0
                || ShootAssistConstants.pitchMinPosition > ShootAssistConstants.pitchMaxPosition) {
            stop();
            return;
        }
        double clippedPosition = Math.max(ShootAssistConstants.pitchMinPosition,
                Math.min(ShootAssistConstants.pitchMaxPosition, position));
        shooter.adjustPitch(clippedPosition - shooter.getPitchPosition());
        trackPitch(); // Restarts the pitch settle timer when the change exceeds the threshold.
    }

    public boolean isPitchReady() {
        trackPitch();
        double pitchPosition = shooter.getPitchPosition();
        return Double.isFinite(pitchPosition)
                && hasTargetVelocity()
                && Double.isFinite(ShootAssistConstants.pitchSettleTimeSeconds)
                && ShootAssistConstants.pitchSettleTimeSeconds >= 0.0
                && (System.nanoTime() - pitchChangedAtNanos) / 1_000_000_000.0
                >= ShootAssistConstants.pitchSettleTimeSeconds;
    }

    public void setTargetVelocity(double velocityTicksPerSecond) {
        if (!Double.isFinite(velocityTicksPerSecond) || velocityTicksPerSecond <= 0.0) {
            stop();
            return;
        }
        if (!velocityControlActive
                || !Double.isFinite(targetVelocity)
                || !Double.isFinite(velocitySettleReferenceTarget)
                || Math.abs(velocityTicksPerSecond - velocitySettleReferenceTarget)
                > ShootAssistConstants.targetChangeResetThresholdTicksPerSecond) {
            velocitySettleReferenceTarget = velocityTicksPerSecond;
            resetReadyState();
        }
        targetVelocity = velocityTicksPerSecond;
        velocityControlActive = true;
        shooter.startFlywheelsAtTargetRpm(
                velocityTicksPerSecond * 60.0 / ShooterConstants.rightEncoderTicksPerRevolution);
    }

    public void stop() {
        velocityControlActive = false;
        targetVelocity = Double.NaN;
        velocitySettleReferenceTarget = Double.NaN;
        resetReadyState();
        shooter.stop();
    }

    public boolean hasTargetVelocity() {
        return velocityControlActive && Double.isFinite(targetVelocity);
    }

    public double getTargetVelocity() { return targetVelocity; }
    public boolean isReady() { return ready; }
    public Shooter getShooter() { return shooter; }

    /** Overtune Shooter.periodic: run Spinup's PIDF, then update readiness from the right encoder. */
    public void periodic() {
        trackPitch();
        if (!velocityControlActive || !Double.isFinite(targetVelocity)) {
            if (velocityControlActive) stop();
            resetReadyState();
            shooter.periodic();
            return;
        }
        shooter.periodic();
        double rightVelocity = shooter.getFlywheelVelocityTicksPerSecond();
        if (!Double.isFinite(rightVelocity)) {
            stop();
            return;
        }
        updateReadyState(rightVelocity);
    }

    private void updateReadyState(double rightVelocity) {
        double error = Math.abs(targetVelocity - rightVelocity);
        if (ready) {
            if (error > ShootAssistConstants.readyExitToleranceTicksPerSecond) {
                resetReadyState();
            }
            return;
        }
        if (error > ShootAssistConstants.readyEnterToleranceTicksPerSecond) {
            withinToleranceSinceNanos = -1;
            return;
        }
        long now = System.nanoTime();
        if (withinToleranceSinceNanos < 0) {
            withinToleranceSinceNanos = now;
            // Spinup Solo opens with no hold time; Overtune needs a second in-band sample.
            if (ShootAssistConstants.readyStableTimeSeconds > 0.0) return;
        }
        double stableSeconds = (now - withinToleranceSinceNanos) / 1_000_000_000.0;
        ready = stableSeconds >= ShootAssistConstants.readyStableTimeSeconds;
    }

    private void trackPitch() {
        double position = shooter.getPitchPosition();
        if (Math.abs(position - pitchSettleReferencePosition)
                > ShootAssistConstants.pitchChangeResetThreshold) {
            pitchChangedAtNanos = System.nanoTime();
            pitchSettleReferencePosition = position;
        }
    }

    private void resetReadyState() {
        ready = false;
        withinToleranceSinceNanos = -1;
    }

    private static boolean areReadyValuesValid() {
        return Double.isFinite(ShootAssistConstants.readyEnterToleranceTicksPerSecond)
                && ShootAssistConstants.readyEnterToleranceTicksPerSecond > 0.0
                && Double.isFinite(ShootAssistConstants.readyExitToleranceTicksPerSecond)
                && ShootAssistConstants.readyExitToleranceTicksPerSecond
                >= ShootAssistConstants.readyEnterToleranceTicksPerSecond
                && Double.isFinite(ShootAssistConstants.readyStableTimeSeconds)
                && ShootAssistConstants.readyStableTimeSeconds >= 0.0
                && Double.isFinite(ShootAssistConstants.targetChangeResetThresholdTicksPerSecond)
                && ShootAssistConstants.targetChangeResetThresholdTicksPerSecond >= 0.0;
    }
}
