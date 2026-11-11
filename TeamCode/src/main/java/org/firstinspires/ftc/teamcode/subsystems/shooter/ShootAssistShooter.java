package org.firstinspires.ftc.teamcode.subsystems.shooter;

import java.util.Objects;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.subsystems.drive.ShootAssistConstants;

/**
 * Overtune Shooter API used by ShootAssistCommand / AutoShootCommand
 * (setDistanceTarget / isReady / isPitchReady / stop), on top of Spinup's own {@link Shooter}
 * PIDF and right-encoder feedback.
 *
 * <p>Spinup has no distance shot table, so setDistanceTarget follows Overtune's
 * "fixed distance target velocity" branch: any valid distance selects the supplied fixed TPS
 * (Solo: 900 TPS, D-pad adjustable) and pitch is left at Spinup's current preset. Readiness keeps
 * Overtune's enter/exit hysteresis and stable-time logic with Spinup's +/-30 TPS window.</p>
 */
public class ShootAssistShooter {
    private final Shooter shooter;
    private final DoubleSupplier fixedTargetTicksPerSecond;
    private double targetVelocity = Double.NaN;
    private double velocitySettleReferenceTarget = Double.NaN;
    private boolean velocityControlActive;
    private boolean ready;
    private long withinToleranceSinceNanos = -1;
    private double pitchSettleReferencePosition;
    private long pitchChangedAtNanos;

    public ShootAssistShooter(Shooter shooter, DoubleSupplier fixedTargetTicksPerSecond) {
        this.shooter = Objects.requireNonNull(shooter);
        this.fixedTargetTicksPerSecond = Objects.requireNonNull(fixedTargetTicksPerSecond);
        pitchSettleReferencePosition = shooter.getPitchPosition();
        pitchChangedAtNanos = System.nanoTime();
    }

    public boolean setDistanceTarget(double distanceCm) {
        if (!Double.isFinite(distanceCm) || distanceCm < 0.0 || !areReadyValuesValid()) {
            stop();
            return false;
        }
        setTargetVelocity(fixedTargetTicksPerSecond.getAsDouble());
        return hasTargetVelocity();
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
