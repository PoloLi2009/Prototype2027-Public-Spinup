package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.subsystems.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.drive.ShootAssistConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.ShootAssistIntake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShootAssistShooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.ShootAssistTransit;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;

import java.util.function.BooleanSupplier;

/**
 * Overtune AutoShootCommand, line for line: stops the follower and runs a distance-based shot
 * using Pedro's field pose. Schedule sequentially after a path, and keep calling
 * PedroDrive.periodic(), ShootAssistShooter.periodic(), the Spinup Intake/Transit periodic() and
 * ShootAssistVision.update(). Not called by any autonomous yet.
 *
 * <p>Spinup differences are only at the hardware boundary: PedroDrive wraps Overtune's Follower,
 * and the ShootAssist* adapters are not FTCLib subsystems, so no requirements are declared.</p>
 */
public class AutoShootCommand extends CommandBase {
    private final PedroDrive follower;
    private final ShootAssistShooter shooter;
    private final ShootAssistIntake intake;
    private final ShootAssistTransit transit;
    private final AutoHiveTarget target;
    private final BooleanSupplier localizationReady;
    private final long feedDurationMs;
    private final long timeoutMs;
    private final boolean delayBeforeShooting;

    private long startedNanos;
    private long feedStartedNanos = -1;
    private boolean waitingForDelay;
    private boolean timedOut;
    private String status = "Not started";

    public AutoShootCommand(PedroDrive follower, ShootAssistShooter shooter, ShootAssistIntake intake,
                            ShootAssistTransit transit, AutoHiveTarget target,
                            BooleanSupplier localizationReady,
                            long feedDurationMs, long timeoutMs, boolean delayBeforeShooting) {
        if (feedDurationMs <= 0 || timeoutMs < feedDurationMs) {
            throw new IllegalArgumentException("Shot timeout must cover a positive feed duration");
        }
        this.follower = follower;
        this.shooter = shooter;
        this.intake = intake;
        this.transit = transit;
        this.target = target;
        this.localizationReady = localizationReady;
        this.feedDurationMs = feedDurationMs;
        this.timeoutMs = timeoutMs;
        this.delayBeforeShooting = delayBeforeShooting;
    }

    @Override
    public void initialize() {
        startedNanos = System.nanoTime();
        feedStartedNanos = -1;
        waitingForDelay = delayBeforeShooting;
        timedOut = false;
        status = "Preparing shot";
        follower.stop();
        transit.cancelFeeding();
        intake.updateShotFeed(false);
        // Preserve a distance target already spun up while following the preceding path.
    }

    @Override
    public void execute() {
        if (isFinished()) return;
        long now = System.nanoTime();
        if (waitingForDelay && (now - startedNanos) / 1_000_000L >= ShootAssistConstants.preShotDelayMs) {
            waitingForDelay = false;
            startedNanos = now;
        }

        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        if (!localizationReady.getAsBoolean() || pose == null || velocity == null
                || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading()) || !Double.isFinite(velocity.linSpeed())
                || !Double.isFinite(velocity.omega)) {
            stopForInvalidTarget("Waiting for valid localization");
            return;
        }

        target.update(pose);
        if (!shooter.setDistanceTarget(target.getDistanceCm())) {
            stopForInvalidTarget("Waiting for a valid shot target");
            return;
        }

        if (waitingForDelay) {
            status = "Waiting " + ShootAssistConstants.preShotDelayMs + " ms before shot";
            return;
        }

        boolean speedReady = shooter.isReady();
        transit.requestFeed(speedReady);
        ShootAssistTransit.FiringState firingState = transit.getFiringState();
        boolean feeding = firingState == ShootAssistTransit.FiringState.FEEDING;
        intake.updateShotFeed(feeding);
        if (feeding) {
            if (feedStartedNanos < 0) feedStartedNanos = System.nanoTime();
            status = "Shooting: " + feedDurationMs + " ms";
        } else if (firingState == ShootAssistTransit.FiringState.REVERSING) {
            status = "Reversing before feed";
        } else if (!speedReady) {
            status = "Waiting for shooter speed";
        } else {
            status = "Invalid transit gate or reverse settings";
        }
    }

    @Override
    public boolean isFinished() {
        long now = System.nanoTime();
        if (feedStartedNanos >= 0) {
            return (now - feedStartedNanos) / 1_000_000L >= feedDurationMs;
        }
        timedOut = !waitingForDelay && (now - startedNanos) / 1_000_000L >= timeoutMs;
        return timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        if (transit.getState() == ShootAssistTransit.TransitState.OPEN) intake.clearBallCount();
        transit.cancelFeeding();
        intake.stop();
        shooter.stop();
        follower.stop();
        if (interrupted) {
            status = "Shot interrupted";
        } else if (timedOut) {
            status = "Shot timed out (" + status + ")";
        } else {
            status = "Shot complete";
        }
    }

    public String getStatus() { return status; }
    public boolean hasTimedOut() { return timedOut; }

    private void stopForInvalidTarget(String reason) {
        if (transit.getState() == ShootAssistTransit.TransitState.OPEN) intake.clearBallCount();
        transit.cancelFeeding();
        intake.updateShotFeed(false);
        feedStartedNanos = -1;
        status = reason;
        shooter.stop();
        follower.stop();
    }
}
