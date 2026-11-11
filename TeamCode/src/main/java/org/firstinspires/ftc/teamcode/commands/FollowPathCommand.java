package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Objects;
import org.firstinspires.ftc.teamcode.subsystems.drive.PedroDrive;

/** Overtune-style follow/isBusy completion, with Spinup's requested 5-second per-path budget. */
public class FollowPathCommand extends CommandBase {
    public static final long SEGMENT_TIMEOUT_MS = 5000;
    private final PedroDrive drive;
    private final Path[] paths;
    private final ElapsedTime segmentTimer = new ElapsedTime();
    private int segment, timedOutSegments;
    private boolean poseLost, waitingAtShotPoint;
    private final long shotWindowMs;
    private final java.util.Set<Integer> shotEndpoints = new java.util.HashSet<>();
    private final ElapsedTime shotTimer = new ElapsedTime();

    public FollowPathCommand(PedroDrive drive, Path... paths) {
        this(drive, 0, new int[0], paths);
    }

    /** Reserved shot stops only; never starts a shooter or feeder. Endpoint numbers are one-based. */
    public FollowPathCommand(PedroDrive drive, long shotWindowMs, int[] shotEndpointNumbers, Path... paths) {
        if (paths.length == 0) throw new IllegalArgumentException("At least one path is required");
        if (shotWindowMs < 0 || (shotEndpointNumbers.length > 0 && shotWindowMs == 0))
            throw new IllegalArgumentException("Shot window must have a finite positive duration");
        this.shotWindowMs = shotWindowMs;
        for (int endpoint : shotEndpointNumbers) {
            if (endpoint < 1 || endpoint > paths.length) throw new IllegalArgumentException("Invalid shot endpoint");
            shotEndpoints.add(endpoint);
        }
        this.drive = Objects.requireNonNull(drive);
        this.paths = paths.clone();
        for (Path path : this.paths) Objects.requireNonNull(path);
        addRequirements(drive);
    }
    @Override public void initialize() {
        segment = 0;
        timedOutSegments = 0;
        waitingAtShotPoint = false;
        poseLost = !drive.isReady();
        if (poseLost) drive.stop();
        else startSegment();
    }
    private void startSegment() {
        drive.follow(paths[segment]);
        segmentTimer.reset();
    }
    @Override public void execute() {
        if (isFinished()) return;
        if (!drive.isReady()) {
            poseLost = true;
            drive.stop();
            return;
        }
        if (waitingAtShotPoint) {
            drive.stop();
            if (shotTimer.milliseconds() >= shotWindowMs) {
                waitingAtShotPoint = false;
                advanceSegment();
            }
            return;
        }
        boolean timedOut = segmentTimer.milliseconds() >= SEGMENT_TIMEOUT_MS;
        if (timedOut || !drive.isBusy()) {
            if (timedOut) { timedOutSegments++; drive.stop(); }
            if (shotEndpoints.contains(segment + 1)) {
                drive.stop();
                waitingAtShotPoint = true;
                shotTimer.reset();
            } else advanceSegment();
        }
    }
    private void advanceSegment() {
        segment++;
        if (segment < paths.length) startSegment();
    }
    public int getReservedShotEndpoint() { return waitingAtShotPoint ? segment + 1 : 0; }
    public long getReservedShotRemainingMs() {
        return waitingAtShotPoint ? Math.max(0, shotWindowMs - (long) shotTimer.milliseconds()) : 0;
    }
    @Override public boolean isFinished() { return poseLost || segment >= paths.length; }
    public boolean hasLostPose() { return poseLost; }
    public int getTimedOutSegments() { return timedOutSegments; }
    public int segmentNumber() { return Math.min(segment + 1, paths.length); }
    @Override public void end(boolean interrupted) { waitingAtShotPoint = false; drive.stop(); }
}
