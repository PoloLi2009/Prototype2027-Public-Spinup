package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/** Shared closed-loop shot: qualify RPM, open gate, then feed while aim and RPM remain valid. */
public class ShootCommand extends CommandBase {
    public enum State { IDLE, AIMING, SPINNING_UP, OPENING_GATE, FEEDING }
    private long readyHoldMs = 250;
    private static final long GATE_OPEN_DELAY_MS = 500;
    private final FlywheelRpmCommand spin;
    private final RunIntakeCommand feed;
    private final Intake intake;
    private final Shooter shooter;
    private final Transit transit;
    private final DoubleSupplier targetRpm;
    private final BooleanSupplier aimReady;
    private final boolean spinWhileAiming;
    private final ElapsedTime readyTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();
    private final ElapsedTime autoTimer;
    private final long autoFeedMs, autoTimeoutMs;
    private double autoFeedStartedMs = -1;
    private State shotState = State.IDLE;

    /**
     * Finite autonomous shot at the current pitch. Durations are milliseconds;
     * feedMs counts continuous feed and timeoutMs bounds the entire command.
     * Run with CommandScheduler (including subsystem periodic updates).
     * Does not claim or write Drive: run a separate aim command and pass its readiness.
     */
    public ShootCommand(Intake intake, Shooter shooter, Transit transit, double targetRpm,
                        BooleanSupplier aimReady, long feedMs, long timeoutMs) {
        this(intake, shooter, transit, () -> targetRpm, aimReady, true, feedMs, timeoutMs);
        if (!Double.isFinite(targetRpm) || targetRpm <= 0 || targetRpm > ShooterConstants.maxRpm
                || feedMs <= 0 || timeoutMs <= 0) {
            throw new IllegalArgumentException("Require valid RPM and positive feed/timeout milliseconds");
        }
    }

    /** Continuous shared action; the owning manual command controls release/cancellation. */
    public ShootCommand(Intake intake, Shooter shooter, Transit transit, DoubleSupplier targetRpm,
                        BooleanSupplier aimReady, boolean spinWhileAiming) {
        this(intake, shooter, transit, targetRpm, aimReady, spinWhileAiming, 0, 0);
    }

    private ShootCommand(Intake intake, Shooter shooter, Transit transit, DoubleSupplier targetRpm,
                         BooleanSupplier aimReady, boolean spinWhileAiming, long feedMs, long timeoutMs) {
        this.intake = intake;
        this.shooter = shooter;
        this.transit = transit;
        this.targetRpm = Objects.requireNonNull(targetRpm);
        this.aimReady = Objects.requireNonNull(aimReady);
        this.spinWhileAiming = spinWhileAiming;
        autoFeedMs = feedMs;
        autoTimeoutMs = timeoutMs;
        autoTimer = timeoutMs > 0 ? new ElapsedTime() : null;
        spin = new FlywheelRpmCommand(shooter, targetRpm);
        feed = new RunIntakeCommand(intake, Intake.IntakeState.FEED);
        addRequirements(intake, shooter, transit);
    }

    @Override
    public void initialize() {
        stopShot();
        autoFeedStartedMs = -1;
        readyTimer.reset();
        gateTimer.reset();
        if (autoTimer != null) {
            applyStoppedOutputs();
            autoTimer.reset();
        }
    }

    @Override
    public void execute() {
        if (isFinished()) return;
        State previousState = shotState;
        executeShot();
        if (autoTimer != null && shotState != previousState) {
            // Scheduler periodic precedes execute: start timers after physical gate/feed writes.
            intake.periodic();
            transit.periodic();
            if (shotState == State.OPENING_GATE) gateTimer.reset();
            autoFeedStartedMs = shotState == State.FEEDING ? autoTimer.milliseconds() : -1;
        }
    }

    private void executeShot() {
        boolean aimed = aimReady.getAsBoolean();
        if (aimed || spinWhileAiming) {
            spin.execute();
        } else {
            shooter.stop();
        }
        boolean atSpeed = shooter.isAtTargetRpm();
        if (!atSpeed || shotState == State.IDLE
                || (shotState == State.AIMING && !spinWhileAiming)) {
            readyTimer.reset();
        }

        // Spin-up may overlap aiming, but losing aim always closes the gate and stops feed.
        if (!aimed) {
            shotState = State.AIMING;
            transit.setGateState(Transit.GateState.CLOSED);
            intake.stop();
            return;
        }

        if (shotState == State.IDLE || shotState == State.AIMING
               ) {
            shotState = State.SPINNING_UP;
        }
        // Requalify after a speed drop, including while the gate is opening or feeding.
        // Otherwise a second ball can enter before the flywheel has recovered.
        if (!atSpeed) {
            shotState = State.SPINNING_UP;
        }
        switch (shotState) {
            case SPINNING_UP:
                transit.setGateState(Transit.GateState.CLOSED);
                intake.stop();
                if (atSpeed && readyTimer.milliseconds() >= readyHoldMs) {
                    transit.setGateState(Transit.GateState.OPEN);
                    gateTimer.reset();
                    shotState = State.OPENING_GATE;
                }
                break;
            case OPENING_GATE:
                transit.setGateState(Transit.GateState.OPEN);
                intake.stop();
                if (gateTimer.milliseconds() >= GATE_OPEN_DELAY_MS) {
                    feed.execute();
                    shotState = State.FEEDING;
                }
                break;
            case FEEDING:
                transit.setGateState(Transit.GateState.OPEN);
                feed.execute();
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        if (autoTimer == null) return false;
        double elapsedMs = autoTimer.milliseconds();
        return elapsedMs >= autoTimeoutMs
                || (autoFeedStartedMs >= 0 && elapsedMs - autoFeedStartedMs >= autoFeedMs);
    }

    public void setReadyHoldMilliseconds(long milliseconds) {
        if (milliseconds < 0) throw new IllegalArgumentException("Negative readiness hold");
        readyHoldMs = milliseconds;
        restartReadiness();
    }

    // The manual input adapter owns the lifecycle of this child; never schedule it separately.
    void resetState() { shotState = State.IDLE; }
    void restartReadiness() { shotState = State.SPINNING_UP; readyTimer.reset(); }
    public State getShotState() { return shotState; }
    public double getTargetRpm() { return targetRpm.getAsDouble(); }
    public long getGateElapsedMs() {
        return shotState == State.OPENING_GATE
                ? Math.min((long) gateTimer.milliseconds(), GATE_OPEN_DELAY_MS) : 0;
    }

    @Override
    public void end(boolean interrupted) {
        stopShot();
        if (autoTimer != null) applyStoppedOutputs();
    }

    private void stopShot() {
        intake.stop();
        shooter.stop();
        transit.setGateState(Transit.GateState.CLOSED);
        resetState();
    }

    private void applyStoppedOutputs() {
        intake.periodic();
        shooter.periodic();
        transit.periodic();
    }
}
