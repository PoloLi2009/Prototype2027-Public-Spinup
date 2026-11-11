package org.firstinspires.ftc.teamcode.subsystems.transit;

import java.util.Objects;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.drive.ShootAssistConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

/**
 * Port of Overtune Transit.java (limit gate + transit motor + FiringState machine) onto Spinup's
 * gate servo ({@link Transit}, "limitServo") and Spinup's single intake/transit motor
 * ({@link Intake}, "transitMotor").
 *
 * <p>Overtune has separate intake and transit motors; Spinup has one. The transit request
 * (gate sequence, jam reverse) owns that motor; the ported Intake request
 * ({@link ShootAssistIntake}) is used only when the transit motor is idle, and its shot-feed
 * request is never allowed to push into a closed gate.</p>
 */
public class ShootAssistTransit {
    public enum FiringState { IDLE, WAITING, REVERSING, FEEDING }
    public enum TransitState { CLOSE, OPEN }

    private final Transit gate;
    private final Intake motor;
    private TransitState transitState = TransitState.CLOSE;
    private Intake.IntakeState requestedPower = Intake.IntakeState.STOP;
    private Intake.IntakeState intakeRequest = Intake.IntakeState.STOP;
    private final ElapsedTime feedReverseTimer = new ElapsedTime();
    private double feedReverseTimeSeconds;
    private FiringState firingState = FiringState.IDLE;

    public ShootAssistTransit(Transit gate, Intake motor) {
        this.gate = Objects.requireNonNull(gate);
        this.motor = Objects.requireNonNull(motor);
        stopMotor();
        close();
    }

    // Once the gate opens, only explicit cancellation ends the reverse/feed sequence.
    public void requestFeed(boolean readyToOpenLimit) {
        if (!isFeedActive()) {
            double reverseTime = ShootAssistConstants.feedReverseTime;
            if (!readyToOpenLimit || !areLimitPositionsCalibrated()
                    || !Double.isFinite(reverseTime) || reverseTime < 0.0) {
                cancelFeeding();
                firingState = FiringState.WAITING;
                return;
            }
            // RT takes ownership before the gate moves, including while LT remains held.
            stopMotor();
            open();
            feedReverseTimeSeconds = reverseTime;
            feedReverseTimer.reset();
            firingState = FiringState.REVERSING;
        }
        if (firingState == FiringState.REVERSING) {
            if (feedReverseTimer.seconds() >= feedReverseTimeSeconds) {
                firingState = FiringState.FEEDING;
            } else {
                setPower(Intake.IntakeState.REVERSED); // OT feedReversePower
            }
        }
        if (firingState == FiringState.FEEDING) {
            setPower(Intake.IntakeState.FEED); // OT feedPower
        }
    }

    public boolean isFeedActive() {
        return firingState == FiringState.REVERSING || firingState == FiringState.FEEDING;
    }

    public void cancelFeeding() {
        stopMotor();
        close();
        firingState = FiringState.IDLE;
        feedReverseTimer.reset();
    }

    public FiringState getFiringState() { return firingState; }

    public void forward() { setPower(Intake.IntakeState.FAST_FORWARD); }
    public void reverse() { setPower(Intake.IntakeState.FAST_REVERSED); }
    public void stopMotor() { setPower(Intake.IntakeState.STOP); }
    public Intake.IntakeState getRequestedPower() { return requestedPower; }

    public boolean areLimitPositionsCalibrated() {
        return validServoPosition(TransitConstants.gateClosedPosition)
                && validServoPosition(TransitConstants.gateOpenPosition)
                && TransitConstants.gateClosedPosition != TransitConstants.gateOpenPosition;
    }

    public void close() { setState(TransitState.CLOSE); }

    public void open() {
        if (!areLimitPositionsCalibrated()) {
            close();
            return;
        }
        setState(TransitState.OPEN);
    }

    public TransitState getState() { return transitState; }

    /** Ported Intake motor request; applied only while the transit motor request is STOP. */
    public void setIntakeRequest(Intake.IntakeState state) {
        intakeRequest = state;
        applyOutputs();
    }

    private void setState(TransitState state) {
        transitState = state;
        applyOutputs();
    }

    private void setPower(Intake.IntakeState power) {
        requestedPower = power;
        applyOutputs();
    }

    /** Writes the commanded gate and single-motor states; Spinup periodic() sends them to hardware. */
    public void applyOutputs() {
        gate.setGateState(transitState == TransitState.OPEN
                ? Transit.GateState.OPEN : Transit.GateState.CLOSED);
        Intake.IntakeState output = requestedPower;
        if (output == Intake.IntakeState.STOP && intakeRequest != Intake.IntakeState.FEED) {
            output = intakeRequest;
        }
        motor.setIntakeState(output);
    }

    private static boolean validServoPosition(double position) {
        return Double.isFinite(position) && position >= 0.0 && position <= 1.0;
    }
}
