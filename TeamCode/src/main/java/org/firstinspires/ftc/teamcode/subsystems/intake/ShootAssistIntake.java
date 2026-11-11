package org.firstinspires.ftc.teamcode.subsystems.intake;

import java.util.Objects;

import org.firstinspires.ftc.teamcode.subsystems.transit.ShootAssistTransit;

/**
 * Port of the Overtune Intake.java methods used by ShootAssistCommand / AutoShootCommand
 * (updateInputs / updateShotFeed / setReverseRequested / clearBallCount / suspend / stop).
 * Spinup has one intake/transit motor and no ball-count distance sensor: the request is merged by
 * {@link ShootAssistTransit} and ball counting (full-hopper reverse) is not available.
 */
public class ShootAssistIntake {
    private final ShootAssistTransit transit;
    private boolean jamRequested;
    private boolean reverseRequested;
    private boolean intakeRequested;
    private boolean shootRequested;
    private boolean shotFeedEnabled = true;

    public ShootAssistIntake(ShootAssistTransit transit) {
        this.transit = Objects.requireNonNull(transit);
        stop();
    }

    public void updateInputs(boolean intakeRequested, boolean jamRequested) {
        updateInputs(intakeRequested, jamRequested, false);
    }

    public void updateInputs(boolean intakeRequested, boolean jamRequested, boolean shootRequested) {
        if (this.jamRequested && !jamRequested) {
            clearBallCount();
        }
        this.jamRequested = jamRequested;
        this.intakeRequested = intakeRequested;
        this.shootRequested = shootRequested;
        shotFeedEnabled = true;
        updateMotorPower();
    }

    /** Holds collected balls while aiming; feeds only after the shot gate opens. */
    public void updateShotFeed(boolean feeding) {
        jamRequested = false;
        reverseRequested = false;
        intakeRequested = false;
        shootRequested = true;
        shotFeedEnabled = feeding;
        updateMotorPower();
    }

    public void setReverseRequested(boolean reverseRequested) {
        this.reverseRequested = reverseRequested;
        updateMotorPower();
    }

    /** No Spinup ball counter; kept so the Overtune call sites stay identical. */
    public void clearBallCount() {
        updateMotorPower();
    }

    public void suspend() {
        shootRequested = false;
        transit.setIntakeRequest(Intake.IntakeState.STOP);
    }

    public void stop() {
        jamRequested = false;
        reverseRequested = false;
        intakeRequested = false;
        shootRequested = false;
        transit.setIntakeRequest(Intake.IntakeState.STOP);
    }

    private void updateMotorPower() {
        if (jamRequested || reverseRequested) {
            transit.setIntakeRequest(Intake.IntakeState.FAST_REVERSED); // OT jamPower
        } else if (shootRequested) {
            transit.setIntakeRequest(shotFeedEnabled ? Intake.IntakeState.FEED : Intake.IntakeState.STOP);
        } else if (intakeRequested) {
            transit.setIntakeRequest(Intake.IntakeState.FAST_FORWARD); // OT activePower
        } else {
            transit.setIntakeRequest(Intake.IntakeState.STOP);
        }
    }
}
