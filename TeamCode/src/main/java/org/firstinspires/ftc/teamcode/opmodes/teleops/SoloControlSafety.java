package org.firstinspires.ftc.teamcode.opmodes.teleops;

/** Require neutral controls after STOP or a drive-coordinate change. No hardware dependencies. */
public final class SoloControlSafety {
    private boolean stopped = true;
    private boolean frameChanged;
    private boolean previousFieldMode;

    public boolean update(boolean stopPressed, boolean controlsNeutral, boolean fieldMode) {
        if (fieldMode != previousFieldMode) frameChanged = true;
        previousFieldMode = fieldMode;
        if (stopPressed) stopped = true;
        else if (controlsNeutral) {
            stopped = false;
            frameChanged = false;
        }
        return stopped || frameChanged;
    }

    public void requireNeutral() { stopped = true; }
}
