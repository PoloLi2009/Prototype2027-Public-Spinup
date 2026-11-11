package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.utils.TurretMath;

/** Holds the turret's initial field direction; right stick press captures chassis forward. */
public class TurretHeadingHoldCommand extends CommandBase {
    private final Turret turret;
    private final Pinpoint pinpoint;
    private final Gamepad gamepad;
    private boolean holdInitialized;
    private boolean lastRightStickButton;
    private double holdWorldDegrees;
    private double targetDegrees = Double.NaN;
    private boolean tracking;

    public TurretHeadingHoldCommand(Turret turret, Pinpoint pinpoint, Gamepad gamepad) {
        this.turret = turret;
        this.pinpoint = pinpoint;
        this.gamepad = gamepad;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        lastRightStickButton = false;
    }

    @Override
    public void execute() {
        if (!pinpoint.isReady()) {
            turret.stop();
            tracking = false;
            return;
        }
        if (!pinpoint.isFieldPoseInitialized()) {
            pinpoint.setFieldPose(0, 0, 0);
            pinpoint.periodic();
        }
        double headingDegrees = pinpoint.getHeadingDegrees();
        if (!holdInitialized) {
            holdWorldDegrees = turret.getAngleDegrees() - headingDegrees;
            holdInitialized = true;
        }
        if (gamepad.right_stick_button && !lastRightStickButton) {
            // At the press, request 0 degrees relative to the current chassis heading.
            holdWorldDegrees = -headingDegrees;
        }
        lastRightStickButton = gamepad.right_stick_button;

        targetDegrees = TurretMath.reachableDegrees(holdWorldDegrees + headingDegrees,
                turret.getAngleDegrees(), TurretConstants.minDegrees(),
                TurretConstants.maxDegrees());
        tracking = turret.aimAtPhysicalDegrees(targetDegrees, 0.0,
                TurretConstants.headingHoldMaxPower, TurretConstants.headingHoldKp);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    public double getTargetDegrees() { return targetDegrees; }
    public boolean isTracking() { return tracking; }
}
