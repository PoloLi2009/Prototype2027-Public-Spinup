package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.field.CellTarget;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.utils.TurretMath;
import java.util.function.BooleanSupplier;

/** Selects the alliance CELL from OD pose and tracks it with the turret. */
public class OdCellAimCommand extends CommandBase {
    private final Turret turret;
    private final Pinpoint pinpoint;
    private final Gamepad gamepad;
    private final double startXmm;
    private final double startYmm;
    private final double startHeadingDegrees;
    private final boolean toggleWithY;
    private final BooleanSupplier aimEnabled;
    private boolean blueAlliance = true;
    private boolean lastToggle;
    private CellTarget target;
    private double targetDegrees = Double.NaN;
    private boolean tracking;

    public OdCellAimCommand(Turret turret, Pinpoint pinpoint, Gamepad gamepad,
                            double startXmm, double startYmm, double startHeadingDegrees) {
        this(turret, pinpoint, gamepad, startXmm, startYmm, startHeadingDegrees,
                false, () -> true);
    }

    public OdCellAimCommand(Turret turret, Pinpoint pinpoint, Gamepad gamepad,
                            double startXmm, double startYmm, double startHeadingDegrees,
                            boolean toggleWithY, BooleanSupplier aimEnabled) {
        this.turret = turret;
        this.pinpoint = pinpoint;
        this.gamepad = gamepad;
        this.startXmm = startXmm;
        this.startYmm = startYmm;
        this.startHeadingDegrees = startHeadingDegrees;
        this.toggleWithY = toggleWithY;
        this.aimEnabled = aimEnabled;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        lastToggle = toggleWithY ? gamepad.y : gamepad.x;
    }

    @Override
    public void execute() {
        boolean togglePressed = toggleWithY ? gamepad.y : gamepad.x;
        if (togglePressed && !lastToggle) blueAlliance = !blueAlliance;
        lastToggle = togglePressed;

        if (!pinpoint.isReady()) {
            target = null;
            targetDegrees = Double.NaN;
            tracking = false;
            turret.stop();
            return;
        }
        if (!pinpoint.isFieldPoseInitialized()) {
            pinpoint.setFieldPose(startXmm, startYmm, startHeadingDegrees);
            pinpoint.periodic();
        }

        double xMm = pinpoint.getXmm();
        target = CellTarget.forRobotX(blueAlliance, xMm);
        double requestedDegrees = target == null ? Double.NaN
                : target.turretAngleDegrees(xMm, pinpoint.getYmm(),
                        pinpoint.getHeadingDegrees());
        targetDegrees = TurretMath.reachableDegrees(requestedDegrees,
                turret.getAngleDegrees(), TurretConstants.minDegrees(),
                TurretConstants.maxDegrees());
        if (!aimEnabled.getAsBoolean()) {
            tracking = false;
            turret.stop();
            return;
        }
        tracking = turret.aimAtPhysicalDegrees(targetDegrees, 0.0,
                TurretConstants.odCellAimMaxPower, TurretConstants.odCellAimKp);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    public boolean isBlueAlliance() { return blueAlliance; }
    public CellTarget getTarget() { return target; }
    public double getTargetDegrees() { return targetDegrees; }
    public boolean isTracking() { return tracking; }
    public boolean isOnTarget() {
        return tracking && Double.isFinite(targetDegrees)
                && Math.abs(targetDegrees - turret.getAngleDegrees())
                <= TurretConstants.angleToleranceDegrees;
    }
}
