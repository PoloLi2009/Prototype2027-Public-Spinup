package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

/** Selects the flat-HIVE map and passes odometry and turret angles to vision fusion. */
public class VisionReadCommand extends CommandBase {
    private final LimelightVision vision;
    private final Pinpoint pinpoint;
    private final DoubleSupplier turretAngleDegrees;
    private final VisionLocalization localization;
    private final Gamepad gamepad;
    private final boolean toggleWithX;
    private boolean lastAllianceButton;
    public VisionReadCommand(LimelightVision vision, Pinpoint pinpoint, Turret turret,
                             VisionLocalization localization, Gamepad gamepad) {
        this(vision,pinpoint,turret,localization,gamepad,false);
    }
    public VisionReadCommand(LimelightVision vision, Pinpoint pinpoint, Turret turret,
                             VisionLocalization localization, Gamepad gamepad, boolean toggleWithX) {
        this(vision, pinpoint, localization, gamepad, toggleWithX, turret::getAngleDegrees);
    }
    public VisionReadCommand(LimelightVision vision, Pinpoint pinpoint,
                             VisionLocalization localization, Gamepad gamepad,
                             boolean toggleWithX, DoubleSupplier turretAngleDegrees) {
        this.vision=vision; this.pinpoint=pinpoint; this.turretAngleDegrees=turretAngleDegrees;
        this.localization=localization; this.gamepad=gamepad;
        this.toggleWithX=toggleWithX;
        addRequirements(vision,localization);
    }
    @Override public void initialize() {
        lastAllianceButton = toggleWithX ? gamepad.x : gamepad.y;
    }
    @Override public void execute() {
        boolean toggle = toggleWithX ? gamepad.x : gamepad.y;
        if (toggle && !lastAllianceButton) {
            vision.toggleAlliance();
            // Both pipelines share a field frame. Keep predicting from the last correction.
        }
        lastAllianceButton = toggle;
        if(!pinpoint.isReady()) {localization.clear(); return;}
        localization.update(System.nanoTime()/1e9,
                new Pose2d(pinpoint.getXmm()/1000.0,pinpoint.getYmm()/1000.0,
                        Rotation2d.fromDegrees(pinpoint.getHeadingDegrees())),
                turretAngleDegrees.getAsDouble(), vision.getNewFrame());
    }
}
