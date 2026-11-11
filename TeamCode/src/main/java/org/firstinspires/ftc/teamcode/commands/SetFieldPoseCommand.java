package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;

/** Set an explicit field origin in millimetres/degrees; compose before movement. */
public class SetFieldPoseCommand extends InstantCommand {
    public SetFieldPoseCommand(Pinpoint pinpoint, double xMm, double yMm, double headingDegrees) {
        super(() -> pinpoint.setFieldPose(xMm, yMm, headingDegrees), pinpoint);
    }
    public SetFieldPoseCommand(Pinpoint pinpoint) { super(pinpoint::resetFieldPose, pinpoint); }
}
