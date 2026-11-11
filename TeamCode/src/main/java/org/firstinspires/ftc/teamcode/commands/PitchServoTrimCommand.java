package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.shooter.PitchServos;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

/** Edge-triggered individual and opposed pitch-servo adjustments. */
public class PitchServoTrimCommand extends CommandBase {
    private final PitchServos servos;
    private final Gamepad gamepad;
    private boolean oldUp, oldDown, oldLeft, oldRight, oldX, oldY;

    public PitchServoTrimCommand(PitchServos servos, Gamepad gamepad) {
        this.servos = servos;
        this.gamepad = gamepad;
        addRequirements(servos);
    }

    @Override
    public void execute() {
        double step = ShooterConstants.pitchStep;
        double left = 0.0, right = 0.0;
        if (gamepad.dpad_up && !oldUp) left += step;
        if (gamepad.dpad_down && !oldDown) left -= step;
        if (gamepad.dpad_right && !oldRight) right += step;
        if (gamepad.dpad_left && !oldLeft) right -= step;
        if (gamepad.x && !oldX) { left += step; right -= step; }
        if (gamepad.y && !oldY) { left -= step; right += step; }
        oldUp = gamepad.dpad_up;
        oldDown = gamepad.dpad_down;
        oldLeft = gamepad.dpad_left;
        oldRight = gamepad.dpad_right;
        oldX = gamepad.x;
        oldY = gamepad.y;
        servos.adjust(left, right);
    }
}
