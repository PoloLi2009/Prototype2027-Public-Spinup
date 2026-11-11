package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.shooter.PitchServos;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

/** Edge-triggered individual and opposed pitch-servo adjustments. */
public class PitchServoTrimCommand extends CommandBase {
    private final PitchServos servos;
    private final BooleanSupplier increaseLeft;
    private final BooleanSupplier decreaseLeft;
    private final BooleanSupplier increaseRight;
    private final BooleanSupplier decreaseRight;
    private final BooleanSupplier increaseOpposed;
    private final BooleanSupplier decreaseOpposed;
    private boolean oldUp, oldDown, oldLeft, oldRight, oldX, oldY;

    public PitchServoTrimCommand(PitchServos servos, Gamepad gamepad) {
        this(servos, () -> gamepad.dpad_up, () -> gamepad.dpad_down, () -> gamepad.dpad_right,
                () -> gamepad.dpad_left, () -> gamepad.x, () -> gamepad.y);
    }

    /** Live control inputs; axes retain the original command signs and deadbands. */
    public PitchServoTrimCommand(PitchServos servos, BooleanSupplier increaseLeft, BooleanSupplier decreaseLeft,
            BooleanSupplier increaseRight, BooleanSupplier decreaseRight, BooleanSupplier increaseOpposed,
            BooleanSupplier decreaseOpposed) {
        this.servos = servos;
        this.increaseLeft = Objects.requireNonNull(increaseLeft);
        this.decreaseLeft = Objects.requireNonNull(decreaseLeft);
        this.increaseRight = Objects.requireNonNull(increaseRight);
        this.decreaseRight = Objects.requireNonNull(decreaseRight);
        this.increaseOpposed = Objects.requireNonNull(increaseOpposed);
        this.decreaseOpposed = Objects.requireNonNull(decreaseOpposed);
        addRequirements(servos);
    }

    @Override
    public void execute() {
        double step = ShooterConstants.pitchStep;
        double left = 0.0, right = 0.0;
        if (increaseLeft.getAsBoolean() && !oldUp) left += step;
        if (decreaseLeft.getAsBoolean() && !oldDown) left -= step;
        if (increaseRight.getAsBoolean() && !oldRight) right += step;
        if (decreaseRight.getAsBoolean() && !oldLeft) right -= step;
        if (increaseOpposed.getAsBoolean() && !oldX) { left += step; right -= step; }
        if (decreaseOpposed.getAsBoolean() && !oldY) { left -= step; right += step; }
        oldUp = increaseLeft.getAsBoolean();
        oldDown = decreaseLeft.getAsBoolean();
        oldLeft = decreaseRight.getAsBoolean();
        oldRight = increaseRight.getAsBoolean();
        oldX = increaseOpposed.getAsBoolean();
        oldY = decreaseOpposed.getAsBoolean();
        servos.adjust(left, right);
    }
}
