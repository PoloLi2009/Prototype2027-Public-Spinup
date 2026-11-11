package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

/** Edge-triggered direct gate-servo position trim. */
public class LimitServoTrimCommand extends CommandBase {
    private static final double STEP = 0.01;
    private final Transit transit;
    private final BooleanSupplier increaseRequested;
    private final BooleanSupplier decreaseRequested;
    private boolean oldUp, oldDown;
    private double position = TransitConstants.gateClosedPosition;

    public LimitServoTrimCommand(Transit transit, Gamepad gamepad) {
        this(transit, () -> gamepad.dpad_up, () -> gamepad.dpad_down);
    }

    /** Live control inputs; axes retain the original command signs and deadbands. */
    public LimitServoTrimCommand(Transit transit, BooleanSupplier increaseRequested,
            BooleanSupplier decreaseRequested) {
        this.transit = transit;
        this.increaseRequested = Objects.requireNonNull(increaseRequested);
        this.decreaseRequested = Objects.requireNonNull(decreaseRequested);
        addRequirements(transit);
        transit.setManualPosition(position);
    }

    @Override
    public void execute() {
        if (increaseRequested.getAsBoolean() && !oldUp) position += STEP;
        if (decreaseRequested.getAsBoolean() && !oldDown) position -= STEP;
        oldUp = increaseRequested.getAsBoolean();
        oldDown = decreaseRequested.getAsBoolean();
        transit.setManualPosition(position);
        position = transit.getPosition();
    }

    public double getPosition() { return position; }
}
