package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

/** Edge-triggered direct gate-servo position trim. */
public class LimitServoTrimCommand extends CommandBase {
    private static final double STEP = 0.01;
    private final Transit transit;
    private final Gamepad gamepad;
    private boolean oldUp, oldDown;
    private double position = TransitConstants.gateClosedPosition;

    public LimitServoTrimCommand(Transit transit, Gamepad gamepad) {
        this.transit = transit;
        this.gamepad = gamepad;
        addRequirements(transit);
        transit.setManualPosition(position);
    }

    @Override
    public void execute() {
        if (gamepad.dpad_up && !oldUp) position += STEP;
        if (gamepad.dpad_down && !oldDown) position -= STEP;
        oldUp = gamepad.dpad_up;
        oldDown = gamepad.dpad_down;
        transit.setManualPosition(position);
        position = transit.getPosition();
    }

    public double getPosition() { return position; }
}
