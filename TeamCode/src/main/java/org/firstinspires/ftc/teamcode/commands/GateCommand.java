package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/** Holds a requested gate state and closes it when the command ends. */
public class GateCommand extends CommandBase {
    private final Transit transit;
    private final Supplier<Transit.GateState> state;
    public GateCommand(Transit transit, Transit.GateState state) { this(transit, () -> state); }
    public GateCommand(Transit transit, Supplier<Transit.GateState> state) {
        this.transit = transit;
        this.state = Objects.requireNonNull(state);
        addRequirements(transit);
    }
    @Override public void execute() { transit.setGateState(state.get()); }
    @Override public void end(boolean interrupted) { transit.setGateState(Transit.GateState.CLOSED); }
}
