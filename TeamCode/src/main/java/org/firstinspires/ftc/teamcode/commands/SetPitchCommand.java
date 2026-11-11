package org.firstinspires.ftc.teamcode.commands;

import java.util.function.DoubleSupplier;
import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/** One-shot pitch selection; leaves that position selected after completion. */
public class SetPitchCommand extends InstantCommand {
    public SetPitchCommand(Shooter shooter, Shooter.PitchState state) {
        super(() -> shooter.setPitchState(state), shooter);
    }
    /** Apply a relative position adjustment using the same clamping as manual trim. */
    public SetPitchCommand(Shooter shooter, double adjustment) {
        this(shooter, () -> adjustment);
    }
    public SetPitchCommand(Shooter shooter, DoubleSupplier adjustment) {
        super(() -> shooter.adjustPitch(adjustment.getAsDouble()), shooter);
    }
}
