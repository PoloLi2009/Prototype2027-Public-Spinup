package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

/** Direct left-stick power for encoder direction checks. */
public class TurretEncoderTestCommand extends CommandBase {
    private final Turret turret;
    private final DoubleSupplier power;

    public TurretEncoderTestCommand(Turret turret, Gamepad gamepad) {
        this(turret, () -> gamepad.left_stick_x);
    }

    /** Live control inputs; axes retain the original command signs and deadbands. */
    public TurretEncoderTestCommand(Turret turret, DoubleSupplier power) {
        this.turret = turret;
        this.power = Objects.requireNonNull(power);
        addRequirements(turret);
    }

    @Override
    public void execute() { turret.setPower(power.getAsDouble()); }

    @Override
    public void end(boolean interrupted) { turret.stop(); }
}
