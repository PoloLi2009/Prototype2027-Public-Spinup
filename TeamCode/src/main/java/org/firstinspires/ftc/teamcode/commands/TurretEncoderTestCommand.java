package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

/** Direct left-stick power for encoder direction checks. */
public class TurretEncoderTestCommand extends CommandBase {
    private final Turret turret;
    private final Gamepad gamepad;

    public TurretEncoderTestCommand(Turret turret, Gamepad gamepad) {
        this.turret = turret;
        this.gamepad = gamepad;
        addRequirements(turret);
    }

    @Override
    public void execute() { turret.setPower(gamepad.left_stick_x); }

    @Override
    public void end(boolean interrupted) { turret.stop(); }
}
