package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

/** X forward, Y reverse, either both or neither stopped. */
public class TransitMotorTestCommand extends CommandBase {
    private final Intake intake;
    private final Gamepad gamepad;

    public TransitMotorTestCommand(Intake intake, Gamepad gamepad) {
        this.intake = intake;
        this.gamepad = gamepad;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (gamepad.x && !gamepad.y) intake.setIntakeState(Intake.IntakeState.FORWARD);
        else if (gamepad.y && !gamepad.x) intake.setIntakeState(Intake.IntakeState.REVERSED);
        else intake.stop();
    }

    @Override
    public void end(boolean interrupted) { intake.stop(); }
}
