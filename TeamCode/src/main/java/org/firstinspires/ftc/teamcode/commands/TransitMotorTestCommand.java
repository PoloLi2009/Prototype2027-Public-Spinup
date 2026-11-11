package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.BooleanSupplier;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

/** X forward, Y reverse, either both or neither stopped. */
public class TransitMotorTestCommand extends CommandBase {
    private final Intake intake;
    private final BooleanSupplier forwardRequested;
    private final BooleanSupplier reverseRequested;

    public TransitMotorTestCommand(Intake intake, Gamepad gamepad) {
        this(intake, () -> gamepad.x, () -> gamepad.y);
    }

    /** Live control inputs; axes retain the original command signs and deadbands. */
    public TransitMotorTestCommand(Intake intake, BooleanSupplier forwardRequested,
            BooleanSupplier reverseRequested) {
        this.intake = intake;
        this.forwardRequested = Objects.requireNonNull(forwardRequested);
        this.reverseRequested = Objects.requireNonNull(reverseRequested);
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (forwardRequested.getAsBoolean() && !reverseRequested.getAsBoolean()) intake.setIntakeState(Intake.IntakeState.FORWARD);
        else if (reverseRequested.getAsBoolean() && !forwardRequested.getAsBoolean()) intake.setIntakeState(Intake.IntakeState.REVERSED);
        else intake.stop();
    }

    @Override
    public void end(boolean interrupted) { intake.stop(); }
}
