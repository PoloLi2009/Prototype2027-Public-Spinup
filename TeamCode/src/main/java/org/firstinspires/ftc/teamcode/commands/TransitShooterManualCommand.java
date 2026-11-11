package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/** Independent manual shooter, feeder and gate test controls. */
public class TransitShooterManualCommand extends CommandBase {
    private static final double RPM_STEP = 100.0;
    private final Intake intake;
    private final Shooter shooter;
    private final Transit transit;
    private final Gamepad gamepad;
    private double targetRpm = 3000.0;
    private boolean oldUp, oldDown;

    public TransitShooterManualCommand(Intake intake, Shooter shooter, Transit transit,
                                       Gamepad gamepad) {
        this.intake = intake;
        this.shooter = shooter;
        this.transit = transit;
        this.gamepad = gamepad;
        addRequirements(intake, shooter, transit);
    }

    @Override
    public void execute() {
        if (gamepad.dpad_up && !oldUp)
            targetRpm = Math.min(ShooterConstants.maxRpm, targetRpm + RPM_STEP);
        if (gamepad.dpad_down && !oldDown) targetRpm = Math.max(0, targetRpm - RPM_STEP);
        oldUp = gamepad.dpad_up;
        oldDown = gamepad.dpad_down;
        if (gamepad.right_trigger >= 0.5) shooter.startFlywheelsAtTargetRpm(targetRpm);
        else shooter.stop();
        if (gamepad.a) intake.setIntakeState(Intake.IntakeState.FEED);
        else if (gamepad.y) intake.setIntakeState(Intake.IntakeState.REVERSED);
        else if (gamepad.x) intake.setIntakeState(Intake.IntakeState.FORWARD);
        else intake.stop();
        transit.setGateState(gamepad.left_trigger >= 0.5
                ? Transit.GateState.OPEN : Transit.GateState.CLOSED);
    }

    public double getTargetRpm() { return targetRpm; }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        shooter.stop();
        transit.setGateState(Transit.GateState.CLOSED);
    }
}
