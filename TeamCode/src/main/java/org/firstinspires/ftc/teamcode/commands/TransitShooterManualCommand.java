package org.firstinspires.ftc.teamcode.commands;

import java.util.Objects;
import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
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
    private final BooleanSupplier increaseRpm;
    private final BooleanSupplier decreaseRpm;
    private final DoubleSupplier spinRequest;
    private final DoubleSupplier openGateRequest;
    private final BooleanSupplier feedRequested;
    private final BooleanSupplier reverseRequested;
    private final BooleanSupplier collectRequested;
    private final FlywheelRpmCommand spin;
    private final RunIntakeCommand motor;
    private final GateCommand gate;
    private double targetRpm = 3000.0;
    private boolean oldUp, oldDown;

    public TransitShooterManualCommand(Intake intake, Shooter shooter, Transit transit,
                                       Gamepad gamepad) {
        this(intake, shooter, transit, () -> gamepad.dpad_up, () -> gamepad.dpad_down,
                () -> gamepad.right_trigger, () -> gamepad.left_trigger, () -> gamepad.a, () -> gamepad.y,
                () -> gamepad.x);
    }

    /** Live control inputs; axes retain the original command signs and deadbands. */
    public TransitShooterManualCommand(Intake intake, Shooter shooter, Transit transit,
                                       BooleanSupplier increaseRpm, BooleanSupplier decreaseRpm,
                                               DoubleSupplier spinRequest, DoubleSupplier openGateRequest,
                                               BooleanSupplier feedRequested, BooleanSupplier reverseRequested,
                                               BooleanSupplier collectRequested) {
        this.intake = intake;
        this.shooter = shooter;
        this.transit = transit;
        this.increaseRpm = Objects.requireNonNull(increaseRpm);
        this.decreaseRpm = Objects.requireNonNull(decreaseRpm);
        this.spinRequest = Objects.requireNonNull(spinRequest);
        this.openGateRequest = Objects.requireNonNull(openGateRequest);
        this.feedRequested = Objects.requireNonNull(feedRequested);
        this.reverseRequested = Objects.requireNonNull(reverseRequested);
        this.collectRequested = Objects.requireNonNull(collectRequested);
        spin = new FlywheelRpmCommand(shooter, () -> targetRpm);
        motor = new RunIntakeCommand(intake, () -> feedRequested.getAsBoolean() ? Intake.IntakeState.FEED
                : reverseRequested.getAsBoolean() ? Intake.IntakeState.REVERSED
                : collectRequested.getAsBoolean() ? Intake.IntakeState.FORWARD : Intake.IntakeState.STOP);
        gate = new GateCommand(transit, () -> openGateRequest.getAsDouble() >= 0.5
                ? Transit.GateState.OPEN : Transit.GateState.CLOSED);
        addRequirements(intake, shooter, transit);
    }

    @Override
    public void execute() {
        if (increaseRpm.getAsBoolean() && !oldUp)
            targetRpm = Math.min(ShooterConstants.maxRpm, targetRpm + RPM_STEP);
        if (decreaseRpm.getAsBoolean() && !oldDown) targetRpm = Math.max(0, targetRpm - RPM_STEP);
        oldUp = increaseRpm.getAsBoolean();
        oldDown = decreaseRpm.getAsBoolean();
        if (spinRequest.getAsDouble() >= 0.5) spin.execute();
        else shooter.stop();
        motor.execute();
        gate.execute();
    }

    public double getTargetRpm() { return targetRpm; }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        shooter.stop();
        transit.setGateState(Transit.GateState.CLOSED);
    }
}
