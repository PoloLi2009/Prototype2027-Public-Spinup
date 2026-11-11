package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.tuning.ShooterPidSearch;

/** Motor-only PIDF search. The test OpMode drives this lifecycle explicitly, including outputs. */
public final class ShooterPidAutoTuneCommand extends CommandBase {
    private final Shooter shooter;
    private final Intake intake;
    private final Transit transit;
    private final double targetTps, maxPower;
    private final ElapsedTime timer=new ElapsedTime();
    private ShooterPidSearch search;
    private ShooterPidSearch.Gains applied;
    public ShooterPidAutoTuneCommand(Shooter shooter, Intake intake, Transit transit, double targetTps, double maxPower) {
        this.shooter=shooter;this.intake=intake;this.transit=transit;this.targetTps=targetTps;this.maxPower=maxPower;
        addRequirements(shooter,intake,transit);
    }
    @Override public void initialize() {
        stopOutputs();timer.reset();applied=null;
        if(!Double.isFinite(ShooterConstants.rightEncoderTicksPerRevolution)
                || ShooterConstants.rightEncoderTicksPerRevolution<=0
                || !Double.isFinite(ShooterConstants.maxRpm) || ShooterConstants.maxRpm<=0
                || targetTps*60/ShooterConstants.rightEncoderTicksPerRevolution>ShooterConstants.maxRpm)
            throw new IllegalArgumentException("Invalid encoder scale or target above max RPM");
        search=new ShooterPidSearch(new ShooterPidSearch.Gains(ShooterConstants.flywheelKpPowerPerRpm,
                ShooterConstants.flywheelKiPowerPerRpmSecond,ShooterConstants.flywheelKdPowerPerRpmPerSecond,
                ShooterConstants.flywheelKfPowerPerRpm),targetTps,maxPower,0);
    }
    @Override public void execute() {
        if(search==null||search.finished()) {stopOutputs();return;}
        search.update(timer.seconds(),shooter.getFlywheelVelocityTicksPerSecond(),shooter.getCommandedFlywheelPower());
        intake.stop();intake.periodic();transit.setGateState(Transit.GateState.CLOSED);transit.periodic();
        if(search.running()) {
            if(applied!=search.gains()) {
                applied=search.gains();shooter.setTuningGains(applied.p,applied.i,applied.d,applied.f,maxPower);
            }
            shooter.startFlywheelsAtTargetRpm(search.targetTps()*60/ShooterConstants.rightEncoderTicksPerRevolution);
        } else shooter.stop();
        shooter.periodic();
    }
    public void abort(String reason) {if(search!=null)search.abort(reason);stopOutputs();}
    public ShooterPidSearch search(){return search;}
    public double elapsedSeconds(){return timer.seconds();}
    @Override public boolean isFinished(){return search!=null&&search.finished();}
    @Override public void end(boolean interrupted) {
        if(interrupted && search!=null)search.abort("Cancelled");
        try {shooter.clearTuningGains();} finally {stopOutputs();}
    }
    private void stopOutputs() {
        try {shooter.stop();shooter.periodic();}
        finally {try {intake.stop();intake.periodic();} finally {transit.setGateState(Transit.GateState.CLOSED);transit.periodic();}}
    }
}
