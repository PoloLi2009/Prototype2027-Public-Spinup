package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.commands.ShooterPidAutoTuneCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.tuning.ShooterPidSearch;

/** Uses the competition Shooter and Dashboard telemetry, as Overtune ShooterPID does. */
@Config
@TeleOp(name="Shooter PID AutoTune",group="Test")
public class ShooterPidAutoTune extends LinearOpMode {
    public static double targetTps=900.0;
    public static double maxPower=.85;
    @Override public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        Shooter shooter=null;Intake intake=null;Transit transit=null;
        ShooterPidAutoTuneCommand command=null;
        BufferedWriter csv=null;
        File runFolder=null;
        String status="START, then release/press A to run";
        try {
            shooter=new Shooter(hardwareMap,false); // No pitch/drive/turret commands in this test.
            intake=new Intake(hardwareMap);transit=new Transit(hardwareMap);
            shooter.stop();shooter.periodic();intake.stop();intake.periodic();transit.periodic();
            telemetry.addLine("Remove balls. Flywheels will run automatically; no feeding.");
            telemetry.addLine("START then A: ~2-3 min bounded PIDF search; B or STOP aborts.");
            telemetry.addLine("Dashboard config: ShooterPidAutoTune targetTps/maxPower (read on A).");
            telemetry.update();waitForStart();
            if(isStopRequested())return;
            boolean lastA=gamepad1.a;boolean active=false;int lastTrial=0;
            long lastTelemetry=0;
            while(opModeIsActive()) {
                if(gamepad1.b && active) {
                    command.abort("B pressed");command.end(true);active=false;status=command.search().message();
                    saveResults(runFolder,command.search());
                    if(csv!=null){csv.close();csv=null;}
                }
                if(gamepad1.a&&!lastA&&!gamepad1.b&&!active) {
                    command=new ShooterPidAutoTuneCommand(shooter,intake,transit,targetTps,maxPower);
                    command.initialize();
                    File parent=hardwareMap.appContext.getExternalFilesDir(null);
                    if(parent==null)throw new IOException("Robot external files directory unavailable");
                    runFolder=new File(parent,"shooter-pid-autotune/run-"+System.currentTimeMillis());
                    if(!runFolder.mkdirs())throw new IOException("Cannot create log folder");
                    csv=new BufferedWriter(new FileWriter(new File(runFolder,"samples.csv")));
                    csv.write("seconds,trial,target_tps,actual_tps,output,F,P,I,D,loop_ms\n");
                    active=true;lastTrial=0;
                }
                lastA=gamepad1.a;
                if(active) {
                    // Log measured response against the target that produced it, before transitioning.
                    double target=command.search().targetTps();
                    csv.write(String.format(Locale.US,"%.4f,%d,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f%n",
                            command.elapsedSeconds(),command.search().trialNumber(),target,
                            shooter.getFlywheelVelocityTicksPerSecond(),shooter.getCommandedFlywheelPower(),
                            shooter.getFeedforwardPower(),shooter.getProportionalPower(),shooter.getIntegralPower(),
                            shooter.getDerivativePower(),shooter.getVelocityUpdateSeconds()*1000));
                    command.execute();status=command.search().message();
                    if(lastTrial!=command.search().trialNumber()){csv.flush();lastTrial=command.search().trialNumber();}
                    if(command.isFinished()) {
                        command.end(false);active=false;csv.close();csv=null;
                        saveResults(runFolder,command.search());
                    }
                }
                long now=System.nanoTime();
                if(now-lastTelemetry>100_000_000L) {
                    lastTelemetry=now;TelemetryPacket packet=new TelemetryPacket();
                    packet.put("status",status);packet.put("targetTps",active?command.search().targetTps():0);
                    packet.put("actualTps",shooter.getFlywheelVelocityTicksPerSecond());
                    packet.put("output",shooter.getCommandedFlywheelPower());
                    packet.put("P",shooter.getProportionalPower());packet.put("I",shooter.getIntegralPower());
                    packet.put("D",shooter.getDerivativePower());packet.put("F",shooter.getFeedforwardPower());
                    if(command!=null && command.search()!=null)packet.put("gains",command.search().gains().toString());
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                    telemetry.addData("Status",status);
                    telemetry.addData("Actual TPS / power","%.1f / %.3f",shooter.getFlywheelVelocityTicksPerSecond(),shooter.getCommandedFlywheelPower());
                    if(command!=null&&command.search()!=null) {
                        telemetry.addData("Trial / seconds","%d / %.1f",command.search().trialNumber(),command.elapsedSeconds());
                        telemetry.addData("Candidate",command.search().gains());
                        ShooterPidSearch.Result result=command.search().recommendation();
                        if(result!=null){telemetry.addData("RECOMMENDED (RPM units)",result.gains);telemetry.addLine("Saved only; match gains unchanged. Verify under ball load before applying.");}
                    }
                    telemetry.addData("Logs",runFolder==null?"Not started":runFolder.getAbsolutePath());
                    telemetry.addLine("A: new search after stopping. B: abort. Dashboard: targetTps/actualTps/output/P/I/D/F.");
                    telemetry.update();
                }
                sleep(20);
            }
        } catch(RuntimeException|IOException e) {
            status="Stopped: "+e.getClass().getSimpleName()+": "+e.getMessage();
            if(command!=null)command.abort(status);
            telemetry.addData("AutoTune",status);telemetry.update();
        } finally {
            try {if(command!=null)command.end(true);}
            finally {
                try {if(shooter!=null){shooter.clearTuningGains();shooter.periodic();}}
                finally {
                    try {if(intake!=null){intake.stop();intake.periodic();}}
                    finally {
                        try {if(transit!=null){transit.setGateState(Transit.GateState.CLOSED);transit.periodic();}}
                        finally {
                            try {if(csv!=null)csv.close();if(runFolder!=null&&command!=null&&command.search()!=null)saveResults(runFolder,command.search());}
                            catch(IOException e){telemetry.addData("Log save failed",e.getMessage());telemetry.update();}
                            finally {CommandScheduler.getInstance().reset();}
                        }
                    }
                }
            }
        }
    }
    private static void saveResults(File folder,ShooterPidSearch search)throws IOException {
        if(folder==null)return;
        try(BufferedWriter out=new BufferedWriter(new FileWriter(new File(folder,"results.csv")))) {
            out.write("trial,P_power_per_RPM,I_power_per_RPM_s,D_power_per_RPM_per_s,F_power_per_RPM,score,mean_relative_error,worst_steady_error,ripple,saturated_fraction,peak_tps,qualified\n");
            int n=0;for(ShooterPidSearch.Result r:search.results())out.write(String.format(Locale.US,
                    "%d,%.10f,%.10f,%.10f,%.10f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%b%n",
                    ++n,r.gains.p,r.gains.i,r.gains.d,r.gains.f,r.score,r.meanRelativeError,r.worstSteadyError,r.worstRipple,r.saturationFraction,r.peakTps,r.qualified));
        }
        try(BufferedWriter out=new BufferedWriter(new FileWriter(new File(folder,"recommendation.txt")))) {
            out.write(search.message()+"\nUnits: P power/RPM; I power/(RPM*s); D power/(RPM/s); F power/RPM\n");
            ShooterPidSearch.Result result=search.recommendation();
            if(result==null)out.write("No verified recommendation. Review logs; do not copy the lowest score blindly.\n");
            else {
                out.write(result.gains+"\nTested without ball load; not applied to match settings.\n");
                out.write(String.format(Locale.US,
                        "ShooterConstants.flywheelKpPowerPerRpm = %.10f;\nShooterConstants.flywheelKiPowerPerRpmSecond = %.10f;\nShooterConstants.flywheelKdPowerPerRpmPerSecond = %.10f;\nShooterConstants.flywheelKfPowerPerRpm = %.10f;\n",
                        result.gains.p,result.gains.i,result.gains.d,result.gains.f));
            }
        }
    }
}
