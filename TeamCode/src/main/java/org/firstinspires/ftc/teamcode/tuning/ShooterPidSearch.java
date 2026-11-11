package org.firstinspires.ftc.teamcode.tuning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/** Bounded coordinate search with a fresh final verification; no hardware or global config writes. */
public final class ShooterPidSearch {
    public static final double PHASE_SECONDS = 3.0, MAX_SECONDS = 210.0;
    public static final class Gains {
        public final double p, i, d, f;
        public Gains(double p, double i, double d, double f) {
            for (double x : new double[]{p,i,d,f})
                if (!Double.isFinite(x) || x < 0) throw new IllegalArgumentException("Invalid PIDF");
            if (f <= 0) throw new IllegalArgumentException("F must be positive");
            this.p=p; this.i=i; this.d=d; this.f=f;
        }
        public String toString() { return String.format(Locale.US,"P=%.8f I=%.8f D=%.8f F=%.8f",p,i,d,f); }
    }
    public static final class Result {
        public final Gains gains;
        public final double score, meanRelativeError, worstSteadyError, worstRipple, saturationFraction, peakTps;
        public final boolean qualified;
        Result(Gains gains, double score, double error, double worstError, double ripple,
               double saturated, double peak, boolean qualified) {
            this.gains=gains; this.score=score; meanRelativeError=error; worstSteadyError=worstError;
            worstRipple=ripple; saturationFraction=saturated; peakTps=peak; this.qualified=qualified;
        }
    }
    private enum State { REST, RUN, DONE, ABORTED }
    private State state=State.REST;
    private final Gains initial;
    private Gains current, bestGains;
    private Result best, recommendation;
    private final double target, powerLimit;
    private final List<Result> results=new ArrayList<>();
    private final List<Gains> candidates=new ArrayList<>();
    private double started, stateStarted, lastTime, phaseStarted, zeroSince=Double.NaN;
    private double sumError, duration, saturatedTime, peak;
    private final double[] steadyTime=new double[3], steadyError=new double[3], steadySpeed=new double[3], steadySquared=new double[3], steadyWorst=new double[3];
    private int stage=-1, candidateIndex, phase, trial=1;
    private boolean verify;
    private String message="Resting before baseline";

    public ShooterPidSearch(Gains initial, double targetTps, double powerLimit, double now) {
        if (!Double.isFinite(targetTps) || targetTps < 200 || targetTps > 2000
                || !Double.isFinite(powerLimit) || powerLimit < .1 || powerLimit > 1 || !Double.isFinite(now))
            throw new IllegalArgumentException("Target 200..2000 TPS; power limit 0.1..1");
        this.initial=initial; current=bestGains=initial; target=targetTps; this.powerLimit=powerLimit;
        started=stateStarted=lastTime=now;
    }
    public void update(double now, double measured, double output) {
        if (finished()) return;
        if (!Double.isFinite(now) || now<lastTime || !Double.isFinite(measured) || measured<0
                || !Double.isFinite(output)) { abort("Invalid clock/encoder/output"); return; }
        double dt=now-lastTime; lastTime=now;
        if (now-started>=MAX_SECONDS) { abort("210 second total limit"); return; }
        if (measured>target*1.4) { abort("Overspeed >140% of highest test target"); return; }
        if (state==State.REST) {
            if (now-stateStarted>=.5 && (measured<target*.1 || now-stateStarted>=4)) {
                state=State.RUN; phase=0; phaseStarted=now; resetStats();
                message=verify?"Verifying selected PIDF":"Testing candidate "+trial+" / 13";
            }
            return;
        }
        if (dt>.25) { abort("Control loop >250ms; fix loop latency before tuning PID"); return; }
        if (measured<20 && output>=.35) {
            if (!Double.isFinite(zeroSince)) zeroSince=now;
            if (now-zeroSince>=1.5) { abort("No encoder response under power"); return; }
        } else zeroSince=Double.NaN;
        double phaseAge=now-phaseStarted;
        double relative=Math.abs(measured-targetTps())/targetTps();
        sumError+=relative*dt; duration+=dt; peak=Math.max(peak,measured);
        if (output>=powerLimit-.01) saturatedTime+=dt;
        if (phaseAge>=2.0) {
            steadyTime[phase]+=dt; steadyError[phase]+=relative*dt;
            steadySpeed[phase]+=measured*dt; steadySquared[phase]+=measured*measured*dt;
            steadyWorst[phase]=Math.max(steadyWorst[phase],relative);
        }
        if (phaseAge>=PHASE_SECONDS) {
            phase++;
            if (phase<3) phaseStarted=now;
            else completeTrial(now);
        }
    }
    private void resetStats() {
        sumError=duration=saturatedTime=peak=0;zeroSince=Double.NaN;
        for(double[] x:new double[][]{steadyTime,steadyError,steadySpeed,steadySquared,steadyWorst}) java.util.Arrays.fill(x,0);
    }
    private void completeTrial(double now) {
        double meanError=0,ripple=0,worst=0; boolean valid=true;
        for(int n=0;n<3;n++) {
            double time=steadyTime[n];
            if(time<.5) { abort("Not enough steady samples"); return; }
            double mean=steadySpeed[n]/time;
            double sd=Math.sqrt(Math.max(0,steadySquared[n]/time-mean*mean));
            double ref=n==1?target:target*.8;
            double e=steadyError[n]/time;
            meanError+=e/3; ripple=Math.max(ripple,sd/ref); worst=Math.max(worst,steadyWorst[n]);
            valid &= e<=.06 && sd/ref<=.03 && steadyWorst[n]<=.12;
        }
        double saturated=saturatedTime/Math.max(duration,.001);
        double score=4*meanError+2*ripple+sumError/Math.max(duration,.001)+.1*saturated;
        Result result=new Result(current,score,meanError,worst,ripple,saturated,peak,valid && saturated<.8);
        results.add(result);
        if (verify) {
            if (result.qualified) { recommendation=result; message="DONE: verified candidate; not applied to match settings"; }
            else message="DONE: selected candidate failed repeat validation; no recommendation";
            state=State.DONE;return;
        }
        if(best==null || result.score<best.score) { best=result;bestGains=result.gains; }
        if(stage==-1) { stage=0; prepareStage(); }
        else if(++candidateIndex>=candidates.size()) { stage++; if(stage<4)prepareStage(); }
        if(stage>=4) { verify=true;current=bestGains; }
        else current=candidates.get(candidateIndex);
        trial++;state=State.REST;stateStarted=now;message="Coasting before next trial";
    }
    private void prepareStage() {
        candidates.clear();candidateIndex=0;
        // Coordinate descent: F, P, I, then D. Nonzero D is always tested.
        double[] values=stage==0?new double[]{initial.f*.75,initial.f*1.25,initial.f*1.75}
                :stage==1?new double[]{Math.max(initial.p,.0001)*.5,Math.max(initial.p,.0001)*1.5,Math.max(initial.p,.0001)*4}
                :stage==2?new double[]{0,Math.max(initial.i,.00015)*.5,Math.max(initial.i,.00015)*2}
                :new double[]{0,Math.max(initial.d,.000002),Math.max(initial.d,.000002)*4};
        Gains anchor=bestGains;
        for(double value:values)candidates.add(new Gains(stage==1?value:anchor.p,stage==2?value:anchor.i,
                stage==3?value:anchor.d,stage==0?value:anchor.f));
    }
    public void abort(String reason) { if(!finished()){state=State.ABORTED;message="ABORTED: "+reason;recommendation=null;} }
    public boolean finished(){return state==State.DONE||state==State.ABORTED;}
    public boolean running(){return state==State.RUN;}
    public double targetTps(){return state==State.RUN?(phase==1?target:target*.8):0;}
    public Gains gains(){return current;}
    public int trialNumber(){return trial;}
    public String message(){return message;}
    public Result recommendation(){return recommendation;}
    public List<Result> results(){return Collections.unmodifiableList(results);}
}
