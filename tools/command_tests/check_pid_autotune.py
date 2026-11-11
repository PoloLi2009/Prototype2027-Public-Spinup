"""Exercise production PIDF search and command with deterministic clock/encoder boundaries."""
from pathlib import Path
import ast, subprocess, tempfile
here=Path(__file__).resolve().parent;root=here.parents[1]
tree=ast.parse((here/'check_shooter.py').read_text())
stubs=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='stubs' for t in n.targets))
stubs.pop('ShooterThresholdCheck.java')
stubs.update({
'com/arcrobotics/ftclib/command/CommandBase.java':'package com.arcrobotics.ftclib.command; public class CommandBase {public void addRequirements(Object... o){}public void initialize(){}public void execute(){}public boolean isFinished(){return false;}public void end(boolean b){}}',
'com/qualcomm/robotcore/util/ElapsedTime.java':'package com.qualcomm.robotcore.util; public class ElapsedTime {public static double now;private double start=now;public void reset(){start=now;}public double seconds(){return now-start;}}',
'org/firstinspires/ftc/teamcode/subsystems/intake/Intake.java':'package org.firstinspires.ftc.teamcode.subsystems.intake; public class Intake {public double power=1;public void stop(){power=0;}public void periodic(){}}',
'org/firstinspires/ftc/teamcode/subsystems/transit/Transit.java':'package org.firstinspires.ftc.teamcode.subsystems.transit; public class Transit {public enum GateState {OPEN,CLOSED} public GateState gate=GateState.OPEN; public void setGateState(GateState s){gate=s;}public void periodic(){}}',
'AutoTuneCheck.java':'''import com.qualcomm.robotcore.hardware.*;import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.shooter.*;import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;import org.firstinspires.ftc.teamcode.tuning.ShooterPidSearch;
import org.firstinspires.ftc.teamcode.commands.ShooterPidAutoTuneCommand;
public class AutoTuneCheck {static int checks;static void check(boolean b,String s){checks++;if(!b)throw new AssertionError(s);}
static class M implements DcMotorEx {double velocity,power;public double getVelocity(){return velocity;}public void setPower(double p){power=p;}public void setDirection(Direction d){}public void setMode(RunMode m){}public void setZeroPowerBehavior(ZeroPowerBehavior b){}}
static ShooterPidSearch model(){return new ShooterPidSearch(new ShooterPidSearch.Gains(.0002,.0003,0,1.0/6000),900,.85,0);}
public static void main(String[] args){
var m=model();double t=0;while(!m.finished()){t+=.02;m.update(t,m.targetTps(),m.running()?.4:0);check(t<211,"search bounded");}
check(m.results().size()==14&&m.recommendation()!=null,"baseline plus twelve candidates plus independent verification");
check(m.results().stream().anyMatch(r->r.gains.d>0),"nonzero D actually tested");
check(m.results().stream().anyMatch(r->r.gains.f!=1.0/6000),"F actually tested");
check(m.results().stream().anyMatch(r->r.gains.p!=.0002)&&m.results().stream().anyMatch(r->r.gains.i!=.0003),"P and I actually tested");
m=model();t=0;while(!m.finished()){t+=.02;m.update(t,m.trialNumber()==14?m.targetTps()*.7:m.targetTps(),m.running()?.4:0);}
check(m.recommendation()==null,"failed independent repeat cannot recommend");
m=model();m.update(.6,0,0);m.update(.9,0,.5);check(m.finished()&&m.recommendation()==null,"slow loop aborts");
m=model();m.update(.1,1500,0);check(m.finished(),"overspeed aborts");
m=model();m.update(.1,Double.NaN,0);check(m.finished(),"invalid encoder aborts");
m=model();m.update(210,0,0);check(m.finished(),"hard total deadline");
m=model();for(t=0;!m.finished()&&t<10;t+=.02)m.update(t,0,m.running()?.8:0);check(m.finished()&&m.message().contains("No encoder"),"missing encoder response stops");
double oldP=ShooterConstants.flywheelKpPowerPerRpm,oldI=ShooterConstants.flywheelKiPowerPerRpmSecond,oldD=ShooterConstants.flywheelKdPowerPerRpmPerSecond,oldF=ShooterConstants.flywheelKfPowerPerRpm;
HardwareMap h=new HardwareMap();M l=new M(),r=new M();h.items.put(ShooterConstants.leftShooterName,l);h.items.put(ShooterConstants.rightShooterName,r);
Shooter shooter=new Shooter(h,false);Intake intake=new Intake();Transit gate=new Transit();ElapsedTime.now=0;
var command=new ShooterPidAutoTuneCommand(shooter,intake,gate,900,.85);command.initialize();
check(l.power==0&&r.power==0&&intake.power==0&&gate.gate==Transit.GateState.CLOSED,"initialize stops all test outputs");
while(!command.isFinished()) {
ElapsedTime.now+=.02;r.velocity=command.search().targetTps();command.execute();
check(l.power>=0&&l.power<=.85&&r.power==l.power,"same competition Shooter honors test-only cap");
check(intake.power==0&&gate.gate==Transit.GateState.CLOSED,"no intake or feeding during search");}
command.end(false);check(command.search().recommendation()!=null&&l.power==0&&r.power==0,"completed search stops before publishing recommendation");
command.initialize();r.velocity=0;ElapsedTime.now+=.6;command.execute();check(l.power>0,"restart runs");command.abort("B");command.end(true);
check(l.power==0&&intake.power==0&&gate.gate==Transit.GateState.CLOSED&&command.search().recommendation()==null,"B immediate stop, no stale recommendation");
check(ShooterConstants.flywheelKpPowerPerRpm==oldP&&ShooterConstants.flywheelKiPowerPerRpmSecond==oldI&&ShooterConstants.flywheelKdPowerPerRpmPerSecond==oldD&&ShooterConstants.flywheelKfPowerPerRpm==oldF,"shared competition gains unchanged");
System.out.println("PASS: "+checks+" PIDF search/command checks, all four gains, repeat-validation rejection, safety aborts and output cleanup");}}
'''})
source=root/'TeamCode/src/main/java/org/firstinspires/ftc/teamcode';java=Path('/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin')
with tempfile.TemporaryDirectory(prefix='spinup-autotune-check-') as temp:
    out=Path(temp)
    for name,code in stubs.items():
        p=out/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(code)
    actual=[source/p for p in ['subsystems/shooter/Shooter.java','subsystems/shooter/ShooterConstants.java','tuning/ShooterPidSearch.java','commands/ShooterPidAutoTuneCommand.java']]
    subprocess.run([str(java/'javac'),'-d',temp,*map(str,out.rglob('*.java')),*map(str,actual)],check=True)
    subprocess.run([str(java/'java'),'-cp',temp,'AutoTuneCheck'],check=True)
