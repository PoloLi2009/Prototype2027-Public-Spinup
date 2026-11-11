"""Verify the production Shooter readiness gate against synthetic encoder measurements."""
from pathlib import Path
import os, subprocess, tempfile
root = Path(__file__).resolve().parents[2]
source = root/'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/shooter'
java = Path(os.environ.get('JAVA_HOME', '/Applications/Android Studio.app/Contents/jbr/Contents/Home'))/'bin'
stubs = {
'com/arcrobotics/ftclib/command/SubsystemBase.java': 'package com.arcrobotics.ftclib.command; public class SubsystemBase {public void periodic(){}}',
'com/qualcomm/robotcore/hardware/DcMotorSimple.java': 'package com.qualcomm.robotcore.hardware; public interface DcMotorSimple {enum Direction {FORWARD,REVERSE} void setDirection(Direction d);}',
'com/qualcomm/robotcore/hardware/DcMotor.java': 'package com.qualcomm.robotcore.hardware; public interface DcMotor extends DcMotorSimple {enum RunMode {RUN_WITHOUT_ENCODER} enum ZeroPowerBehavior {FLOAT,BRAKE} void setMode(RunMode m); void setZeroPowerBehavior(ZeroPowerBehavior b); void setPower(double p);}',
'com/qualcomm/robotcore/hardware/DcMotorEx.java': 'package com.qualcomm.robotcore.hardware; public interface DcMotorEx extends DcMotor {double getVelocity();}',
'com/qualcomm/robotcore/hardware/Servo.java': 'package com.qualcomm.robotcore.hardware; public interface Servo {enum Direction {FORWARD,REVERSE} void setDirection(Direction d); void setPosition(double p);}',
'com/qualcomm/robotcore/hardware/HardwareMap.java': '''package com.qualcomm.robotcore.hardware;
public class HardwareMap {public java.util.Map<String,Object> items = new java.util.HashMap<>();
 public <T>T get(Class<T> type,String name){return type.cast(items.get(name));}}''',
'com/qualcomm/robotcore/util/Range.java': 'package com.qualcomm.robotcore.util; public class Range {public static double clip(double x,double a,double b){return Math.max(a,Math.min(x,b));}}',
'ShooterThresholdCheck.java': '''import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.subsystems.shooter.*;
public class ShooterThresholdCheck {
 static class Motor implements DcMotorEx {
  double velocity,power; public double getVelocity(){return velocity;}
  public void setPower(double p){power=p;} public void setDirection(Direction d){}
  public void setMode(RunMode m){} public void setZeroPowerBehavior(ZeroPowerBehavior b){}
 }
 static void check(boolean ok,String text){if(!ok)throw new AssertionError(text);}
 public static void main(String[] args)throws Exception{
  HardwareMap map=new HardwareMap(); Motor left=new Motor(),right=new Motor();
  map.items.put(ShooterConstants.leftShooterName,left);map.items.put(ShooterConstants.rightShooterName,right);
  Shooter shooter=new Shooter(map,false);
  right.velocity=900; check(!shooter.isAtTargetRpm(),"stopped flywheel cannot feed");
  shooter.startFlywheelsAtTargetRpm(900*60.0/28);
  right.velocity=845;check(!shooter.isAtTargetRpm(),"below 6 percent band blocks");
  right.velocity=855;check(shooter.isAtTargetRpm(),"slightly below target can qualify");
  right.velocity=900;check(shooter.isAtTargetRpm(),"target qualifies");
  right.velocity=955;check(!shooter.isAtTargetRpm(),"overspeed does not qualify");
  right.velocity=-900;check(shooter.isAtTargetRpm(),"encoder magnitude semantics unchanged");
  shooter.startFlywheelsAtTargetRpm(700*60.0/28);
  right.velocity=700;check(shooter.isAtTargetRpm(),"lowered target below old 900 floor qualifies");
  right.velocity=900;check(!shooter.isAtTargetRpm(),"old speed cannot qualify after lowering target");
  right.velocity=Double.NaN;check(!shooter.isAtTargetRpm(),"invalid encoder blocks");
  right.velocity=Double.POSITIVE_INFINITY;check(!shooter.isAtTargetRpm(),"infinite encoder blocks");
  right.velocity=900;shooter.setShooterState(Shooter.ShooterState.HIGH);
  check(!shooter.isAtTargetRpm(),"open-loop preset does not bypass velocity mode gate");
  shooter.startFlywheelsAtTargetRpm(3000);shooter.periodic();
  check(shooter.getTargetRpm()==3000 && left.power>0 && left.power==right.power,"PI target retained");
  shooter.setReadyRelativeTolerance(.05);shooter.startFlywheelsAtTargetRpm(900*60.0/28);
  right.velocity=854.9;check(!shooter.isAtTargetRpm(),"Solo below -5 percent blocks");
  right.velocity=855;check(shooter.isAtTargetRpm(),"Solo -5 percent boundary qualifies");
  right.velocity=945;check(shooter.isAtTargetRpm(),"Solo +5 percent boundary qualifies");
  right.velocity=945.1;check(!shooter.isAtTargetRpm(),"Solo above +5 percent blocks");
  check(ShooterConstants.readyRelativeTolerance==.06,"Solo instance does not change other programs tolerance");
  shooter.setReadyAbsoluteToleranceTicksPerSecond(30);shooter.startFlywheelsAtTargetRpm(900*60.0/28);
  right.velocity=870;check(shooter.isAtTargetRpm(),"900-30 qualifies");
  right.velocity=930;check(shooter.isAtTargetRpm(),"900+30 qualifies");
  right.velocity=869.9;check(!shooter.isAtTargetRpm(),"below fixed band blocks");
  right.velocity=930.1;check(!shooter.isAtTargetRpm(),"above fixed band blocks");
  shooter.startFlywheelsAtTargetRpm(600*60.0/28);
  right.velocity=570;check(shooter.isAtTargetRpm(),"lower target retains absolute -30 window");
  right.velocity=630;check(shooter.isAtTargetRpm(),"lower target retains absolute +30 window");
  right.velocity=630.1;check(!shooter.isAtTargetRpm(),"outside lower target fixed window blocks");
  shooter.startFlywheelsAtTargetRpm(10);right.velocity=0;check(!shooter.isAtTargetRpm(),"zero measured speed never permits feed");
  // Real PI with a synthetic slow cycle: old code discarded every >=200ms update.
  shooter.startFlywheelsAtTargetRpm(900*60.0/28);right.velocity=800;
  shooter.periodic();double before=left.power;
  var last=Shooter.class.getDeclaredField("lastVelocityUpdateNanos");last.setAccessible(true);
  last.setLong(shooter,System.nanoTime()-300_000_000L);shooter.periodic();
  check(shooter.getIntegralPower()>0 && left.power>before,"300ms cycle still integrates bounded error");
  double accumulated=shooter.getIntegralPower();shooter.startFlywheelsAtTargetRpm(900*60.0/28);
  check(shooter.getIntegralPower()==accumulated,"same target each loop does not clear I");
  for(int i=0;i<100;i++){last.setLong(shooter,System.nanoTime()-300_000_000L);shooter.periodic();}
  check(left.power>.99&&left.power<=1,"persistent positive error can reach full output beyond old I=0.5 cap");
  double saturatedI=shooter.getIntegralPower();last.setLong(shooter,System.nanoTime()-300_000_000L);shooter.periodic();
  check(shooter.getIntegralPower()==saturatedI,"no positive windup at saturated output");
  right.velocity=1600;last.setLong(shooter,System.nanoTime()-300_000_000L);shooter.periodic();
  check(left.power<1&&shooter.getIntegralPower()<saturatedI,"overspeed lowers power and unwinds I");
  shooter.setTuningGains(0,0,.00001,1.0/6000,.5);
  shooter.startFlywheelsAtTargetRpm(900*60.0/28);right.velocity=600;shooter.periodic();
  right.velocity=650;last.setLong(shooter,System.nanoTime()-20_000_000L);shooter.periodic();
  check(shooter.getDerivativePower()<0,"nonzero filtered D resists increasing measured speed");
  shooter.startFlywheelsAtTargetRpm(1000*60.0/28);shooter.periodic();
  check(shooter.getDerivativePower()==0,"target step causes no derivative kick");
  shooter.setTuningGains(0,0,0,.0002,.4);
  shooter.startFlywheelsAtTargetRpm(900*60.0/28);shooter.periodic();
  check(Math.abs(left.power-900*60.0/28*.0002)<1e-9,"F is applied once without old target/maxRPM double-counting");
  shooter.startFlywheelsAtTargetRpm(6000);shooter.periodic();check(left.power==.4,"test-only output cap");
  shooter.clearTuningGains();right.velocity=900;shooter.startFlywheelsAtTargetRpm(900*60.0/28);shooter.periodic();
  check(Math.abs(left.power-900*60.0/28*ShooterConstants.flywheelKfPowerPerRpm)<1e-9,"clear test overrides restores shared gains");
  shooter.startFlywheelsAtTargetRpm(Double.NaN);shooter.periodic();
  check(left.power==0&&!shooter.isAtTargetRpm(),"invalid target stops");
  shooter.stop();check(!shooter.isAtTargetRpm(),"stop blocks feeding");
  System.out.println("PASS: production Shooter target-relative readiness, lowering target, invalid values, slow-loop integration, saturation, filtered D, single F and override cleanup");
 }
}'''
}
with tempfile.TemporaryDirectory(prefix='spinup-shooter-check-') as temp:
    out=Path(temp)
    for name,text in stubs.items():
        f=out/name;f.parent.mkdir(parents=True,exist_ok=True);f.write_text(text)
    subprocess.run([str(java/'javac'),'-d',temp,*map(str,out.rglob('*.java')),str(source/'Shooter.java'),str(source/'ShooterConstants.java')],check=True)
    subprocess.run([str(java/'java'),'-cp',temp,'ShooterThresholdCheck'],check=True)
