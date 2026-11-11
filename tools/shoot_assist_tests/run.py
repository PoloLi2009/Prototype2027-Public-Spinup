#!/usr/bin/env python3
"""Run the ported Overtune ShootAssist/AutoShoot chain against fake Spinup hardware.

Real sources: ShootAssistCommand, AutoShootCommand, AutoAimController, PositionHoldController,
ShootAssist{Drive,Shooter,Transit,Intake,Vision,Constants}, AutoHiveTarget, BlueManualDriveCommand
and the plain constants classes. Fakes: Drive, Shooter, Transit, Intake, Pinpoint, LimelightVision,
HiveFrameSolver, PedroDrive. Real FTCLib 2.1.1, FTC RobotCore 12.0.0 and Pedro core 3.0.2 jars.
No robot, network or checkout writes.
"""
import os, subprocess, tempfile, glob
from pathlib import Path

root = Path(__file__).resolve().parents[2]
src = root / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
cache = Path(os.environ.get('GRADLE_USER_HOME', str(Path.home() / '.cache/ftc1-gradle'))) / 'caches'
java_home = Path(os.environ.get('JAVA_HOME', '/Library/Java/JavaVirtualMachines/liberica-jdk-17-full.jdk/Contents/Home'))

def jar(pattern):
    hits = sorted(glob.glob(str(cache / pattern), recursive=True))
    if not hits: raise SystemExit('missing jar: ' + pattern)
    return hits[0]

cp = os.pathsep.join([
    jar('**/transformed/core-2.1.1/jars/classes.jar'),
    jar('**/transformed/RobotCore-12.0.0/jars/classes.jar'),
    jar('modules-2/files-2.1/com.pedropathing/core/3.0.2/**/core-3.0.2.jar'),
])
real = ['commands/ShootAssistCommand.java', 'commands/AutoShootCommand.java',
        'commands/BlueManualDriveCommand.java',
        'subsystems/drive/AutoAimController.java', 'subsystems/drive/PositionHoldController.java',
        'subsystems/drive/ShootAssistDrive.java', 'subsystems/drive/ShootAssistConstants.java',
        'subsystems/drive/DriveConstants.java',
        'subsystems/shooter/ShootAssistShooter.java', 'subsystems/shooter/ShooterConstants.java',
        'subsystems/transit/ShootAssistTransit.java', 'subsystems/transit/TransitConstants.java',
        'subsystems/intake/ShootAssistIntake.java',
        'subsystems/vision/ShootAssistVision.java', 'subsystems/vision/AutoHiveTarget.java',
        'subsystems/vision/VisionConstants.java', 'subsystems/field/CellTarget.java']
P = 'package org.firstinspires.ftc.teamcode.subsystems.'
fakes = {
'drive/Drive.java': P + '''drive;
public class Drive extends com.arcrobotics.ftclib.command.SubsystemBase {
 public double[] powers = new double[4]; public int stops;
 public void setWheelPowers(double a,double b,double c,double d){powers=new double[]{a,b,c,d};}
 public void stop(){powers=new double[4];stops++;}
}''',
'drive/PedroDrive.java': P + '''drive;
import com.pedropathing.math.*;
public class PedroDrive { public Pose pose=new Pose(72,72,0); public Velocity vel=new Velocity(0,0,0); public int stops;
 public Pose pose(){return pose;} public Velocity velocity(){return vel;} public void stop(){stops++;} }''',
'shooter/Shooter.java': P + '''shooter;
public class Shooter { public double targetRpm, velocity, pitch=0.5; public boolean running;
 public void startFlywheelsAtTargetRpm(double rpm){targetRpm=rpm;running=true;}
 public void stop(){running=false;targetRpm=0;} public void periodic(){}
 public double getFlywheelVelocityTicksPerSecond(){return velocity;} public double getPitchPosition(){return pitch;} }''',
'transit/Transit.java': P + '''transit;
public class Transit { public enum GateState{CLOSED,OPEN} public GateState gate=GateState.CLOSED;
 public void setGateState(GateState g){gate=g;} public GateState getGateState(){return gate;} }''',
'intake/Intake.java': P + '''intake;
public class Intake { public enum IntakeState{STOP,FORWARD,REVERSED,FAST_FORWARD,FAST_REVERSED,FEED}
 public IntakeState state=IntakeState.STOP;
 public void setIntakeState(IntakeState s){state=s;} public IntakeState getIntakeState(){return state;} }''',
'localization/Pinpoint.java': P + '''localization;
public class Pinpoint { public boolean ready=true; public double x,y,h,speed; public int writes;
 public boolean isReady(){return ready;} public double getXmm(){return x;} public double getYmm(){return y;}
 public double getHeadingDegrees(){return h;} public void setFieldPose(double x,double y,double h){this.x=x;this.y=y;this.h=h;writes++;}
 public void periodic(){} public double getTranslationSpeedMmPerSecond(){return speed;} }''',
'vision/LimelightVision.java': P + '''vision;
public class LimelightVision { public static final class Frame { public final HiveFrameSolver.Observation obs; public final Object rawPose=new Object();
  public Frame(HiveFrameSolver.Observation o){obs=o;} }
 public Frame next; public boolean blue=true;
 public void periodic(){} public Frame getNewFrame(){Frame f=next;next=null;return f;}
 public String getStatus(){return "fake";} public boolean isBlueAlliance(){return blue;} public void stop(){} }''',
'vision/HiveFrameSolver.java': P + '''vision;
public final class HiveFrameSolver { public static final class Observation {
  public final double xMm,yMm,headingDegrees,hiveTiltDegrees,timestamp; public final long captureNanos;
  public Observation(double x,double y,double h,double t,double ts,long n){xMm=x;yMm=y;headingDegrees=h;hiveTiltDegrees=t;timestamp=ts;captureNanos=n;} }
 public static Observation solve(LimelightVision.Frame f,double turret){return f==null?null:f.obs;} }''',
}
with tempfile.TemporaryDirectory(prefix='spinup-shoot-assist-') as tmp:
    t = Path(tmp)
    files = []
    for rel, body in fakes.items():
        p = t / 'src/org/firstinspires/ftc/teamcode/subsystems' / rel
        p.parent.mkdir(parents=True, exist_ok=True); p.write_text(body); files.append(str(p))
    files += [str(src / r) for r in real]
    files.append(str(Path(__file__).with_name('ShootAssistCheck.java')))
    out = t / 'classes'; out.mkdir()
    subprocess.run([str(java_home / 'bin/javac'), '-nowarn', '-encoding', 'UTF-8', '-cp', cp, '-d', str(out), *files], check=True)
    subprocess.run([str(java_home / 'bin/java'), '-cp', str(out) + os.pathsep + cp, 'ShootAssistCheck'], check=True)
