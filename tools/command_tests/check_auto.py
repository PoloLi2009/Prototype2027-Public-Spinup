"""Actual AutoFormal entry point + actual FTCLib/Pedro paths, with fake hardware and time."""
from pathlib import Path
import os, subprocess, tempfile, zipfile
root=Path(__file__).resolve().parents[2]
src=root/'TeamCode/src/main/java'
pkg='org/firstinspires/ftc/teamcode'
base=Path(__file__).parent/'fixtures'
java=Path('/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin')
cache=Path(os.environ.get('GRADLE_USER_HOME',Path.home()/'.cache/ftc1-gradle'))/'caches/modules-2/files-2.1'
stubs={
'com/qualcomm/robotcore/hardware/HardwareMap.java':'package com.qualcomm.robotcore.hardware; public class HardwareMap {}',
'com/qualcomm/robotcore/util/ElapsedTime.java':'''package com.qualcomm.robotcore.util; public class ElapsedTime {
public static long now; private long start=now; public void reset(){start=now;} public double milliseconds(){return now-start;} public double seconds(){return milliseconds()/1000;}}''',
'com/qualcomm/robotcore/eventloop/opmode/Autonomous.java':'package com.qualcomm.robotcore.eventloop.opmode; public @interface Autonomous {String name();String group();}',
'com/qualcomm/robotcore/eventloop/opmode/LinearOpMode.java':'''package com.qualcomm.robotcore.eventloop.opmode;
import com.qualcomm.robotcore.util.ElapsedTime; import com.qualcomm.robotcore.hardware.HardwareMap;
public abstract class LinearOpMode { public HardwareMap hardwareMap=new HardwareMap(); public Telemetry telemetry=new Telemetry();
public boolean started,stop; public int waits; public int initLoops=3; public long stopTime=1000000;
public abstract void runOpMode() throws InterruptedException; public boolean opModeInInit(){if(initLoops-->0)return true;started=true;return false;}
public boolean isStopRequested(){return stop||ElapsedTime.now>=stopTime;} public void waitForStart(){waits++;started=true;}
public boolean opModeIsActive(){return started&&!isStopRequested();} public void sleep(long ms){ElapsedTime.now+=ms;}
public static class Telemetry {public String result;public void setMsTransmissionInterval(int x){}public void addData(String k,Object v){if(k.equals("AutoFormal"))result=String.valueOf(v);} public void addData(String k,String fmt,Object... args){}public void update(){}}
}''',
pkg+'/pedro/Constants.java':'package org.firstinspires.ftc.teamcode.pedro; public class Constants {public static Object foresightConfig;}',
pkg+'/subsystems/drive/PedroDrive.java':'''package org.firstinspires.ftc.teamcode.subsystems.drive;
import com.arcrobotics.ftclib.command.SubsystemBase; import com.pedropathing.paths.Path; import com.pedropathing.math.Pose; import com.qualcomm.robotcore.hardware.HardwareMap;
public class PedroDrive extends SubsystemBase {public static PedroDrive last;public static boolean ready=true,complete=true,fail;
public boolean busy;public int starts,stops,updates;private Pose pose;public PedroDrive(HardwareMap h){last=this;}
public void periodic(){updates++;if(fail)throw new RuntimeException("I2C");if(complete)busy=false;}
public boolean isReady(){return ready;}public boolean isBusy(){return busy;}public void setPose(Pose p){pose=p;}public Pose pose(){return pose;}
public void follow(Path p){busy=true;starts++;}public void stop(){busy=false;stops++;}}
''',
pkg+'/subsystems/turret/Turret.java':'''package org.firstinspires.ftc.teamcode.subsystems.turret;
import com.arcrobotics.ftclib.command.SubsystemBase;import com.qualcomm.robotcore.hardware.HardwareMap;
public class Turret extends SubsystemBase {public static Turret last;public boolean shutdown;public int holds;
public Turret(HardwareMap h){last=this;}public double getAngleDegrees(){return 12;}
public boolean holdAtPhysicalDegrees(double a){holds++;return true;}public void stop(){}public void shutdown(){shutdown=true;}}
''',
'AutoEntryCheck.java':'''import org.firstinspires.ftc.teamcode.opmodes.auto.SpinupAutoFormal;
import org.firstinspires.ftc.teamcode.pedro.Constants; import org.firstinspires.ftc.teamcode.subsystems.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret; import com.qualcomm.robotcore.util.ElapsedTime;
public class AutoEntryCheck {static void check(boolean b,String s){if(!b)throw new AssertionError(s);}
static SpinupAutoFormal init(){ElapsedTime.now=0;PedroDrive.last=null;PedroDrive.ready=true;PedroDrive.complete=true;PedroDrive.fail=false;Turret.last=null;return new SpinupAutoFormal();}
static void stopped(){check(!PedroDrive.last.busy&&PedroDrive.last.stops>0&&Turret.last.shutdown,"all output owners cleaned");}
public static void main(String[] args)throws Exception{
var a=init();Constants.foresightConfig=null;a.runOpMode();check(PedroDrive.last==null&&Turret.last==null&&a.waits==1,"uncalibrated auto never constructs hardware");
a=init();Constants.foresightConfig=new Object();a.stop=true;a.runOpMode();stopped();check(PedroDrive.last.starts==0&&Turret.last.holds==0,"STOP in INIT never runs a path or turret");
a=init();PedroDrive.ready=false;a.runOpMode();stopped();check(PedroDrive.last.starts==0,"unready OD never starts path");
a=init();a.runOpMode();stopped();check(PedroDrive.last.starts==8&&a.telemetry.result.contains("0"),"all eight paths finish on Pedro isBusy");
check(ElapsedTime.now>=15000&&ElapsedTime.now<15500,"three reserved stops each consume five seconds");
a=init();PedroDrive.complete=false;a.runOpMode();stopped();check(ElapsedTime.now>=55000&&ElapsedTime.now<55500&&PedroDrive.last.starts==8,"eight independent path deadlines plus three reserved five-second stops");
a=init();a.stopTime=200;a.runOpMode();stopped();
a=init();PedroDrive.fail=true;try{a.runOpMode();throw new AssertionError("exception missing");}catch(RuntimeException expected){}stopped();
System.out.println("PASS: actual AutoFormal uncalibrated guard, INIT/STOP, OD readiness, 8 paths, reserved stops and exception cleanup");}}
'''}
with tempfile.TemporaryDirectory(prefix='spinup-auto-entry-') as temp:
    out=Path(temp)
    for name,code in stubs.items():
        p=out/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(code)
    with zipfile.ZipFile(next((cache/'org.ftclib.ftclib/core/2.1.1').rglob('*.aar'))) as z:
        (out/'ftclib.jar').write_bytes(z.read('classes.jar'))
    cp=os.pathsep.join(map(str,[out/'ftclib.jar',next((cache/'com.pedropathing/core/3.0.2').rglob('*.jar'))]))
    actual=[src/pkg/p for p in ['opmodes/auto/SpinupAutoFormal.java','opmodes/auto/AutoFormalRoute.java','commands/FollowPathCommand.java','commands/TurretStartupHoldCommand.java']]
    subprocess.run([str(java/'javac'),'-cp',cp,'-d',temp,*map(str,out.rglob('*.java')),*map(str,actual)],check=True)
    subprocess.run([str(java/'java'),'-cp',temp+os.pathsep+cp,'AutoEntryCheck'],check=True)
