"""Exercise actual Solo routing and shot state machine with deterministic fake devices/time."""
from pathlib import Path
import os, subprocess, tempfile, zipfile

root = Path(__file__).resolve().parents[2]
src = root / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
pkg = 'org.firstinspires.ftc.teamcode'
stubs = {
'com/arcrobotics/ftclib/gamepad/GamepadEx.java': '''package com.arcrobotics.ftclib.gamepad;
import com.qualcomm.robotcore.hardware.Gamepad; public class GamepadEx {private Gamepad g;
public GamepadEx(Gamepad g){this.g=g;} public double getLeftY(){return -g.left_stick_y;}
public double getLeftX(){return g.left_stick_x;} public double getRightX(){return g.right_stick_x;} }''',
'com/qualcomm/robotcore/hardware/DcMotorSimple.java': '''package com.qualcomm.robotcore.hardware; public interface DcMotorSimple {enum Direction{FORWARD,REVERSE}}''',
'com/arcrobotics/ftclib/command/InstantCommand.java': '''package com.arcrobotics.ftclib.command;
public class InstantCommand extends CommandBase {private final Runnable run;
 public InstantCommand(Runnable run,Object... requirements){this.run=run;}
 @Override public void initialize(){run.run();} @Override public boolean isFinished(){return true;} }''',
'com/arcrobotics/ftclib/command/CommandBase.java': '''package com.arcrobotics.ftclib.command;
public class CommandBase { public void addRequirements(Object... x) {} public void initialize() {} public void execute() {} public void end(boolean b) {} public boolean isFinished(){return false;} }''',
'com/arcrobotics/ftclib/command/CommandScheduler.java': '''package com.arcrobotics.ftclib.command;
public class CommandScheduler { static CommandScheduler s=new CommandScheduler(); public static CommandScheduler getInstance(){return s;} public void reset(){} }''',
'com/qualcomm/robotcore/hardware/Gamepad.java': '''package com.qualcomm.robotcore.hardware;
public class Gamepad { public boolean a,b,x,y,back,left_bumper,right_bumper,left_stick_button,right_stick_button,dpad_left,dpad_right,dpad_up,dpad_down;
public float left_stick_x,left_stick_y,right_stick_x,left_trigger,right_trigger; }''',
'com/qualcomm/robotcore/hardware/HardwareMap.java': 'package com.qualcomm.robotcore.hardware; public class HardwareMap {}',
'com/qualcomm/robotcore/util/Range.java': '''package com.qualcomm.robotcore.util;
public class Range { public static double clip(double value,double min,double max){return Math.max(min,Math.min(value,max));} }''',
'com/qualcomm/robotcore/util/ElapsedTime.java': '''package com.qualcomm.robotcore.util;
public class ElapsedTime { public static double now; private double start=now; public void reset(){start=now;} public double milliseconds(){return now-start;} public double seconds(){return milliseconds()/1000;} }''',
'com/qualcomm/robotcore/eventloop/opmode/TeleOp.java': '''package com.qualcomm.robotcore.eventloop.opmode; public @interface TeleOp {String name();String group();}''',
'com/qualcomm/robotcore/eventloop/opmode/LinearOpMode.java': '''package com.qualcomm.robotcore.eventloop.opmode;
import com.qualcomm.robotcore.hardware.*; import com.qualcomm.robotcore.util.ElapsedTime;
public abstract class LinearOpMode {
 public HardwareMap hardwareMap=new HardwareMap(); public Gamepad gamepad1=new Gamepad(); public Gamepad gamepad2=new Gamepad(); public Telemetry telemetry=new Telemetry();
 public int tick, startAt=1, stopAt=10; public java.util.function.IntConsumer script=t->{};
 public abstract void runOpMode() throws InterruptedException;
 public boolean isStarted(){return tick>=startAt;} public boolean isStopRequested(){return tick>=stopAt;}
 public boolean opModeIsActive(){return tick<stopAt;}
 public void sleep(long ms){ElapsedTime.now+=ms;script.accept(++tick);}
 public static class Telemetry {public void setMsTransmissionInterval(int i){} public void addLine(String s){}
 public void addData(String s,Object v){} public void addData(String s,String f,Object... v){} public void update(){} }
}''',
}
def stub(name, text):
    stubs[pkg.replace('.','/')+'/'+name+'.java'] = 'package '+pkg+'.'+name.rsplit('/',1)[0].replace('/','.')+';\n'+text
stub('subsystems/drive/Drive', '''import com.qualcomm.robotcore.hardware.HardwareMap;
public class Drive {public static Drive last; public static boolean throwOnStop; public double f,s,t; public double[] powers=new double[4]; public int fieldCalls;
 public Drive(HardwareMap h){last=this;} public void moveRobot(double a,double b,double c){f=a;s=b;t=c;}
 public void moveFieldCentric(double a,double b,double c,double h){
 fieldCalls++;double r=Math.toRadians(h);moveRobot(-(a*Math.cos(r)-b*Math.sin(r)),a*Math.sin(r)+b*Math.cos(r),c);}
 public void setWheelPowers(double lf,double lb,double rf,double rb){powers=new double[]{lf,lb,rf,rb};f=(lf+lb+rf+rb)/4;s=(lf-lb-rf+rb)/4;t=(lf+lb-rf-rb)/4;}
 public void stop(){powers=new double[4];f=s=t=0;if(throwOnStop)throw new RuntimeException("offline");} }''')
stub('subsystems/intake/Intake', '''import com.qualcomm.robotcore.hardware.HardwareMap;
public class Intake {public enum IntakeState {STOP,FORWARD,REVERSED,FAST_FORWARD,FAST_REVERSED,FEED}
 public static Intake last; public IntakeState state=IntakeState.STOP, output=state;
 public Intake(HardwareMap h){last=this;} public void setIntakeState(IntakeState s){state=s;}
 public void stop(){state=IntakeState.STOP;} public IntakeState getIntakeState(){return state;} public void periodic(){output=state;} }''')
stub('subsystems/transit/Transit', '''import com.qualcomm.robotcore.hardware.HardwareMap;
public class Transit {public enum GateState{CLOSED,OPEN} public static Transit last; public GateState state=GateState.CLOSED,output=state;
 public Transit(HardwareMap h){last=this;} public void setGateState(GateState s){state=s;}
 public GateState getGateState(){return state;} public void periodic(){output=state;} }''')
stub('subsystems/shooter/Shooter', '''import com.qualcomm.robotcore.hardware.HardwareMap;
public class Shooter {public double readyTolerance=.06,absoluteTolerance=-1;public void setReadyAbsoluteToleranceTicksPerSecond(double x){absoluteTolerance=x;}public double getReadyToleranceTicksPerSecond(){return absoluteTolerance;}public void setReadyRelativeTolerance(double x){readyTolerance=x;}public double getReadyRelativeTolerance(){return readyTolerance;}public enum PitchState{HIGH,LOW} public PitchState pitchState=PitchState.LOW;
 public void setPitchState(PitchState s){pitchState=s;pitch=.5;} public PitchState getPitchState(){return pitchState;}
 public static Shooter last; public boolean enabled,atSpeed; public double target,pitch=.5,power; public int piRequests;
 public Shooter(HardwareMap h){last=this;} public void startFlywheelsAtTargetRpm(double r){target=r;enabled=true;piRequests++;}
 public void stop(){enabled=false;} public void periodic(){power=enabled?.5:0;}
 public double getPitchPosition(){return pitch;} public void adjustPitch(double d){pitch=Math.max(0,Math.min(1,pitch+d));}
 public boolean isAtTargetRpm(){return enabled&&atSpeed;}
 public double getFlywheelVelocityTicksPerSecond(){return atSpeed?target*ShooterConstants.rightEncoderTicksPerRevolution/60:0;}
 public double getCommandedFlywheelPower(){return power;} public double getFeedforwardPower(){return 0;} public double getProportionalPower(){return 0;} public double getIntegralPower(){return 0;} public double getDerivativePower(){return 0;} public double getVelocityUpdateSeconds(){return .02;} }''')
stub('subsystems/turret/Turret', '''import com.qualcomm.robotcore.hardware.HardwareMap;
public class Turret {public static boolean holdReady=true;public static Turret last; public double angle=12, requested;public int holds;public boolean shutdown;
 public Turret(HardwareMap h){last=this;} public double getAngleDegrees(){return angle;}
 public boolean holdAtPhysicalDegrees(double a){requested=a;holds++;return holdReady;} public void periodic(){} public void stop(){} public void shutdown(){shutdown=true;} }''')
stub('subsystems/localization/Pinpoint', '''import com.qualcomm.robotcore.hardware.HardwareMap;
public class Pinpoint {public static int constructed,reads;public static boolean fail,ready=true;public static double heading=25;
 public Pinpoint(HardwareMap h){constructed++;} public void periodic(){reads++;if(fail)throw new RuntimeException("I2C");}
 public double getHeadingDegrees(){return heading;} public boolean isReady(){return ready;}
 public double getXmm(){return 0;} public double getYmm(){return 0;} }''')
stub('subsystems/localization/PinpointConstants', '''public class PinpointConstants {
 public static double testStartFieldXmm, testStartFieldYmm, testStartFieldHeadingDegrees; }''')
stub('subsystems/vision/LimelightVision', '''import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class LimelightVision {
 public static int constructed; public static LimelightVision last; public static Pose2d nextPose;
 public static class Frame { public final Pose2d pose; Frame(Pose2d pose){this.pose=pose;} }
 public boolean blue=true, stopped;
 public LimelightVision(HardwareMap h){last=this;constructed++;}
 public void toggleAlliance(){blue=!blue;} public boolean isBlueAlliance(){return blue;}
 public void periodic(){} public String getStatus(){return "fake camera";}
 public Frame getNewFrame(){return nextPose==null?null:new Frame(nextPose);}
 public void stop(){stopped=true;} }''')
stub('subsystems/localization/VisionLocalization', '''import com.arcrobotics.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
public class VisionLocalization {
 private Pose2d pose;
 public void clear(){pose=null;}
 public void update(double time,Pose2d odom,double turret,LimelightVision.Frame frame){if(frame!=null)pose=frame.pose;}
 public boolean hasFieldPose(){return pose!=null;} public Pose2d getEstimatedPosition(){return pose;}
 public String getStatus(){return pose==null?"unseeded":"seeded";} }''')
# Constants remain the real robot sources; stub only their enum dependencies.
stubs['com/qualcomm/robotcore/hardware/DcMotor.java']='package com.qualcomm.robotcore.hardware; public interface DcMotor {}'
reference = root.parent/'Overtune/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/drive/Drive.java'
# Pin the exact Blue implementation that Solo was ported from. Overtune is a separate
# actively edited checkout; its subsequent sensor changes must not silently change this oracle.
reference_source=subprocess.run(['git','-C',str(root.parent/'Overtune'),'show',
    '5045231:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/drive/Drive.java'],
    check=True,capture_output=True,text=True).stdout
def method(signature):
    start=reference_source.index(signature)
    brace=reference_source.index('{',start); depth=1; end=brace+1
    while depth:
        depth += (reference_source[end]=='{') - (reference_source[end]=='}'); end+=1
    return reference_source[start:end]
methods='\n'.join(method(sig) for sig in ['public void resetDriverHeading()', 'public double getDriverHeadingRadians()',
    'public void moveFieldCentric(', 'public void moveRobot(', 'private void setMotorPowers(', 'public void stop()'])
methods=methods.replace('pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY','ready')
methods=methods.replace('pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY','!ready')
methods=methods.replace('pinpoint.getHeading(AngleUnit.RADIANS)','heading')
stubs['BlueReference.java']='public class BlueReference { public boolean ready=true; public double heading; private double driverHeadingReferenceRadians=Double.NaN; public static class Wheel {public double power; void setPower(double p){power=p;}} public final Wheel leftFrontMotor=new Wheel(),leftBackMotor=new Wheel(),rightFrontMotor=new Wheel(),rightBackMotor=new Wheel();'+methods+'}'
with tempfile.TemporaryDirectory(prefix='spinup-solo-tests-') as tmp:
    out=Path(tmp)
    for path,code in stubs.items():
        dest=out/path;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(code)
    actual=[src/name for name in ['commands/SequencedShotCommand.java','commands/ShootCommand.java','commands/FlywheelRpmCommand.java',
        'commands/BlueManualDriveCommand.java','commands/RunIntakeCommand.java','commands/SetPitchCommand.java','commands/DriveWithAimCommand.java','commands/TurretStartupHoldCommand.java',
        'commands/VisionReadCommand.java','commands/VisionCellChassisAimCommand.java',
        'opmodes/teleops/SpinupSolo.java','opmodes/teleops/SoloControlSafety.java',
        'subsystems/shooter/ShooterConstants.java','subsystems/drive/DriveConstants.java','subsystems/field/CellTarget.java']]
    gradle_cache = Path(os.environ.get('GRADLE_USER_HOME', Path.home()/'.cache/ftc1-gradle'))
    ftclib = next((gradle_cache/'caches/modules-2/files-2.1/org.ftclib.ftclib/core/2.1.1').rglob('*.aar'))
    geometry_jar = out/'ftclib.jar'
    with zipfile.ZipFile(ftclib) as archive:
        geometry_jar.write_bytes(archive.read('classes.jar'))
    java=Path('/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin')
    subprocess.run([str(java/'javac'),'-cp',str(geometry_jar),'-d',tmp,*map(str,out.rglob('*.java')),*map(str,actual),str(Path(__file__).with_name('SoloBehaviorTest.java'))],check=True)
    subprocess.run([str(java/'java'),'-cp',tmp+os.pathsep+str(geometry_jar),'SoloBehaviorTest'],check=True)
