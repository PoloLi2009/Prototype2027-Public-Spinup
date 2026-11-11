import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.commands.SequencedShotCommand;
import org.firstinspires.ftc.teamcode.opmodes.teleops.*;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.field.CellTarget;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;

public class SoloBehaviorTest {
 static int checks;
 static void check(boolean value,String message){checks++;if(!value)throw new AssertionError(message);}
 static HardwareMap map=new HardwareMap();
 static Gamepad pad; static Shooter shooter; static Intake intake; static Transit gate; static SequencedShotCommand shot;
 static void fixture(){
  ElapsedTime.now=0;pad=new Gamepad();shooter=new Shooter(map);intake=new Intake(map);gate=new Transit(map);
  shot=new SequencedShotCommand(intake,shooter,gate,pad,3000,()->true,
   Intake.IntakeState.FAST_FORWARD,Intake.IntakeState.FAST_REVERSED,100,()->pad.a&&pad.right_trigger<.5,true);
 }
 static void step(double ms){ElapsedTime.now+=ms;shot.execute();}
 static void gateOpen(){pad.right_trigger=1;shooter.atSpeed=true;step(0);step(251);check(gate.state==Transit.GateState.OPEN,"gate after stable RPM");}
 public static void main(String[] args)throws Exception{
  fixture();pad.right_trigger=1;step(1000);
  check(shooter.enabled&&shooter.piRequests>0,"RT uses PI even without vision/aim");
  check(gate.state==Transit.GateState.CLOSED&&intake.state==Intake.IntakeState.STOP,"no feed before speed");
  shooter.atSpeed=true;step(0);step(249);check(gate.state==Transit.GateState.CLOSED,"hold speed for 250ms");
  step(2);check(gate.state==Transit.GateState.OPEN,"open after qualification");
  step(499);check(intake.state==Intake.IntakeState.STOP,"wait for gate travel");
  step(2);check(intake.state==Intake.IntakeState.FEED,"feed when gate cleared");
  shooter.atSpeed=false;step(10);check(gate.state==Transit.GateState.CLOSED&&intake.state==Intake.IntakeState.STOP,"drop RPM while feeding stops feed");
  shooter.atSpeed=true;step(251);check(shot.getShotState()==SequencedShotCommand.ShotState.OPENING_GATE,"requalify after drop");
  shooter.atSpeed=false;step(1);check(gate.state==Transit.GateState.CLOSED,"drop RPM during gate opening closes gate");
  fixture();gateOpen();pad.dpad_right=true;step(10);check(shot.getTargetRpm()==3100&&gate.state==Transit.GateState.CLOSED,"RPM change resets gate");
  step(10);check(shot.getTargetRpm()==3100,"RPM step is edge triggered");
  fixture();gateOpen();pad.dpad_up=true;step(10);check(shooter.pitch==.51&&gate.state==Transit.GateState.CLOSED,"pitch change restarts qualification");
  fixture();pad.a=true;step(0);check(shooter.enabled&&gate.state==Transit.GateState.CLOSED,"pre-spin closed gate");
  pad.left_bumper=true;pad.right_trigger=1;step(0);check(!shooter.enabled&&intake.state==Intake.IntakeState.FAST_REVERSED,"reverse overrides shoot and pre-spin");
  pad.left_bumper=false;pad.a=false;pad.right_trigger=0;pad.left_trigger=1;step(0);check(intake.state==Intake.IntakeState.FAST_FORWARD&&!shooter.enabled,"collection independent of shooter");
  shot.end(true);check(!shooter.enabled&&gate.state==Transit.GateState.CLOSED&&intake.state==Intake.IntakeState.STOP,"command end safe");
  SoloControlSafety safe=new SoloControlSafety();
  check(safe.update(false,false,false),"START with held controls blocked");
  check(!safe.update(false,true,false),"neutral arms controls");
  check(safe.update(true,true,false),"B stops even neutral");
  check(safe.update(false,false,false),"held RT cannot restart after B");
  check(!safe.update(false,true,false),"release after B rearms");
  check(safe.update(false,false,true),"field frame change with stick held blocked");
  check(!safe.update(false,true,true),"neutral accepts field frame");
  check(safe.update(false,false,false),"OD fallback while moving requires neutral");

  // Execute the actual OpMode through INIT/START/loop/STOP using the fake SDK.
  Pinpoint.constructed=Pinpoint.reads=0;
  SpinupSolo solo=new SpinupSolo();solo.stopAt=8;
  solo.script=t->{
   if(t==1){check(Shooter.last.power==0&&Intake.last.output==Intake.IntakeState.STOP&&Turret.last.holds==0,"INIT no motors");}
   if(t==2){solo.gamepad1.right_trigger=1;solo.gamepad1.left_stick_y=-1;}
   if(t==3){check(Shooter.last.enabled&&Shooter.last.piRequests>0,"Solo RT calls PI in field drive");check(Math.abs(Drive.last.f-1)<1e-9 && Math.abs(Drive.last.s)<1e-9,"Blue START heading is driver zero");
    check(Math.abs(Shooter.last.target*org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.rightEncoderTicksPerRevolution/60-900)<1e-9,"Solo starts at 900 TPS, not 1500");solo.gamepad1.b=true;}
   if(t==4){check(Drive.last.f==0&&Shooter.last.power==0,"B stops chassis/shooter same iteration");solo.gamepad1.b=false;}
   if(t==5){check(Shooter.last.power==0,"B latch blocks held RT");solo.gamepad1.right_trigger=0;solo.gamepad1.left_stick_y=0;}
   if(t==6){solo.gamepad1.a=true;solo.gamepad1.left_trigger=1;}
   if(t==7){check(Shooter.last.enabled&&Intake.last.output==Intake.IntakeState.FAST_FORWARD&&Transit.last.output==Transit.GateState.CLOSED,"A+LT collects while pre-spinning");}
  };
  solo.runOpMode();
  check(Pinpoint.constructed==1&&Pinpoint.reads>0,"default Solo initializes Pinpoint without a button");
  check(Turret.last.holds>0&&Turret.last.requested==12,"all loops hold original turret angle");
  check(Turret.last.shutdown&&Shooter.last.power==0&&Drive.last.f==0&&Intake.last.output==Intake.IntakeState.STOP&&Transit.last.output==Transit.GateState.CLOSED,"STOP all outputs safe");

  SpinupSolo directions=new SpinupSolo();directions.stopAt=12;
  directions.script=t->{
   if(t==2){Pinpoint.heading=115;ElapsedTime.now+=60;directions.gamepad1.left_stick_y=-1;directions.gamepad1.right_stick_x=.5f;}
   if(t==3){
    check(Math.abs(Drive.last.f)<1e-9 && Math.abs(Drive.last.s-2.0/3)<1e-9 && Math.abs(Drive.last.t-1.0/3)<1e-9,
          "Blue heading relative to START; normalized translation plus positive manual turn");
    directions.gamepad1.left_stick_y=0;directions.gamepad1.right_stick_x=0;directions.gamepad1.left_stick_x=1;
   }
   if(t==4){check(Math.abs(Drive.last.f+1)<1e-9 && Math.abs(Drive.last.s)<1e-9,"Blue lateral at 90 degrees");directions.gamepad1.left_stick_button=true;}
   if(t==5){check(Math.abs(Drive.last.f)<1e-9 && Math.abs(Drive.last.s-1)<1e-9,"L3 resets driver heading while moving");directions.gamepad1.left_stick_button=false;directions.gamepad1.y=true;directions.gamepad1.right_bumper=true;}
   if(t==6){check(Math.abs(Drive.last.s-1)<1e-9,"Y and RB do not change Blue driving");directions.gamepad1.y=false;directions.gamepad1.right_bumper=false;}
   if(t==8){directions.gamepad1.back=true;directions.gamepad1.left_stick_x=0;directions.gamepad1.left_stick_y=-1;}
   if(t==9){check(Drive.last.f==0&&Drive.last.s==0,"OD fallback requires centered sticks");directions.gamepad1.left_stick_y=0;}
   if(t==10){directions.gamepad1.left_stick_y=-1;}
   if(t==11){check(Drive.last.f==1&&Drive.last.s==0,"Blue robot-relative fallback");}
  };
  directions.runOpMode();
  Pinpoint.heading=25;
  Pinpoint.fail=true;Pinpoint.constructed=Pinpoint.reads=0;
  SpinupSolo failed=new SpinupSolo();failed.stopAt=5;
  failed.script=t->{
   if(t==2){failed.gamepad1.right_trigger=1;failed.gamepad1.left_stick_y=-1;}
   if(t==3){check(Shooter.last.enabled&&Drive.last.f==1,"Pinpoint fault allows corrected manual drive and PI shooting");}
  };
  failed.runOpMode();
  check(Pinpoint.constructed==1&&Pinpoint.reads==1,"I2C exception latched, no endless retry");
  check(Turret.last.shutdown,"optional sensor failure still cleans turret");
  Pinpoint.constructed=Pinpoint.reads=0;
  SpinupSolo skipOd=new SpinupSolo();skipOd.gamepad1.back=true;skipOd.stopAt=4;
  skipOd.runOpMode();
  check(Pinpoint.constructed==0&&Pinpoint.reads==0,"holding Back at INIT bypasses Pinpoint");
  Pinpoint.fail=false;
  verifySpeedOnlyShot();
  verifyBlueDriveMapping();
  Pinpoint.fail=false;Drive.throwOnStop=true;
  SpinupSolo cleanup=new SpinupSolo();cleanup.stopAt=3;
  try{cleanup.runOpMode();}catch(RuntimeException expected){}
  check(Turret.last.shutdown&&Shooter.last.power==0,"one failed STOP write does not skip other cleanup");
  Drive.throwOnStop=false;
  System.out.println("PASS "+checks+" Solo/shot behavior checks (fake SDK, devices and clock)");
 }

 static void verifySpeedOnlyShot() throws InterruptedException {
  int cameras=LimelightVision.constructed;
  Turret.holdReady=false; // Aiming/turret-lock readiness must not block this Solo shot.
  LimelightVision.nextPose=null;
  SpinupSolo solo=new SpinupSolo();solo.stopAt=10;
  solo.script=t->{
   if(t==2){solo.gamepad1.right_trigger=1;Shooter.last.atSpeed=true;solo.gamepad1.right_stick_x=.5f;}
   if(t==3){
    check(Shooter.last.absoluteTolerance==30,"Solo explicitly selects fixed 30 TPS tolerance");
    check(Drive.last.t==.5&&Transit.last.output==Transit.GateState.OPEN,
      "at speed opens gate immediately with manual turning, no vision and no turret readiness");
    check(Intake.last.output==Intake.IntakeState.STOP,"gate travel still completes before feed");
    ElapsedTime.now+=501;
   }
   if(t==4){check(Intake.last.output==Intake.IntakeState.FEED,"feeds after gate travel");solo.gamepad1.right_stick_button=true;solo.gamepad1.x=true;}
   if(t==5){check(Intake.last.output==Intake.IntakeState.FEED&&Drive.last.t==.5,"R3 and X no longer toggle aiming or block manual shot");Shooter.last.atSpeed=false;}
   if(t==6){check(Transit.last.output==Transit.GateState.CLOSED&&Intake.last.output==Intake.IntakeState.STOP,"out of speed band stops feed and closes gate");Shooter.last.atSpeed=true;}
   if(t==7){check(Transit.last.output==Transit.GateState.OPEN,"speed recovery immediately reopens gate");solo.gamepad1.left_bumper=true;}
   if(t==8){check(!Shooter.last.enabled&&Intake.last.output==Intake.IntakeState.FAST_REVERSED,"unjam retains priority over shot");solo.gamepad1.left_bumper=false;solo.gamepad1.right_trigger=0;}
   if(t==9){check(!Shooter.last.enabled&&Transit.last.output==Transit.GateState.CLOSED,"release RT stops and closes gate");}
  };
  solo.runOpMode();
  check(LimelightVision.constructed==cameras,"Solo no longer constructs a camera/vision pipeline");
  Turret.holdReady=true;
 }
 static void verifyBlueDriveMapping() {
  double[] heading={.5}, inputs={0,0,0}; boolean[] ready={true};
  Drive actual=new Drive(map); BlueReference reference=new BlueReference();reference.heading=heading[0];reference.resetDriverHeading();
  var candidate=new org.firstinspires.ftc.teamcode.commands.BlueManualDriveCommand(actual,
    ()->inputs[0],()->inputs[1],()->inputs[2],()->heading[0],()->ready[0],turn->turn);
  candidate.initialize();java.util.Random random=new java.util.Random(16095);
  for(int i=0;i<2000;i++) {
   heading[0]=reference.heading=(random.nextDouble()-.5)*8*Math.PI;
   for(int j=0;j<3;j++)inputs[j]=(random.nextDouble()-.5)*2;
   if(i%17==0){candidate.resetDriverHeading();reference.resetDriverHeading();}
   candidate.execute();
   double f=deadband(inputs[0]),s=deadband(inputs[1]),t=deadband(inputs[2]);reference.moveFieldCentric(f,s,t);
   check(Math.abs(actual.powers[0]-reference.leftFrontMotor.power)<1e-12 && Math.abs(actual.powers[1]-reference.leftBackMotor.power)<1e-12
     &&Math.abs(actual.powers[2]-reference.rightFrontMotor.power)<1e-12 &&Math.abs(actual.powers[3]-reference.rightBackMotor.power)<1e-12,
     "Blue source and Solo wheel powers identical");
  }
  ready[0]=reference.ready=false;candidate.execute();reference.moveFieldCentric(deadband(inputs[0]),deadband(inputs[1]),deadband(inputs[2]));
  check(Math.abs(actual.powers[0]-reference.leftFrontMotor.power)<1e-12,"Blue unavailable OD fallback");
  inputs[0]=Double.POSITIVE_INFINITY;candidate.execute();check(actual.f==0&&actual.s==0&&actual.t==0,"non-finite input stops wheels");
  candidate.end(true);check(actual.f==0&&actual.s==0&&actual.t==0,"cancellation stops wheels");
  Pinpoint.heading=25;
 }
 static double deadband(double x){return Math.abs(x)>org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.joystickDeadband?x:0;}
}
