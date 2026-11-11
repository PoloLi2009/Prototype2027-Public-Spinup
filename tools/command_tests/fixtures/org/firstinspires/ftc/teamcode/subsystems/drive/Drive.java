package org.firstinspires.ftc.teamcode.subsystems.drive;
public class Drive extends com.arcrobotics.ftclib.command.SubsystemBase {
 public enum DriveState {STOP,TELEOP} public DriveState state=DriveState.STOP;
 public double forward,strafe,turn;
 public void setDriveState(DriveState state){this.state=state;}
 public void moveRobot(double forward,double strafe,double turn){this.forward=forward;this.strafe=strafe;this.turn=turn;}
 public int fieldCalls;
 public void moveFieldCentric(double f,double s,double t,double heading){
  fieldCalls++;double r=Math.toRadians(heading);moveRobot(-(f*Math.cos(r)-s*Math.sin(r)),f*Math.sin(r)+s*Math.cos(r),t);
 }
 public void stop(){forward=strafe=turn=0;}
}
