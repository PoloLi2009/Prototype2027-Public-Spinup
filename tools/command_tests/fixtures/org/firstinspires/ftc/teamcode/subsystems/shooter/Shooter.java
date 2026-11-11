package org.firstinspires.ftc.teamcode.subsystems.shooter;
public class Shooter extends com.arcrobotics.ftclib.command.SubsystemBase {
 public enum PitchState {LOW,HIGH}
 public PitchState pitchState=PitchState.LOW;
 public double pitch=.5,rpm,output; public boolean ready;
 public void setPitchState(PitchState s){pitchState=s;pitch=s==PitchState.HIGH?ShooterConstants.pitchHighPosition:ShooterConstants.pitchLowPosition;}
 public double getPitchPosition(){return pitch;}
 public void adjustPitch(double delta){pitch=Math.max(0,Math.min(1,pitch+delta));}
 public void startFlywheelsAtTargetRpm(double value){rpm=value;} public void stop(){rpm=0;}
 public boolean isAtTargetRpm(){return rpm>0&&ready;} public void periodic(){output=rpm;}
}
