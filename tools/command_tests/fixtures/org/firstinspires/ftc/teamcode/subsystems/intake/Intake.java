package org.firstinspires.ftc.teamcode.subsystems.intake;
public class Intake extends com.arcrobotics.ftclib.command.SubsystemBase {
 public enum IntakeState {STOP,FORWARD,FAST_FORWARD,REVERSED,FAST_REVERSED,FEED}
 public IntakeState state=IntakeState.STOP,output=IntakeState.STOP;
 public void setIntakeState(IntakeState s){state=s;} public void stop(){state=IntakeState.STOP;}
 public void periodic(){output=state;}
}
