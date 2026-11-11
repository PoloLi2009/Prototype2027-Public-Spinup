package org.firstinspires.ftc.teamcode.subsystems.transit;
public class Transit extends com.arcrobotics.ftclib.command.SubsystemBase {
 public enum GateState {CLOSED,OPEN}
 public GateState state=GateState.CLOSED,output=GateState.CLOSED;
 public void setGateState(GateState s){state=s;} public void periodic(){output=state;}
}
