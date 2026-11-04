package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Controls ball release into the rotating shooter with the entry blocking rod. */
public class Transit extends SubsystemBase {
    private final Servo gateServo;
    private GateState gateState;

    public enum GateState {
        CLOSED(TransitConstants.gateClosedPosition),
        OPEN(TransitConstants.gateOpenPosition);

        final double position;

        GateState(double position) {
            this.position = position;
        }
    }

    public Transit(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, TransitConstants.gateServoName);
        setGateState(GateState.CLOSED);
    }

    public void setGateState(GateState gateState) {
        this.gateState = gateState;
    }

    public GateState getGateState() {
        return gateState;
    }

    @Override
    public void periodic() {
        gateServo.setPosition(gateState.position);
    }
}
