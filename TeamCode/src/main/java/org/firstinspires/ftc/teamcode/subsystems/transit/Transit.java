package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/** Controls ball release into the rotating shooter with the entry blocking rod. */
public class Transit extends SubsystemBase {
    private final Servo gateServo;
    private GateState gateState;
    private Double manualPosition;

    public enum GateState {
        CLOSED(TransitConstants.gateClosedPosition),
        OPEN(TransitConstants.gateOpenPosition);

        final double position;

        GateState(double position) {
            this.position = position;
        }
    }

    public Transit(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, TransitConstants.limitServoName);
        setGateState(GateState.CLOSED);
    }

    public void setGateState(GateState gateState) {
        this.gateState = gateState;
        manualPosition = null;
    }

    /** Direct position for the isolated servo trim test. */
    public void setManualPosition(double position) {
        manualPosition = Range.clip(position, 0.0, 1.0);
    }

    public double getPosition() {
        return manualPosition != null ? manualPosition : gateState.position;
    }

    public GateState getGateState() {
        return gateState;
    }

    @Override
    public void periodic() {
        gateServo.setPosition(getPosition());
    }
}
