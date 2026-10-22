package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Controls ball release into the rotating shooter.  The gate blocks early entry;
 * the ramp works with the mechanical limit rod to prevent an unintended feed.
 */
public class Transit extends SubsystemBase {
    private final Servo gateServo;
    private final Servo rampServo;
    private GateState gateState;
    private RampState rampState;

    public enum GateState {
        CLOSED(TransitConstants.gateClosedPosition),
        OPEN(TransitConstants.gateOpenPosition);

        final double position;

        GateState(double position) {
            this.position = position;
        }
    }

    public enum RampState {
        UP(TransitConstants.rampUpPosition),
        DOWN(TransitConstants.rampDownPosition);

        final double position;

        RampState(double position) {
            this.position = position;
        }
    }

    public Transit(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, TransitConstants.gateServoName);
        rampServo = hardwareMap.get(Servo.class, TransitConstants.rampServoName);
        setGateState(GateState.CLOSED);
        setRampState(RampState.DOWN);
    }

    public void setGateState(GateState gateState) {
        this.gateState = gateState;
    }

    public void setRampState(RampState rampState) {
        this.rampState = rampState;
    }

    public GateState getGateState() {
        return gateState;
    }

    public RampState getRampState() {
        return rampState;
    }

    @Override
    public void periodic() {
        gateServo.setPosition(gateState.position);
        rampServo.setPosition(rampState.position);
    }
}
