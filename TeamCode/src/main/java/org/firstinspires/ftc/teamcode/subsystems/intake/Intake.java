package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** The one motor which both collects Pollen and advances it toward the shooter. */
public class Intake extends SubsystemBase {
    private final DcMotor intakeMotor;
    private IntakeState intakeState = IntakeState.STOP;

    public enum IntakeState {
        STOP(0),
        FORWARD(IntakeConstants.intakePower),
        FEED(IntakeConstants.feedPower),
        REVERSED(IntakeConstants.reversePower);

        final double power;

        IntakeState(double power) {
            this.power = power;
        }
    }

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.intakeMotorName);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    public void setIntakeState(IntakeState intakeState) {
        
        this.intakeState = intakeState;
    }

    public IntakeState getIntakeState() {

        return intakeState;
    }

    public void stop() {

        setIntakeState(IntakeState.STOP);
    }

    @Override
    public void periodic() {
        intakeMotor.setPower(intakeState.power);
    }
}
