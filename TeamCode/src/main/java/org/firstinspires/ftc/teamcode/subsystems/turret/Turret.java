package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/** Manual first-stage turret control; future closed-loop aiming belongs here. */
public class Turret extends SubsystemBase {
    private final DcMotor turretMotor;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, TurretConstants.turretMotorName);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stop();
    }

    public void rotate(double power) {
        double output = Range.clip(power, -1.0, 1.0);
        if (TurretConstants.useSoftLimits) {
            int currentTicks = getTurretPos();
            if ((currentTicks <= TurretConstants.minTicks && output < 0)
                    || (currentTicks >= TurretConstants.maxTicks && output > 0)) {
                output = 0;
            }
        }
        turretMotor.setPower(output);
    }

    public int getTurretPos() {
        return turretMotor.getCurrentPosition();
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}
