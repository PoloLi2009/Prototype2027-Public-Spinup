package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

/** Runs one short motor pulse per press for electrical troubleshooting. */
public class AllMotorTestCommand extends CommandBase {
    private enum MotorGroup { DRIVE, TURRET, FLYWHEELS, TRANSIT, ALL }
    private static final MotorGroup[] SELECTABLE_GROUPS = {
            MotorGroup.DRIVE, MotorGroup.TURRET, MotorGroup.FLYWHEELS, MotorGroup.TRANSIT
    };
    private static final double MOTOR_POWER = 0.30;
    private static final double TURRET_POWER = 0.15;
    private static final double PULSE_SECONDS = 0.50;

    private final Drive drive;
    private final Turret turret;
    private final DcMotor leftFlywheel;
    private final DcMotor rightFlywheel;
    private final DcMotor transit;
    private final Gamepad gamepad;
    private final ElapsedTime pulseTimer = new ElapsedTime();
    private int selectedGroup;
    private MotorGroup activeGroup;
    private boolean lastLeft;
    private boolean lastRight;
    private boolean releaseRequired;

    public AllMotorTestCommand(Drive drive, Turret turret, DcMotor leftFlywheel,
                               DcMotor rightFlywheel, DcMotor transit, Gamepad gamepad) {
        this.drive = drive;
        this.turret = turret;
        this.leftFlywheel = leftFlywheel;
        this.rightFlywheel = rightFlywheel;
        this.transit = transit;
        this.gamepad = gamepad;
        addRequirements(drive, turret);
    }

    @Override
    public void initialize() {
        // Require a fresh press if a test control was held at START.
        releaseRequired = true;
    }

    @Override
    public void execute() {
        int step = (gamepad.dpad_right && !lastRight ? 1 : 0)
                - (gamepad.dpad_left && !lastLeft ? 1 : 0);
        selectedGroup = (selectedGroup + step + SELECTABLE_GROUPS.length) % SELECTABLE_GROUPS.length;
        lastLeft = gamepad.dpad_left;
        lastRight = gamepad.dpad_right;

        boolean leftTrigger = gamepad.left_trigger >= 0.5;
        boolean rightTrigger = gamepad.right_trigger >= 0.5;
        boolean allHeld = leftTrigger && rightTrigger;
        if (!gamepad.a && !leftTrigger && !rightTrigger) {
            releaseRequired = false;
        }
        if (!releaseRequired && (allHeld || gamepad.a)) {
            activeGroup = allHeld ? MotorGroup.ALL : SELECTABLE_GROUPS[selectedGroup];
            pulseTimer.reset();
            releaseRequired = true;
        }
        if (activeGroup != null) {
            boolean stillHeld = activeGroup == MotorGroup.ALL ? allHeld : gamepad.a;
            if (!stillHeld || pulseTimer.seconds() >= PULSE_SECONDS) activeGroup = null;
        }
        applyMotorPower();
    }

    private void applyMotorPower() {
        boolean all = activeGroup == MotorGroup.ALL;
        if (all || activeGroup == MotorGroup.DRIVE) drive.moveRobot(-MOTOR_POWER, 0, 0);
        else drive.stop();
        if (all || activeGroup == MotorGroup.TURRET) turret.rotate(TURRET_POWER);
        else turret.stop();

        double flywheelPower = all || activeGroup == MotorGroup.FLYWHEELS ? MOTOR_POWER : 0.0;
        leftFlywheel.setPower(flywheelPower);
        rightFlywheel.setPower(flywheelPower);
        transit.setPower(all || activeGroup == MotorGroup.TRANSIT ? MOTOR_POWER : 0.0);
    }

    public String getSelectedGroup() { return SELECTABLE_GROUPS[selectedGroup].name(); }
    public String getActiveGroup() { return activeGroup == null ? "STOPPED" : activeGroup.name(); }

    @Override
    public void end(boolean interrupted) {
        activeGroup = null;
        applyMotorPower();
    }
}
