package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.DashboardTuningCommand;
import org.firstinspires.ftc.teamcode.subsystems.tuning.DashboardTuner;

/** Select this OpMode; edit DashboardTuner fields in FTC Dashboard before INIT. */
@TeleOp(name = "DashTuner", group = "Test")
public class DashTuner extends CommandOpMode {
    private DashboardTuner tuner;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        tuner = new DashboardTuner(hardwareMap);
        tuner.setDefaultCommand(new DashboardTuningCommand(tuner));
        telemetry.addLine("Tune DashboardTuner values in FTC Dashboard.");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try { super.runOpMode(); }
        finally {
            CommandScheduler.getInstance().reset();
            if (tuner != null) tuner.stop();
        }
    }
}
