package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.TurretStartupHoldCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@Autonomous(name = "BIOBUZZ: Spinup AutoFormal", group = "Autonomous")
public class SpinupAutoFormal extends LinearOpMode {
    private static final long RESERVED_SHOT_MS = 5000;
    private static final int[] RESERVED_SHOT_ENDPOINTS = {1, 4, 7};

    @Override public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();
        PedroDrive drive = null;
        Turret turret = null;
        TurretStartupHoldCommand turretHold = null;
        FollowPathCommand routine = null;
        try {
            if (Constants.foresightConfig == null) {
                showResult("Run AutoTune > Foresight Tuner; paste Spinup's export into pedro/Constants.java");
                waitForStart();
                return;
            }
            drive = new PedroDrive(hardwareMap);
            turret = new Turret(hardwareMap);
            turretHold = new TurretStartupHoldCommand(turret);
            Path[] paths = AutoFormalRoute.paths();
            drive.setPose(AutoFormalRoute.START);
            telemetry.setMsTransmissionInterval(100);
            while (opModeInInit() && !isStopRequested()) {
                drive.periodic();
                telemetry.addData("Pinpoint", drive.isReady() ? "READY" : "Waiting");
                telemetry.update();
                sleep(40);
            }
            waitForStart();
            if (isStopRequested()) return;
            if (!drive.isReady()) { showResult("Stopped: Pinpoint not ready"); return; }
            drive.setPose(AutoFormalRoute.START);
            routine = new FollowPathCommand(drive, RESERVED_SHOT_MS, RESERVED_SHOT_ENDPOINTS, paths);
            scheduler.schedule(turretHold, routine);
            // No total time limit: the Driver Station / match system ends autonomous.
            while (opModeIsActive() && scheduler.isScheduled(routine)) {
                scheduler.run();
                telemetry.addData("Reserved shot endpoint / remaining ms", "%d / %d",
                        routine.getReservedShotEndpoint(), routine.getReservedShotRemainingMs());
                Pose pose = drive.pose();
                telemetry.addData("Route segment", "%d / %d", routine.segmentNumber(), paths.length);
                telemetry.addData("Timed-out segments / limit ms", "%d / %d",
                        routine.getTimedOutSegments(), FollowPathCommand.SEGMENT_TIMEOUT_MS);
                if (pose != null) telemetry.addData("Pose x/y in, heading deg", "%.2f / %.2f / %.1f",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.update();
                sleep(20);
            }
            showResult(routine.hasLostPose() ? "Stopped: Pinpoint pose lost"
                    : isStopRequested() ? "Stopped by driver"
                    : "Route ended; timed-out segments: " + routine.getTimedOutSegments());
        } finally {
            // FTCLib 2.1.1 cancelAll mutates its live key set when multiple commands run.
            // Cancel our two owned commands individually; always attempt both cleanups.
            try {
                try { if (routine != null) scheduler.cancel(routine); }
                finally { if (turretHold != null) scheduler.cancel(turretHold); }
            }
            finally {
                try { if (drive != null) drive.stop(); }
                finally {
                    try { if (turret != null) turret.shutdown(); }
                    finally { scheduler.reset(); }
                }
            }
        }
    }
    private void showResult(String message) {
        telemetry.addData("AutoFormal", message);
        telemetry.update();
    }
}
