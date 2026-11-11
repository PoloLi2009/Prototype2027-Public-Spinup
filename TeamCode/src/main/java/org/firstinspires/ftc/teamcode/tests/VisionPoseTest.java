package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.VisionReadCommand;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

/** Diagnostics for vision conversion and two parallel estimators; no automatic aiming. */
@TeleOp(name = "Vision Pose Test", group = "Test")
public class VisionPoseTest extends CommandOpMode {
    private LimelightVision vision;
    private Pinpoint pinpoint;
    private Turret turret;
    private VisionLocalization localization;
    @Override public void initialize() {
        CommandScheduler.getInstance().reset();
        pinpoint=new Pinpoint(hardwareMap);
        turret=new Turret(hardwareMap);
        vision=new LimelightVision(hardwareMap);
        localization=new VisionLocalization();
        CommandScheduler.getInstance().registerSubsystem(pinpoint,localization);
        vision.setDefaultCommand(new VisionReadCommand(vision,pinpoint,turret,localization,gamepad1));
        telemetry.addLine("Turret FORWARD before INIT. Keep chassis level for HIVE correction.");
        telemetry.addLine("Y: BLUE pipeline 0 / RED pipeline 1; both use supplied flat-HIVE maps.");
        telemetry.addLine("Camera web pose: 0.117 / 0 / 0.278 m; 180 / 50 / 0 deg.");
        telemetry.addLine("Pose diagnostic only; drive and turret are not commanded to move.");
        telemetry.update();
    }
    @Override public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("Vision",vision.getStatus());
        telemetry.addData("Fusion",localization.getStatus());
        telemetry.addData("Alliance / pipeline","%s / %d",
                vision.isBlueAlliance()?"BLUE":"RED",vision.getExpectedPipeline());
        telemetry.addData("Field anchored",localization.hasFieldPose());
        telemetry.addData("OD local X/Y m, yaw deg",format(localization.getOdometry()));
        telemetry.addData("Corrected vision field",format(localization.getCorrectedVision()));
        telemetry.addData("WPI port field",format(localization.getWpiPosition()));
        telemetry.addData("EKF field",format(localization.getEstimatedPosition()));
        telemetry.addData("HIVE tilt deg","%.2f",localization.getHiveTiltDegrees());
        telemetry.addData("Turret now / at capture deg","%.2f / %.2f",
                turret.getAngleDegrees(),localization.getTurretAtCaptureDegrees());
        double[] std=localization.getStandardDeviations();
        telemetry.addData("EKF sigma m/m/deg","%.3f / %.3f / %.2f",std[0],std[1],Math.toDegrees(std[2]));
        telemetry.addData("EKF innovation NIS",localization.getLastInnovationNis());
        telemetry.addData("Accepted / rejected","%d / %d",localization.getAcceptedCount(),localization.getRejectedCount());
        telemetry.addData("Last accepted vision age s",localization.getLastVisionAgeSeconds());
        telemetry.update();
    }
    private String format(Pose2d p) {
        return p==null?"UNAVAILABLE":String.format("%.3f / %.3f / %.1f",p.getX(),p.getY(),p.getRotation().getDegrees());
    }
    @Override public void runOpMode() throws InterruptedException {
        try {super.runOpMode();}
        finally {
            CommandScheduler.getInstance().reset();
            if(vision!=null)vision.stop();
            if(turret!=null)turret.stop();
        }
    }
}
