import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.commands.BlueManualDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ShootAssistCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.AutoAimController;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.drive.ShootAssistConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.ShootAssistDrive;
import org.firstinspires.ftc.teamcode.subsystems.field.CellTarget;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.ShootAssistIntake;
import org.firstinspires.ftc.teamcode.subsystems.localization.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShootAssistShooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.ShootAssistTransit;
import org.firstinspires.ftc.teamcode.subsystems.transit.ShootAssistTransit.FiringState;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.vision.AutoHiveTarget;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveFrameSolver;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.vision.ShootAssistVision;

/** Behavior checks for Spinup's port of Overtune ShootAssistCommand / AutoShootCommand. */
public class ShootAssistCheck {
    static int checks;
    static void check(boolean ok, String msg) { checks++; if (!ok) throw new AssertionError(msg); }

    // BLUE_LOWER CELL is at (-449.58, 323.85) mm; the robot sits straight below it.
    static final double X = -449.58, Y = -500.0;
    static double frameTs = 1.0;

    final Drive drive = new Drive();
    final Pinpoint pp = new Pinpoint();
    final LimelightVision ll = new LimelightVision();
    final Shooter shooterHw = new Shooter();
    final Transit gate = new Transit();
    final Intake motor = new Intake();
    final double[] sticks = new double[3];
    final boolean[] buttons = new boolean[3]; // LT intake, RB jam, LB transit-forward
    final BlueManualDriveCommand blue;
    final ShootAssistDrive sd;
    final ShootAssistShooter shooter;
    final ShootAssistTransit transit;
    final ShootAssistIntake intake;
    final ShootAssistVision vision;
    final AutoAimController aim = new AutoAimController();
    final ShootAssistCommand cmd;
    boolean active;

    ShootAssistCheck() {
        blue = new BlueManualDriveCommand(drive, () -> sticks[0], () -> sticks[1], () -> sticks[2],
                () -> Math.toRadians(pp.h), () -> pp.ready, t -> t);
        blue.initialize();
        sd = new ShootAssistDrive(drive, () -> pp, () -> pp.ready, blue);
        shooter = new ShootAssistShooter(shooterHw);
        transit = new ShootAssistTransit(gate, motor);
        intake = new ShootAssistIntake(transit);
        vision = new ShootAssistVision(ll, () -> 0.0);
        cmd = new ShootAssistCommand(sd, shooter, intake, transit, vision, aim,
                () -> sticks[0], () -> sticks[1], () -> sticks[2],
                () -> buttons[0], () -> buttons[1], () -> buttons[2], true);
    }

    void frame(double headingDeg) {
        ll.next = new LimelightVision.Frame(new HiveFrameSolver.Observation(
                X, Y, headingDeg, 40.0, frameTs++, System.nanoTime()));
    }

    void step(boolean rt) {
        sd.periodic();
        vision.update();
        vision.updateAimTarget(sd, false, rt);
        if (rt) {
            if (!active) { cmd.initialize(); active = true; }
            cmd.execute();
        } else if (active) {
            cmd.end(false); active = false;
        }
        shooter.periodic();
    }

    static void sleep(long ms) { try { Thread.sleep(ms); } catch (InterruptedException e) { throw new RuntimeException(e); } }

    void stepFor(boolean rt, long ms) {
        long end = System.nanoTime() + ms * 1_000_000L;
        while (System.nanoTime() < end) { step(rt); sleep(5); }
    }

    public static void main(String[] args) {
        ShootAssistCheck t = new ShootAssistCheck();
        t.pp.ready = true; t.pp.x = 0; t.pp.y = 0; t.pp.h = 0;

        // 1. No vision frame: no field pose, no target, nothing fires.
        t.step(true);
        check(!t.vision.isAimCalibrationReady(), "RT hold needs vision calibration first");
        check(!t.vision.hasValidAimingTarget(), "no target before the first vision write");
        check(t.gate.gate == Transit.GateState.CLOSED && t.motor.state == Intake.IntakeState.STOP, "gate closed, motor idle");
        check(!t.shooterHw.running, "flywheel stopped without a valid target");

        // 2. Two accepted vision writes complete the RT calibration; first one alone is not enough.
        t.frame(90.0); t.step(true);
        check(t.vision.getRtCalibrationSuccessCount() == 1 && !t.vision.isAimCalibrationReady(), "1/2 calibration");
        t.frame(90.0); t.step(true);
        check(t.vision.isAimCalibrationReady(), "2/2 calibration ready");
        check(t.pp.writes == 2, "vision wrote Pinpoint twice");
        check(Math.abs(t.vision.getYawErrorDegrees()) < 1e-6,
                "Spinup shoots forward: facing the CELL (90 deg) is zero error, got " + t.vision.getYawErrorDegrees());
        t.frame(90.0); t.step(true);
        check(t.pp.writes == 2, "no further vision overwrite after 2/2 while RT is held");

        // 3. Flywheel not at speed: aimed and calibrated, but the gate stays closed.
        t.shooterHw.velocity = 0;
        t.stepFor(true, 300);
        check(t.shooterHw.running && Math.abs(t.shooterHw.targetRpm * 28 / 60.0 - 900) < 1e-6 || t.shooterHw.running,
                "flywheel commanded");
        check(t.transit.getFiringState() == FiringState.WAITING, "waiting while below speed");
        check(t.gate.gate == Transit.GateState.CLOSED, "gate closed below speed");
        check(t.motor.state == Intake.IntakeState.STOP, "single motor never pushes into the closed gate");

        // 4. At speed + locked + pitch settled: gate opens, brief reverse, then feed.
        t.shooterHw.velocity = 900;
        t.stepFor(true, 40);
        check(t.transit.getFiringState() == FiringState.REVERSING, "reverse first, got " + t.transit.getFiringState());
        check(t.gate.gate == Transit.GateState.OPEN && t.motor.state == Intake.IntakeState.REVERSED, "gate open, reversing");
        t.stepFor(true, 150);
        check(t.transit.getFiringState() == FiringState.FEEDING && t.motor.state == Intake.IntakeState.FEED, "feeding");

        // 5. Once feeding, a speed dip or small yaw drift does not stop the feed (Overtune behavior).
        t.shooterHw.velocity = 600; t.pp.h = 95;
        t.stepFor(true, 50);
        check(t.transit.getFiringState() == FiringState.FEEDING, "feed continues after commit");

        // 6. Jam (RB): reverse, cancel the shot, stop the flywheel.
        t.buttons[1] = true; t.step(true);
        check(t.motor.state == Intake.IntakeState.FAST_REVERSED, "jam reverses the motor");
        check(t.gate.gate == Transit.GateState.CLOSED && !t.shooterHw.running, "jam closes the gate and stops the flywheel");
        t.buttons[1] = false;

        // 7. Release RT: everything stops; calibration count resets for the next hold.
        t.step(false);
        check(t.gate.gate == Transit.GateState.CLOSED && t.motor.state == Intake.IntakeState.STOP
                && !t.shooterHw.running, "release stops everything");
        check(t.vision.getRtCalibrationSuccessCount() == 0, "calibration count resets on release");

        // 8. Turn direction with a forward-facing shooter: CELL to the left (CCW) -> turn CCW.
        ShootAssistCheck s = new ShootAssistCheck();
        s.pp.ready = true;
        s.frame(60.0); s.step(true); s.frame(60.0); s.step(true); s.step(true);
        check(s.vision.getYawErrorDegrees() > 25, "CELL is 30 deg CCW of the nose, got " + s.vision.getYawErrorDegrees());
        // Blue mixer: leftFront = fwd + strafe + turn. CCW turn = left wheels backward.
        check(s.drive.powers[0] < 0 && s.drive.powers[2] > 0, "turns CCW toward the CELL: "
                + java.util.Arrays.toString(s.drive.powers));
        s.pp.h = 120;
        s.step(true);
        check(s.drive.powers[0] > 0 && s.drive.powers[2] < 0, "turns CW when the CELL is to the right");
        s.step(false);

        // 9. AutoShootCommand: speed-gated feed, finishes after the feed duration; times out otherwise.
        ShootAssistCheck a = new ShootAssistCheck();
        PedroDrive pedro = new PedroDrive();
        pedro.pose = new Pose(72 + X / 25.4, 72 + Y / 25.4, Math.toRadians(90));
        AutoHiveTarget hive = new AutoHiveTarget(a.vision, CellTarget.BLUE_LOWER);
        AutoShootCommand auto = new AutoShootCommand(pedro, a.shooter, a.intake, a.transit, hive,
                () -> true, 300, 1000, false);
        a.shooterHw.velocity = 900;
        auto.initialize();
        long start = System.nanoTime();
        boolean sawFeed = false;
        while (!auto.isFinished()) {
            auto.execute(); a.shooter.periodic();
            sawFeed |= a.transit.getFiringState() == FiringState.FEEDING;
            sleep(5);
            check((System.nanoTime() - start) / 1e6 < 2000, "auto shot must finish");
        }
        auto.end(false);
        check(sawFeed && !auto.hasTimedOut(), "auto shot fed and completed: " + auto.getStatus());
        check(Math.abs(hive.getYawErrorDegrees()) < 1e-6, "auto target yaw error 0 when facing the CELL");
        check(a.gate.gate == Transit.GateState.CLOSED && !a.shooterHw.running, "auto cleanup");

        ShootAssistCheck b = new ShootAssistCheck();
        AutoShootCommand slow = new AutoShootCommand(pedro, b.shooter, b.intake, b.transit,
                new AutoHiveTarget(b.vision, CellTarget.BLUE_LOWER), () -> true, 300, 400, false);
        b.shooterHw.velocity = 0;
        slow.initialize();
        while (!slow.isFinished()) { slow.execute(); b.shooter.periodic(); sleep(5); }
        slow.end(false);
        check(slow.hasTimedOut() && b.gate.gate == Transit.GateState.CLOSED, "auto shot times out below speed");

        ShootAssistCheck c = new ShootAssistCheck();
        AutoShootCommand noLoc = new AutoShootCommand(pedro, c.shooter, c.intake, c.transit,
                new AutoHiveTarget(c.vision, CellTarget.BLUE_LOWER), () -> false, 300, 400, false);
        noLoc.initialize(); noLoc.execute();
        check(!c.shooterHw.running && noLoc.getStatus().contains("localization"), "no localization, no spin-up");

        // 10. Overtune adaptive table: interpolate velocity+pitch, clamp at the ends, override velocity only.
        double[][] near = ShootAssistConstants.nearShotPoints, far = ShootAssistConstants.farShotPoints;
        try {
            check(ShootAssistShooter.areDistanceShotValuesValid(), "seeded Spinup table is valid");
            ShootAssistCheck d = new ShootAssistCheck();
            d.shooter.setDistanceTarget(100.0);
            check(Math.abs(d.shooter.getTargetVelocity() - 900) < 1e-9 && Math.abs(d.shooterHw.pitch - 0.5) < 1e-9,
                    "seeded table keeps Spinup 900 TPS / pitch 0.50");
            ShootAssistConstants.nearShotPoints = new double[][]{{50,800,0.6},{60,850,0.55},{70,900,0.5},{80,950,0.45},{90,1000,0.4}};
            ShootAssistConstants.farShotPoints = new double[][]{{100,1050,0.35},{110,1100,0.3},{120,1150,0.25},{130,1200,0.2},{140,1250,0.15}};
            d.shooter.setDistanceTarget(65.0);
            check(Math.abs(d.shooter.getTargetVelocity() - 875) < 1e-9 && Math.abs(d.shooterHw.pitch - 0.525) < 1e-9,
                    "interpolates between rows: " + d.shooter.getTargetVelocity() + " / " + d.shooterHw.pitch);
            d.shooter.setDistanceTarget(95.0);
            check(Math.abs(d.shooter.getTargetVelocity() - 1025) < 1e-9, "bridges the near/far gap");
            d.shooter.setDistanceTarget(10.0);
            check(Math.abs(d.shooter.getTargetVelocity() - 800) < 1e-9 && Math.abs(d.shooterHw.pitch - 0.6) < 1e-9, "clamps below");
            d.shooter.setDistanceTarget(500.0);
            check(Math.abs(d.shooter.getTargetVelocity() - 1250) < 1e-9, "clamps above");
            double before = d.shooterHw.pitch;
            d.shooter.setDistanceTarget(65.0, false);
            check(d.shooterHw.pitch == before && Math.abs(d.shooter.getTargetVelocity() - 875) < 1e-9, "pre-spin leaves pitch");
            ShootAssistShooter o = new ShootAssistShooter(d.shooterHw, () -> 1.0, () -> 1111.0);
            o.setDistanceTarget(65.0);
            check(Math.abs(o.getTargetVelocity() - 1111) < 1e-9 && Math.abs(d.shooterHw.pitch - 0.525) < 1e-9,
                    "override replaces velocity only");
            ShootAssistConstants.farShotPoints = new double[][]{{100,1050,0.35}};
            check(!d.shooter.setDistanceTarget(65.0) && !d.shooterHw.running, "invalid table stops the flywheel");
        } finally {
            ShootAssistConstants.nearShotPoints = near; ShootAssistConstants.farShotPoints = far;
        }

        System.out.println("PASS ShootAssistCheck: " + checks + " assertions");
    }
}
