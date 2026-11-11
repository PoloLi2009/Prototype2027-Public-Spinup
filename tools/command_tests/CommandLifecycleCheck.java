import com.arcrobotics.ftclib.command.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit.GateState;
import java.util.Random;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.opmodes.auto.RouteFixture;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class CommandLifecycleCheck {
    private static class Rig {
        final Intake intake = new Intake();
        final Shooter shooter = new Shooter();
        final Transit transit = new Transit();
    }

    private static void require(boolean condition, String message) {
        if (!condition) throw new AssertionError(message + " at " + ElapsedTime.now);
    }

    private static void step(CommandScheduler scheduler, long ms) {
        ElapsedTime.now = ms;
        scheduler.run();
    }

    private static void safe(Rig rig) {
        require(rig.shooter.output == 0 && rig.intake.output == IntakeState.STOP
                && rig.transit.output == GateState.CLOSED, "physical outputs stopped");
    }

    private static void autonomous() {
        CommandScheduler scheduler = CommandScheduler.getInstance(); scheduler.reset();
        scheduler = CommandScheduler.getInstance();
        ElapsedTime.now = 0;
        Rig rig = new Rig(); boolean[] aimed = {true};
        ShootCommand shot = new ShootCommand(rig.intake, rig.shooter,
                rig.transit, 3000, () -> aimed[0], 400, 4000);
        require(shot.getRequirements().size() == 3, "only three mechanism requirements");
        scheduler.schedule(shot); safe(rig); rig.shooter.ready = true;
        step(scheduler, 0); step(scheduler, 249);
        require(rig.transit.output == GateState.CLOSED, "250 ms speed hold");
        step(scheduler, 250);
        require(rig.transit.output == GateState.OPEN && rig.intake.output == IntakeState.STOP, "open first");
        step(scheduler, 749); require(rig.intake.output == IntakeState.STOP, "500 ms gate delay");
        step(scheduler, 750); require(rig.intake.output == IntakeState.FEED, "automatic feed");
        rig.shooter.ready = false; step(scheduler, 900);
        require(rig.transit.output == GateState.CLOSED && rig.intake.output == IntakeState.STOP, "speed drop immediate stop");
        rig.shooter.ready = true; step(scheduler, 1000); step(scheduler, 1150); step(scheduler, 1650);
        require(rig.intake.output == IntakeState.FEED, "feed restarts after requalification");
        step(scheduler, 2049); require(scheduler.isScheduled(shot), "400 ms uninterrupted feed");
        step(scheduler, 2050); require(!scheduler.isScheduled(shot), "natural finish"); safe(rig);
        scheduler.schedule(shot); aimed[0] = false;
        step(scheduler, 2050); step(scheduler, 6049);
        require(rig.transit.output == GateState.CLOSED && rig.intake.output == IntakeState.STOP, "no aim no feed");
        step(scheduler, 6050); require(!scheduler.isScheduled(shot), "timeout"); safe(rig);
        aimed[0] = true; scheduler.schedule(shot);
        step(scheduler, 6050); step(scheduler, 6300); step(scheduler, 6800);
        require(rig.intake.output == IntakeState.FEED, "reschedule after timeout");
        scheduler.cancel(shot); safe(rig);
        System.out.println("PASS: auto scheduler, gate and feed timing, speed loss, timeout, reschedule, and immediate cleanup");
    }

    private static void commands() {
        CommandScheduler.getInstance().reset();
        CommandScheduler scheduler = CommandScheduler.getInstance();
        ElapsedTime.now = 0;
        Rig rig = new Rig();
        IntakeCommand collect = new IntakeCommand(rig.intake, rig.transit, IntakeState.FAST_FORWARD);
        collect.schedule(); scheduler.run();
        require(rig.intake.state == IntakeState.FAST_FORWARD, "shared intake preset");
        ReverseIntakeCommand reverse = new ReverseIntakeCommand(rig.intake, rig.transit);
        reverse.schedule(); scheduler.run();
        require(!scheduler.isScheduled(collect) && rig.intake.state == IntakeState.REVERSED,
                "requirement conflict replaces intake");
        reverse.cancel(); scheduler.run();
        require(rig.intake.output == IntakeState.STOP, "intake cancellation");
        double[] rpm = {2500};
        FlywheelRpmCommand spin = new FlywheelRpmCommand(rig.shooter, () -> rpm[0]);
        spin.schedule(); scheduler.run(); require(rig.shooter.rpm == 2500, "velocity action");
        rpm[0] = 2700; scheduler.run(); require(rig.shooter.rpm == 2700, "live RPM supplier");
        spin.cancel(); scheduler.run(); require(rig.shooter.output == 0, "velocity cancel");
        new SetPitchCommand(rig.shooter, Shooter.PitchState.HIGH).schedule(); scheduler.run();
        require(rig.shooter.pitchState == Shooter.PitchState.HIGH, "autonomous pitch preset");
        new SetPitchCommand(rig.shooter, .01).schedule(); scheduler.run();
        require(Math.abs(rig.shooter.pitch - .51) < 1e-9, "relative pitch adjustment");
        GateCommand gate = new GateCommand(rig.transit, GateState.OPEN);
        gate.schedule(); scheduler.run(); scheduler.run(); require(rig.transit.output == GateState.OPEN,"gate opens");
        gate.cancel(); scheduler.run(); require(rig.transit.output == GateState.CLOSED,"gate closes on cancel");
        RunIntakeCommand motor = new RunIntakeCommand(rig.intake, IntakeState.FEED);
        motor.schedule(); scheduler.run(); motor.cancel(); scheduler.run();
        require(rig.intake.output == IntakeState.STOP,"feed motor cleanup");

        Drive drive = new Drive();
        double[] heading = {0};
        DriveWithAimCommand motion = new DriveWithAimCommand(drive, () -> -1, () -> 0,
                () -> 0, () -> 1, () -> true, () -> heading[0], turn -> turn);
        motion.schedule(); scheduler.run();
        require(drive.forward == -1 && drive.strafe == 0,"robot frame motion");
        heading[0] = 90; scheduler.run();
        require(Math.abs(drive.forward) < 1e-9 && drive.strafe == 1,"test-compatible field transform");
        motion.cancel(); require(drive.forward == 0 && drive.strafe == 0,"drive cancel immediately");
        Path[] paths = RouteFixture.paths();
        var pedro=new org.firstinspires.ftc.teamcode.subsystems.drive.PedroDrive();
        FollowPathCommand follow = new FollowPathCommand(pedro, paths);
        try { new ParallelCommandGroup(follow, new FollowPathCommand(pedro, paths)); throw new AssertionError("accepted two Pedro writers"); }
        catch (IllegalArgumentException expected) { }
        follow = new FollowPathCommand(pedro, paths);
        follow.schedule(); scheduler.run(); require(scheduler.isScheduled(follow)&&pedro.current==paths[0],"first path handed to Pedro");
        for (int i=0; i<paths.length; i++) {
            pedro.busy=false; scheduler.run();
            if (i+1<paths.length) require(follow.segmentNumber()==i+2&&pedro.current==paths[i+1],"isBusy completion advances exactly one path");
        }
        require(!scheduler.isScheduled(follow)&&!follow.hasLostPose()&&!pedro.busy,"eight paths finish and stop");
        follow.schedule();pedro.ready=false;scheduler.run();
        require(!scheduler.isScheduled(follow)&&follow.hasLostPose()&&!pedro.busy,"lost localization stops path");
        follow.schedule();scheduler.run();require(!scheduler.isScheduled(follow)&&!pedro.busy,"cannot start without localization");
        pedro.ready=true; follow.schedule(); scheduler.run(); follow.cancel();
        require(!pedro.busy,"path interruption stops");
        ElapsedTime.now=100_000;
        follow.schedule();scheduler.run();
        step(scheduler,104_999);
        require(follow.segmentNumber()==1&&follow.getTimedOutSegments()==0,"first segment gets full 5 seconds after INIT");
        int stops=pedro.stops;
        step(scheduler,105_000);
        require(follow.segmentNumber()==2&&follow.getTimedOutSegments()==1&&pedro.stops==stops+1,"timeout stops old path before starting next");
        step(scheduler,109_999);require(follow.segmentNumber()==2,"next segment gets fresh timer");
        for(int i=2;i<=paths.length;i++)step(scheduler,100_000+5000L*i);
        require(!scheduler.isScheduled(follow)&&follow.getTimedOutSegments()==paths.length&&!pedro.busy,"eight timeouts finish and stop");
        follow.schedule();require(follow.segmentNumber()==1&&follow.getTimedOutSegments()==0,"reschedule clears timeout state");follow.cancel();
        ElapsedTime.now=200_000;
        follow=new FollowPathCommand(pedro,5000,new int[]{1,4,7},paths);
        follow.schedule();pedro.busy=false;scheduler.run();
        require(follow.getReservedShotEndpoint()==1&&!pedro.busy,"arrival stops drive for reserved shot");
        step(scheduler,204_999);require(follow.segmentNumber()==1&&follow.getReservedShotRemainingMs()==1,"full reserved duration");
        step(scheduler,205_000);require(follow.segmentNumber()==2&&pedro.busy,"reserved stop advances at five seconds");
        step(scheduler,209_999);require(follow.getTimedOutSegments()==0,"pause does not consume next path budget");
        follow.cancel();require(!pedro.busy&&follow.getReservedShotEndpoint()==0,"cancel clears reserved state");
        follow.schedule();pedro.busy=false;scheduler.run();pedro.ready=false;scheduler.run();
        require(!scheduler.isScheduled(follow)&&follow.hasLostPose()&&!pedro.busy,"pose loss during reserved stop cancels safely");
        pedro.ready=true;follow.schedule();pedro.busy=false;scheduler.run();follow.cancel();
        require(!pedro.busy&&follow.getReservedShotEndpoint()==0,"STOP during pause immediately clears output");
        System.out.println("PASS: actions, ownership, path lifecycle, independent 5-second timeouts and cleanup");
    }

    public static void main(String[] args) { autonomous(); commands(); }
}
