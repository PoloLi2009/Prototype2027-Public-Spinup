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

public class ManualCompatibilityCheck {
    private static class Rig {
        final Intake intake = new Intake();
        final Shooter shooter = new Shooter();
        final Transit transit = new Transit();
    }

    private static void require(boolean condition, String message) {
        if (!condition) throw new AssertionError(message + " at " + ElapsedTime.now);
    }

    private static void manual(boolean solo) {
        CommandScheduler.getInstance().reset();
        ElapsedTime.now = 0;
        Gamepad driver = new Gamepad(), pitch = solo ? new Gamepad() : null;
        Rig old = new Rig(), current = new Rig();
        boolean[] aimed = {true};
        BaselineSequencedShotCommand original = new BaselineSequencedShotCommand(old.intake,
            old.shooter, old.transit, driver, 3000, () -> aimed[0], IntakeState.FAST_FORWARD,
            IntakeState.FAST_REVERSED, 100, () -> driver.a && driver.right_trigger < .5, true, pitch);
        SequencedShotCommand updated = new SequencedShotCommand(current.intake,
            current.shooter, current.transit, driver, 3000, () -> aimed[0], IntakeState.FAST_FORWARD,
            IntakeState.FAST_REVERSED, 100, () -> driver.a && driver.right_trigger < .5, true, pitch);
        Random input = new Random(16095);
        updated.initialize();
        for (int tick = 0; tick < 3000; tick++) {
            ElapsedTime.now += 20;
            driver.right_trigger = tick % 200 < 160 ? 1 : 0;
            driver.left_trigger = input.nextInt(20) == 0 ? 1 : 0;
            driver.left_bumper = input.nextInt(100) == 0;
            driver.a = input.nextInt(20) == 0;
            driver.dpad_up = input.nextInt(100) == 0;
            driver.dpad_down = input.nextInt(100) == 0;
            driver.dpad_left = input.nextInt(150) == 0;
            driver.dpad_right = input.nextInt(100) == 0;
            if (solo) { pitch.dpad_up = tick % 60 < 5; pitch.dpad_down = tick % 60 >= 55; }
            aimed[0] = tick % 120 >= 5;
            old.shooter.ready = current.shooter.ready = tick % 100 >= 15;
            if (tick % 503 == 0) { original.end(true); updated.end(true); }
            original.execute(); updated.execute();
            require(original.getShotState().name().equals(updated.getShotState().name()), "manual state");
            require(original.getTargetRpm() == updated.getTargetRpm(), "manual RPM target");
            require(original.getGateElapsedMs() == updated.getGateElapsedMs(), "manual gate time");
            require(old.shooter.rpm == current.shooter.rpm && old.shooter.pitch == current.shooter.pitch
                    && old.shooter.pitchState == current.shooter.pitchState, "manual shooter and pitch");
            require(old.intake.state == current.intake.state && old.transit.state == current.transit.state,
                    "manual feed and gate");
            require(!updated.isFinished(), "manual does not auto-finish");
        }
        System.out.println("PASS: " + (solo ? "Solo" : "test") + " manual matches prior code for 3000 cycles");
    }

    private static void manualFixture() {
        Rig old = new Rig(), current = new Rig(); Gamepad pad = new Gamepad();
        BaselineTransitShooterManualCommand baseline = new BaselineTransitShooterManualCommand(
                old.intake, old.shooter, old.transit, pad);
        TransitShooterManualCommand command = new TransitShooterManualCommand(
                current.intake, current.shooter, current.transit, pad);
        Random random = new Random(12527);
        for (int i=0; i<3000; i++) {
            pad.a=random.nextBoolean(); pad.x=random.nextBoolean(); pad.y=random.nextBoolean();
            pad.dpad_up=random.nextBoolean(); pad.dpad_down=random.nextBoolean();
            pad.left_trigger=random.nextFloat(); pad.right_trigger=random.nextFloat();
            baseline.execute(); command.execute();
            require(old.shooter.rpm == current.shooter.rpm, "fixture RPM");
            require(baseline.getTargetRpm() == command.getTargetRpm(), "fixture RPM adjustment");
            require(old.intake.state == current.intake.state && old.transit.state == current.transit.state,
                    "fixture manual feed/gate priorities");
        }
        baseline.end(true); command.end(true);
        require(old.shooter.rpm == current.shooter.rpm && old.intake.state == current.intake.state
                && old.transit.state == current.transit.state, "fixture cancellation");
        System.out.println("PASS: independent manual fixture matches prior code for 3000 cycles");
    }
    public static void main(String[] args) { manual(true); manual(false); manualFixture(); }
}
