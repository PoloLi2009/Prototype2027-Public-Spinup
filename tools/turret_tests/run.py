"""Check startup hold against a fake motor without powering robot hardware."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = root / 'TeamCode/src/main/java'
stubs = {
    'com/arcrobotics/ftclib/command/SubsystemBase.java': 'package com.arcrobotics.ftclib.command; public class SubsystemBase { public void periodic() {} }',
    'com/qualcomm/robotcore/hardware/DcMotor.java': '''package com.qualcomm.robotcore.hardware;
public interface DcMotor {
 enum RunMode { STOP_AND_RESET_ENCODER, RUN_WITHOUT_ENCODER }
 enum ZeroPowerBehavior { BRAKE }
 void setMode(RunMode mode); void setPower(double power);
 void setZeroPowerBehavior(ZeroPowerBehavior behavior); int getCurrentPosition();
}''',
    'com/qualcomm/robotcore/hardware/DcMotorEx.java': 'package com.qualcomm.robotcore.hardware; public interface DcMotorEx extends DcMotor {}',
    'com/qualcomm/robotcore/hardware/HardwareMap.java': '''package com.qualcomm.robotcore.hardware;
public class HardwareMap {
 public DcMotorEx motor;
 public <T> T get(Class<T> type, String name) { return type.cast(motor); }
}''',
    'com/qualcomm/robotcore/util/Range.java': '''package com.qualcomm.robotcore.util;
public class Range { public static double clip(double v,double lo,double hi) { return Math.max(lo,Math.min(hi,v)); } }''',
    'TurretLockCheck.java': '''
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.subsystems.turret.*;
public class TurretLockCheck {
 static int checks;
 static void check(boolean ok, String message) {
  checks++; if (!ok) throw new AssertionError(message);
 }
 static class Motor implements DcMotorEx {
  double power; int position = 999; int resets;
  public void setPower(double p) { power=p; }
  public void setMode(RunMode mode) { if(mode==RunMode.STOP_AND_RESET_ENCODER) { position=0; resets++; } }
  public void setZeroPowerBehavior(ZeroPowerBehavior b) {}
  public int getCurrentPosition() { return position; }
 }
 public static void main(String[] args) {
  Motor motor = new Motor(); HardwareMap map = new HardwareMap(); map.motor=motor;
  Turret turret = new Turret(map);
  check(motor.power==0, "INIT must not drive motor");
  turret.periodic(); check(motor.power==0, "startup angle should need no correction");
  motor.position=turret.degreesToTicks(10); turret.periodic();
  check(motor.power>0 && motor.power<=0.25, "positive displacement correction with configured powerSign");
  double correction=motor.power;
  turret.rotate(-1); turret.setPower(-1); turret.stop();
  check(motor.power==correction, "manual requests and command cancellation must preserve hold");
  check(!turret.aimAtDegrees(90), "aiming at another angle must be rejected");
  check(!turret.aimAtPhysicalDegrees(-90,0.5,1,1), "test-specific aim must not override lock");
  check(!turret.holdAtPhysicalDegrees(90), "other hold target must be rejected");
  check(!turret.aimAtDegrees(Double.NaN), "invalid aim must not be accepted");
  check(turret.getTargetDegrees()==0, "target must remain startup angle");
  turret.resetEncoder(); check(motor.resets==1, "runtime reset must not shift hold reference");
  motor.position=turret.degreesToTicks(-10); turret.periodic();
  check(motor.power<0 && motor.power>=-0.25, "negative displacement correction");
  motor.position=0; turret.periodic();
  check(motor.power==0 && turret.holdAtPhysicalDegrees(0), "return to startup reference");
  motor.position=turret.degreesToTicks(5); turret.periodic(); turret.shutdown();
  check(motor.power==0, "STOP must remove power");
  turret.periodic(); turret.rotate(1); turret.setPower(1); turret.stop();
  check(!turret.aimAtDegrees(0) && motor.power==0, "shutdown must stay latched");
  Turret next = new Turret(map); motor.position=next.degreesToTicks(5); next.periodic();
  check(motor.power>0, "new session must resume holding its own reference");
  next.shutdown();
  System.out.println("PASS " + checks + " turret startup hold checks (fake motor)");
 }
}''',
}
with tempfile.TemporaryDirectory(prefix='spinup-turret-test-') as temp:
    directory = Path(temp)
    for relative, contents in stubs.items():
        path = directory / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)
    actual = [source / 'org/firstinspires/ftc/teamcode' / name for name in (
        'subsystems/turret/Turret.java', 'subsystems/turret/TurretConstants.java', 'utils/TurretMath.java')]
    java = Path('/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin')
    subprocess.run([str(java/'javac'), '-d', temp, *map(str,directory.rglob('*.java')), *map(str,actual)],check=True)
    subprocess.run([str(java/'java'), '-cp', temp, 'TurretLockCheck'],check=True)
