"""Offline real-FTCLib scheduler tests with fake devices and deterministic time."""
from pathlib import Path
import argparse, os, subprocess, tempfile, zipfile

here = Path(__file__).resolve().parent
root = here.parents[1]
source = root / "TeamCode/src/main/java"
package = Path("org/firstinspires/ftc/teamcode")
parser = argparse.ArgumentParser()
parser.add_argument("--baseline", type=Path, help="Optional pre-refactor SequencedShotCommand.java")
args = parser.parse_args()
cache = Path(os.environ.get("GRADLE_USER_HOME", Path.home()/".cache/ftc1-gradle"))/"caches/modules-2/files-2.1"
java = Path(os.environ.get("JAVA_HOME", "/Applications/Android Studio.app/Contents/jbr/Contents/Home"))/"bin"
names = ["ShootCommand", "SequencedShotCommand", "FlywheelRpmCommand", "RunIntakeCommand",
         "SetPitchCommand", "GateCommand", "IntakeCommand", "ReverseIntakeCommand",
         "DriveWithAimCommand", "FollowPathCommand", "TransitShooterManualCommand"]
actual = [source/package/"commands"/(name+".java") for name in names]
actual += [source/package/path for path in ["subsystems/shooter/ShooterConstants.java",
    "subsystems/drive/DriveConstants.java", "opmodes/auto/AutoFormalRoute.java"]]
with tempfile.TemporaryDirectory(prefix="spinup-command-tests-") as temp:
    out = Path(temp)
    aar = next((cache/"org.ftclib.ftclib/core/2.1.1").rglob("*.aar"))
    with zipfile.ZipFile(aar) as archive:
        (out/"ftclib.jar").write_bytes(archive.read("classes.jar"))
    pedro = next((cache/"com.pedropathing/core/3.0.2").rglob("*.jar"))
    cp = os.pathsep.join(map(str, [out/"ftclib.jar", pedro]))
    sources = list((here/"fixtures").rglob("*.java")) + actual + [here/"CommandLifecycleCheck.java"]
    if args.baseline:
        baseline = out/"BaselineSequencedShotCommand.java"
        baseline.write_text(args.baseline.read_text().replace("SequencedShotCommand", "BaselineSequencedShotCommand"))
        fixture = args.baseline.parent/"commands-before/TransitShooterManualCommand.java"
        if fixture.exists():
            fixture_baseline = out/"BaselineTransitShooterManualCommand.java"
            fixture_baseline.write_text(fixture.read_text().replace("TransitShooterManualCommand", "BaselineTransitShooterManualCommand"))
            sources += [baseline, fixture_baseline, here/"ManualCompatibilityCheck.java"]
        else:
            raise FileNotFoundError("Baseline directory also needs commands-before/TransitShooterManualCommand.java")
    subprocess.run([str(java/"javac"), "-encoding", "UTF-8", "-cp", cp, "-d", str(out), *map(str,sources)], check=True)
    for test in ["CommandLifecycleCheck"] + (["ManualCompatibilityCheck"] if args.baseline else []):
        subprocess.run([str(java/"java"), "-cp", str(out)+os.pathsep+cp, test], check=True)
