"""Run pure-Java geometry/fusion checks with the project's cached FTCLib dependency."""
from pathlib import Path
import subprocess, tempfile, zipfile, os
root=Path(__file__).resolve().parents[2]
cache=Path(os.environ.get('GRADLE_USER_HOME',Path.home()/'.gradle'))/'caches/modules-2/files-2.1/org.ftclib.ftclib/core/2.1.1'
archive=next(cache.rglob('core-2.1.1.aar'))
source=root/'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems'
files=list((source/'localization/fusion').glob('*.java'))+[source/'vision/HiveFlatPoseSolver.java',source/'vision/VisionConstants.java',Path(__file__).with_name('LocalizationMathTest.java')]
with tempfile.TemporaryDirectory(prefix='spinup-localization-') as temp:
 jar=Path(temp)/'classes.jar'
 with zipfile.ZipFile(archive) as z:jar.write_bytes(z.read('classes.jar'))
 subprocess.run(['javac','-cp',str(jar),'-d',temp,*map(str,files)],check=True)
 subprocess.run(['java','-cp',temp+os.pathsep+str(jar),'LocalizationMathTest'],check=True)
