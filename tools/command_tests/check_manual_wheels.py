"""Verify real Blue command -> real Drive -> SDK motor boundary for six axis directions."""
from pathlib import Path
import ast, subprocess, tempfile
root=Path(__file__).resolve().parents[2]
# Reuse SDK type definitions without running another test or copying robot implementation.
tree=ast.parse(Path(__file__).with_name('check_shooter.py').read_text())
stubs=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='stubs' for t in n.targets))
stubs.pop('ShooterThresholdCheck.java')
stubs['com/arcrobotics/ftclib/command/CommandBase.java']='package com.arcrobotics.ftclib.command; public class CommandBase {public void addRequirements(Object... x){}public void initialize(){}public void execute(){}public void end(boolean i){}}'
stubs['com/arcrobotics/ftclib/gamepad/GamepadEx.java']='package com.arcrobotics.ftclib.gamepad; public class GamepadEx {public double getLeftY(){return 0;}public double getLeftX(){return 0;}public double getRightX(){return 0;}}'
stubs['WheelCheck.java']='''import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.subsystems.drive.*;import org.firstinspires.ftc.teamcode.commands.BlueManualDriveCommand;
public class WheelCheck {static class M implements DcMotor {double power;Direction direction; public void setPower(double p){power=p;}public void setDirection(Direction d){direction=d;} public void setMode(RunMode m){}public void setZeroPowerBehavior(ZeroPowerBehavior b){}}
static void check(boolean b,String s){if(!b)throw new AssertionError(s);}
public static void main(String[] args){HardwareMap h=new HardwareMap();M[] motors={new M(),new M(),new M(),new M()};String[] names={DriveConstants.leftFrontMotorName,DriveConstants.leftBackMotorName,DriveConstants.rightFrontMotorName,DriveConstants.rightBackMotorName};
for(int i=0;i<4;i++)h.items.put(names[i],motors[i]);Drive drive=new Drive(h);
check(motors[0].direction==DcMotorSimple.Direction.FORWARD&&motors[1].direction==DcMotorSimple.Direction.FORWARD&&motors[2].direction==DcMotorSimple.Direction.REVERSE&&motors[3].direction==DcMotorSimple.Direction.REVERSE,"all four installed directions inverted from prior left REVERSE/right FORWARD");
double[] axes={0,0,0};var command=new BlueManualDriveCommand(drive,()->axes[0],()->axes[1],()->axes[2],()->0,()->true,t->t);command.initialize();
double[][] expected={{1,1,1,1},{1,-1,-1,1},{1,1,-1,-1}};
for(int axis=0;axis<3;axis++)for(int sign:new int[]{-1,1}){java.util.Arrays.fill(axes,0);axes[axis]=sign;command.execute();for(int m=0;m<4;m++)check(motors[m].power==expected[axis][m]*sign,"six-axis Blue mixer output preserved at actual motor boundary");}
command.end(true);for(M m:motors)check(m.power==0,"stop clears every motor");
System.out.println("PASS: real Blue command and Drive six-axis wheel outputs, all-four direction correction, and STOP");}}
'''
source=root/'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
java=Path('/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin')
with tempfile.TemporaryDirectory(prefix='spinup-wheel-check-') as temp:
    out=Path(temp)
    for name,code in stubs.items():
        p=out/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(code)
    actual=[source/p for p in ['commands/BlueManualDriveCommand.java','subsystems/drive/Drive.java','subsystems/drive/DriveConstants.java']]
    subprocess.run([str(java/'javac'),'-d',temp,*map(str,out.rglob('*.java')),*map(str,actual)],check=True)
    subprocess.run([str(java/'java'),'-cp',temp,'WheelCheck'],check=True)
