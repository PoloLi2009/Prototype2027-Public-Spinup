package org.firstinspires.ftc.teamcode.subsystems.drive;
import com.arcrobotics.ftclib.command.SubsystemBase; import com.pedropathing.paths.Path;
public class PedroDrive extends SubsystemBase {public boolean ready=true,busy;public int stops,starts;public Path current;
public boolean isReady(){return ready;} public boolean isBusy(){return busy;}
public void follow(Path p){current=p;busy=true;starts++;} public void stop(){busy=false;stops++;}}
