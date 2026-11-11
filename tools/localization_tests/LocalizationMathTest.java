import com.arcrobotics.ftclib.geometry.*;
import org.firstinspires.ftc.teamcode.subsystems.localization.fusion.*;
import org.firstinspires.ftc.teamcode.subsystems.vision.*;

public class LocalizationMathTest {
    private static int checks;
    private static void near(double actual,double expected,double tolerance,String label) {
        checks++;
        if(!Double.isFinite(actual)||Math.abs(actual-expected)>tolerance)
            throw new AssertionError(label+": "+actual+" != "+expected);
    }
    private static void yes(boolean value,String label) {
        checks++;if(!value)throw new AssertionError(label);
    }
    private static Pose2d pose(double x,double y,double deg) {
        return new Pose2d(x,y,Rotation2d.fromDegrees(deg));
    }
    public static void main(String[] args) {
        // Simulated raw fmap poses for both signed tilts, field headings and turret positions.
        // Build the physical rigid transform first, then extract yaw/pitch/roll as MT1 does.
        for(double tilt:new double[]{-30,0,30}) for(double heading:new double[]{-179,-90,0,90,179})
        for(double turret:new double[]{-170,-35,0,60,170}) for(double x:new double[]{-1.2,1.2}) {
            double phi=Math.toRadians(-tilt),yaw=Math.toRadians(heading-turret);
            double c=Math.cos(phi),s=Math.sin(phi),cy=Math.cos(yaw),sy=Math.sin(yaw);
            double h=VisionConstants.hivePivotHeightMeters;
            double rawX=c*x-s*h,rawZ=-s*x-c*h+h;
            double rawYaw=Math.atan2(sy,c*cy);
            double rawPitch=Math.asin(s*cy);
            double rawRoll=Math.atan2(s*sy,c);
            HiveFlatPoseSolver.Result r=HiveFlatPoseSolver.solve(rawX,0.7,rawZ,
                    rawYaw,rawPitch,rawRoll,turret);
            yes(r!=null,"valid synthetic geometry");
            near(r.chassisPose.getX(),x,1e-9,"corrected x");
            near(r.chassisPose.getY(),0.7,1e-9,"corrected y");
            near(PinpointEkf.wrap(r.chassisPose.getRotation().getRadians()-Math.toRadians(heading)),0,1e-9,"chassis yaw");
            near(r.hiveTiltDegrees,tilt,1e-9,"tilt direction");
        }
        yes(HiveFlatPoseSolver.solve(1,0,0,0,0,Math.PI,0)==null,"upside-down origin rejected");
        yes(HiveFlatPoseSolver.solve(1,0,0.5,0,0,0,0)==null,"wrong floor height rejected");
        yes(HiveFlatPoseSolver.solve(Double.NaN,0,0,0,0,0,0)==null,"NaN rejected");
        PoseHistory h=new PoseHistory();h.add(0,pose(0,0,179),170);h.add(1,pose(0,0,-179),190);
        near(Math.abs(h.sample(.5).odometry.getRotation().getDegrees()),180,1e-7,"yaw interpolation");
        near(h.sample(.5).turretDegrees,180,1e-7,"unwrapped turret interpolation");
        yes(h.sample(-.1)==null && h.sample(1.1)==null,"history rejects outside time");

        PinpointEkf ekf=new PinpointEkf();WpiPinpointPoseEstimator wpi=new WpiPinpointPoseEstimator();
        for(int i=0;i<=10;i++){ekf.updateWithTime(i*.1,pose(i*.1,0,0));wpi.updateWithTime(i*.1,pose(i*.1,0,0));}
        yes(ekf.addVisionMeasurement(pose(2,3,90),.5,.1,.2,pose(.5,0,0)),"EKF first vision seed");
        yes(wpi.addVisionMeasurement(pose(2,3,90),.5,.1,.2),"WPI first vision seed");
        near(ekf.getEstimatedPosition().getX(),2,1e-9,"EKF delayed replay x");
        near(ekf.getEstimatedPosition().getY(),3.5,1e-9,"EKF rotates future OD into corrected heading");
        near(wpi.getEstimatedPosition().getY(),3.5,1e-9,"WPI delayed replay");
        yes(!ekf.addVisionMeasurement(pose(20,30,90),.6,.1,.2,pose(.6,0,0)),"EKF rejects outlier");
        yes(!wpi.addVisionMeasurement(pose(20,30,90),.6,.1,.2),"WPI rejects outlier");
        yes(!ekf.addVisionMeasurement(pose(2,3,90),.5,.1,.2,pose(.5,0,0)),"EKF rejects duplicate");
        yes(!wpi.addVisionMeasurement(pose(2,3,90),.4,.1,.2),"WPI rejects old vision");
        yes(ekf.addVisionMeasurement(pose(2,3.3,90),.8,.1,.2,pose(.8,0,0)),"EKF subsequent delayed measurement");
        near(ekf.getEstimatedPosition().getY(),3.5,1e-9,"no motion lost after correction");
        for(double sigma:ekf.getStandardDeviations())yes(Double.isFinite(sigma)&&sigma>0,"positive covariance");
        PinpointEkf angle=new PinpointEkf();angle.updateWithTime(0,pose(0,0,0));angle.updateWithTime(.1,pose(0,0,0));
        yes(angle.addVisionMeasurement(pose(0,0,179),0,.1,.1,pose(0,0,0)),"yaw seed");
        yes(angle.addVisionMeasurement(pose(0,0,-179),.1,.1,.1,pose(0,0,0)),"wrap innovation accepted");
        // Long-running stationary noisy observations must converge, not accumulate noise or NaNs.
        java.util.Random rng=new java.util.Random(7);
        for(int i=1;i<=500;i++){
            double time=.1+i*.02;angle.updateWithTime(time,pose(0,0,0));
            angle.addVisionMeasurement(pose(rng.nextGaussian()*.02,rng.nextGaussian()*.02,
                    180+rng.nextGaussian()),time,.08,.15,pose(0,0,0));
        }
        yes(Math.hypot(angle.getEstimatedPosition().getX(),angle.getEstimatedPosition().getY())<.03,"stationary noise filtered");
        for(double sigma:angle.getStandardDeviations())yes(Double.isFinite(sigma)&&sigma>0,"covariance after many updates");
        System.out.println("Passed "+checks+" localization checks.");
    }
}
