package org.firstinspires.ftc.teamcode.subsystems.localization.fusion;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import java.util.ArrayList;
import java.util.Map;
import java.util.TreeMap;

/** Three-state EKF (field x,y,yaw), driven by local Pinpoint increments.
 * Delayed corrections are applied at capture time and subsequent odometry is replayed.
 */
public class PinpointEkf {
    private static final class State {
        double time;
        Pose2d odometry;
        double[] x;
        double[][] p;
        State(double t, Pose2d odom, double[] x, double[][] p) {
            time=t; odometry=odom; this.x=x; this.p=p;
        }
    }
    private final TreeMap<Double, State> history = new TreeMap<>();
    private boolean seeded;
    private double lastVisionTime = Double.NEGATIVE_INFINITY;
    private String status = "Waiting for OD";
    private double lastNis = Double.NaN;
    public void reset() {
        history.clear(); seeded=false; lastVisionTime=Double.NEGATIVE_INFINITY;
        status="Waiting for OD"; lastNis=Double.NaN;
    }
    public void updateWithTime(double time, Pose2d odometry) {
        if (history.isEmpty()) {
            history.put(time, new State(time, odometry, vector(odometry), diagonal(1,1,1)));
        } else {
            State last = history.lastEntry().getValue();
            if (time <= last.time) return;
            history.put(time, predict(last, time, odometry));
        }
        while (history.size()>2 && history.firstKey()<time-PoseHistory.DURATION_SECONDS)
            history.pollFirstEntry();
    }
    private State predict(State prev, double time, Pose2d odom) {
        Transform2d delta = odom.minus(prev.odometry);
        double dx=delta.getTranslation().getX(), dy=delta.getTranslation().getY();
        double h=prev.x[2], c=Math.cos(h), s=Math.sin(h);
        double[] x={prev.x[0]+c*dx-s*dy,prev.x[1]+s*dx+c*dy,
                wrap(h+delta.getRotation().getRadians())};
        double[][] f=identity(); f[0][2]=-s*dx-c*dy; f[1][2]=c*dx-s*dy;
        double dt=Math.max(0,time-prev.time), dist=Math.hypot(dx,dy);
        // Starting noise estimates; tune with logged stationary/moving measurements.
        double qxy=0.03*0.03*dt+0.02*0.02*dist;
        double qh=Math.pow(Math.toRadians(2),2)*dt
                +Math.pow(Math.toRadians(1),2)*Math.abs(delta.getRotation().getRadians());
        double[][] p=add(mul(mul(f,prev.p),transpose(f)),diagonal(qxy,qxy,qh));
        return new State(time,odom,x,p);
    }
    public boolean addVisionMeasurement(Pose2d measurement, double time, double xyStd,
                                        double headingStd, Pose2d odomAtCapture) {
        if (history.isEmpty() || time<history.firstKey() || time>history.lastKey()
                || time<=lastVisionTime) { status="Outside history / out of order"; return false; }
        State floor=history.floorEntry(time).getValue();
        State at=predict(floor,time,odomAtCapture);
        double[] z=vector(measurement);
        double[][] r=diagonal(xyStd*xyStd,xyStd*xyStd,headingStd*headingStd);
        if (!seeded) {
            at.x=z; at.p=r; lastNis=0;
        } else {
            double[] error={z[0]-at.x[0],z[1]-at.x[1],wrap(z[2]-at.x[2])};
            double[][] inv=inverse(add(at.p,r));
            if (inv==null) {status="Invalid covariance"; return false;}
            lastNis=0;
            for(int i=0;i<3;i++) for(int j=0;j<3;j++) lastNis+=error[i]*inv[i][j]*error[j];
            if (!Double.isFinite(lastNis) || lastNis>11.345
                    || Math.hypot(error[0],error[1])>0.75
                    || Math.abs(error[2])>Math.toRadians(45)) {
                status="Rejected vision innovation"; return false;
            }
            double[][] k=mul(at.p,inv);
            for(int i=0;i<3;i++) for(int j=0;j<3;j++) at.x[i]+=k[i][j]*error[j];
            at.x[2]=wrap(at.x[2]);
            double[][] a=identity();
            for(int i=0;i<3;i++) for(int j=0;j<3;j++) a[i][j]-=k[i][j];
            // Joseph covariance update preserves positive semidefiniteness under rounding.
            at.p=add(mul(mul(a,at.p),transpose(a)),mul(mul(k,r),transpose(k)));
        }
        history.put(time,at);
        State previous=at;
        for(Map.Entry<Double,State> e:new ArrayList<>(history.tailMap(time,false).entrySet())) {
            State next=predict(previous,e.getKey(),e.getValue().odometry);
            history.put(e.getKey(),next); previous=next;
        }
        seeded=true; lastVisionTime=time; status="Vision accepted"; return true;
    }
    public Pose2d getEstimatedPosition() {
        if(history.isEmpty()) return new Pose2d();
        double[] x=history.lastEntry().getValue().x;
        return new Pose2d(x[0],x[1],new Rotation2d(x[2]));
    }
    public double[] getStandardDeviations() {
        if(history.isEmpty()) return new double[]{Double.NaN,Double.NaN,Double.NaN};
        double[][] p=history.lastEntry().getValue().p;
        return new double[]{Math.sqrt(Math.max(0,p[0][0])),Math.sqrt(Math.max(0,p[1][1])),
                Math.sqrt(Math.max(0,p[2][2]))};
    }
    public boolean isSeeded() {return seeded;}
    public String getStatus() {return status;}
    public double getLastNis() {return lastNis;}
    private static double[] vector(Pose2d p) {return new double[]{p.getX(),p.getY(),p.getRotation().getRadians()};}
    public static double wrap(double v) {return Math.atan2(Math.sin(v),Math.cos(v));}
    private static double[][] diagonal(double a,double b,double c) {return new double[][]{{a,0,0},{0,b,0},{0,0,c}};}
    private static double[][] identity() {return diagonal(1,1,1);}
    private static double[][] add(double[][]a,double[][]b) {
        double[][]o=new double[3][3];for(int i=0;i<3;i++)for(int j=0;j<3;j++)o[i][j]=a[i][j]+b[i][j];return o;
    }
    private static double[][] mul(double[][]a,double[][]b) {
        double[][]o=new double[3][3];for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)o[i][j]+=a[i][k]*b[k][j];return o;
    }
    private static double[][] transpose(double[][]a) {
        double[][]o=new double[3][3];for(int i=0;i<3;i++)for(int j=0;j<3;j++)o[i][j]=a[j][i];return o;
    }
    private static double[][] inverse(double[][]a) {
        double[][]v=new double[3][6];
        for(int i=0;i<3;i++){System.arraycopy(a[i],0,v[i],0,3);v[i][i+3]=1;}
        for(int c=0;c<3;c++){
            int pivot=c;for(int i=c+1;i<3;i++)if(Math.abs(v[i][c])>Math.abs(v[pivot][c]))pivot=i;
            if(!Double.isFinite(v[pivot][c]) || Math.abs(v[pivot][c])<1e-15)return null;
            double[]tmp=v[c];v[c]=v[pivot];v[pivot]=tmp;
            double scale=v[c][c];for(int j=0;j<6;j++)v[c][j]/=scale;
            for(int i=0;i<3;i++)if(i!=c){double f=v[i][c];for(int j=0;j<6;j++)v[i][j]-=f*v[c][j];}
        }
        double[][]out=new double[3][3];for(int i=0;i<3;i++)System.arraycopy(v[i],3,out[i],0,3);return out;
    }
}
