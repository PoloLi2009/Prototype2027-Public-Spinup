"""Deterministic physics, telemetry, and four-shot video integration checks."""
import math
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from analysis import track, fit_trajectory, shot_advice, rpm_diagnostics


class AnalysisTests(unittest.TestCase):
    def points(self):
        return [dict(t=1+t,x=50+400*t,y=250-300*t+490.3325*t*t)
                for t in np.arange(0,.501,.01)]

    def test_calibrated_motion_and_angle_solutions(self):
        fit=fit_trajectory(self.points(),.01)
        self.assertAlmostEqual(fit['speed_m_s'],5,places=6)
        self.assertAlmostEqual(fit['vertical_accel_m_s2'],-9.80665,places=5)
        target_x=50+400*.4;target_y=250-300*.4+490.3325*.4**2
        advice=shot_advice(fit,target_x,target_y,side_view=True,starts_at_exit=True,current_rpm=5000)
        self.assertAlmostEqual(advice['target_error_px'],0,places=6)
        self.assertAlmostEqual(advice['estimated_speed_ratio'],1,places=6)
        self.assertAlmostEqual(advice['model_angle_change_deg'],0,places=5)

    def test_no_fake_physics_without_calibration(self):
        fit=fit_trajectory(self.points())
        advice=shot_advice(fit,210,210)
        self.assertIsNone(fit['speed_m_s'])
        self.assertNotIn('model_rpm',advice)
        self.assertNotIn('estimated_required_speed_m_s',advice)

    def test_outlier_and_invalid_timestamp(self):
        pts=self.points();pts[20]['x']+=500
        fit=fit_trajectory(pts,.01)
        self.assertTrue(fit['usable'])
        self.assertLess(fit['inliers'],len(pts))
        self.assertAlmostEqual(fit['speed_m_s'],5,places=5)
        pts[2]['t']=pts[1]['t']
        self.assertFalse(fit_trajectory(pts)['usable'])

    def test_rpm_recovery_requires_sustained_window(self):
        rows=[]
        for t in np.arange(0,3,.02):
            rpm=5000 if t<1 or t>=1.5 else 4000+2000*(t-1)
            rows.append(dict(time_s=float(t),target_rpm=5000,actual_rpm=rpm))
        result=rpm_diagnostics(rows,[1.0])
        self.assertAlmostEqual(result['shots'][0]['minimum_rpm'],4000)
        self.assertTrue(.44<=result['shots'][0]['recovery_s']<=.5)
        rows[20]['time_s']=rows[19]['time_s']
        with self.assertRaises(ValueError):rpm_diagnostics(rows)

    def test_four_shot_video_tracking(self):
        fps=120; starts=[.1,.7,1.3,1.9]
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'four.avi'
            writer=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'MJPG'),fps,(400,300))
            self.assertTrue(writer.isOpened())
            for frame_idx in range(300):
                image=np.full((300,400,3),25,np.uint8)
                for start in starts:
                    age=frame_idx/fps-start
                    if 0<=age<=.45:
                        x=round(50+400*age);y=round(250-300*age+490.3325*age*age)
                        cv2.circle(image,(x,y),9,(30,210,250),-1)
                        cv2.circle(image,(x-2,y-2),3,(245,245,245),-1)
                writer.write(image)
            writer.release()
            for start in starts:
                pts,_=track(path,start,start+.45,50,250,13,search=50,threshold=.55)
                self.assertGreater(len(pts),45)
                fit=fit_trajectory(pts,.01)
                self.assertTrue(fit['usable'])
                self.assertAlmostEqual(fit['speed_m_s'],5,delta=.25)


if __name__=='__main__':unittest.main()
