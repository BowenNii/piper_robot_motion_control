import sys
from dataclasses import replace
from pathlib import Path
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"tools/ros2"))
from csv_trajectory_preview import PreviewPoint
from real_ik_trajectory import check_points,ENDPOINT_REFERENCE


class RealTrajectoryGuardTests(unittest.TestCase):
    def points(self):
        return [PreviewPoint(k*20_000_000,[.1,.1,-.1,.1,.01,-.1],[0.]*6,[0.]*6) for k in range(501)]

    def test_stationary_sequence(self):
        self.assertEqual(check_points(self.points())["max_excursion_rad"],0)

    def test_count_time_excursion_and_derivative(self):
        cases=[]
        p=self.points();cases.append(p[:-1])
        p=self.points();p[20]=replace(p[20],time_ns=p[20].time_ns+1);cases.append(p)
        p=self.points();p[20]=replace(p[20],positions=[.2,.1,-.1,.1,.01,-.1]);cases.append(p)
        p=self.points();p[20]=replace(p[20],velocities=[.001]*6);cases.append(p)
        p=self.points();p[20]=replace(p[20],accelerations=[.001]*6);cases.append(p)
        for points in cases:
            with self.assertRaises(ValueError):check_points(points)

    def test_endpoint_profile(self):
        q=[]
        for k in range(3001):
            u=k/3000;s=10*u**3-15*u**4+6*u**5
            q.append([s*x for x in ENDPOINT_REFERENCE])
        p=[];dt=.02
        for k in range(3001):
            if k==0:
                v=[(-11*q[0][j]+18*q[1][j]-9*q[2][j]+2*q[3][j])/(6*dt) for j in range(6)]
                a=[(2*q[0][j]-5*q[1][j]+4*q[2][j]-q[3][j])/dt**2 for j in range(6)]
            elif k==3000:
                v=[(11*q[-1][j]-18*q[-2][j]+9*q[-3][j]-2*q[-4][j])/(6*dt) for j in range(6)]
                a=[(2*q[-1][j]-5*q[-2][j]+4*q[-3][j]-q[-4][j])/dt**2 for j in range(6)]
            else:
                v=[(q[k+1][j]-q[k-1][j])/(2*dt) for j in range(6)]
                a=[(q[k+1][j]-2*q[k][j]+q[k-1][j])/dt**2 for j in range(6)]
            p.append(PreviewPoint(k*20_000_000,q[k],v,a))
        metrics=check_points(p,endpoint=True)
        self.assertLess(metrics['velocity_peak_rad_s'],.016)
        with self.assertRaises(ValueError):check_points(p) # 不能混用放宽旧小幅模式。
        wrong=list(p);wrong[100]=replace(p[100],positions=[.1]*6)
        with self.assertRaises(ValueError):check_points(wrong,endpoint=True)


if __name__=="__main__":unittest.main()
