"""python -m unittest discover -s analysis/tests -v；纯离线，不连接硬件。"""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from piper_analysis.robot_model.urdf import load_piper
from piper_analysis.robot_model.types import RobotModel
from piper_analysis.robot_model.payload import with_payload
from piper_analysis.kinematics.forward import forward
from piper_analysis.dynamics.spatial_inertia import spatial_inertia
from piper_analysis.lie_group.transforms import transform
from piper_analysis.dynamics.rnea import rnea
from piper_analysis.dynamics.crba import crba
from piper_analysis.dynamics.terms import gravity, velocity_bias, forward_dynamics
from piper_analysis.dynamics.friction import friction
from piper_analysis.identification.regressor import regressor, stack_regressors
from piper_analysis.identification.parameters import inertial_parameters
from piper_analysis.identification.fitting import fit_parameters
from piper_analysis.control.computed_torque import computed_torque
from piper_analysis.data_io.csv import read_numeric, columns
from piper_analysis.experiments.project2.pipeline import analyze_csv
from piper_analysis.dynamics.batch import torque_prediction


class DynamicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = load_piper()

    def test_matlab_reference(self):
        file = Path(__file__).parent/'fixtures/matlab_dynamics_reference.csv'
        self.assertTrue(file.exists(), 'MATLAB reference fixture is required, not skipped')
        from piper_analysis.common.paths import REPO
        from piper_analysis.data_io.provenance import sha256
        provenance = json.loads((file.parent/'reference_provenance.json').read_text())
        for path, expected in provenance['hashes'].items():
            self.assertEqual(sha256(REPO/path), expected, path)
        d = read_numeric(file)
        for k in range(len(d['q1'])):
            q, dq, ddq = [columns(d, [f'{prefix}{i}' for i in range(1, 7)])[k] for prefix in ['q', 'dq', 'ddq']]
            np.testing.assert_allclose(rnea(q, dq, ddq, self.model), columns(d, [f'tau{i}' for i in range(1, 7)])[k], atol=1e-10)
            np.testing.assert_allclose(gravity(q, self.model), columns(d, [f'g{i}' for i in range(1, 7)])[k], atol=1e-10)
            np.testing.assert_allclose(crba(q, self.model), columns(d, [f'M{i}{j}' for i in range(1, 7) for j in range(1, 7)])[k].reshape(6, 6), atol=1e-10)
            np.testing.assert_allclose(rnea(q, dq, ddq, self.model, [.1, -.2, .3, .4, -.5, .6]),
                                       columns(d, [f'loaded{i}' for i in range(1, 7)])[k], atol=1e-10)

    def test_identities_and_positive_mass(self):
        rng = np.random.default_rng(43); m = self.model
        for _ in range(8):
            q = rng.uniform(m.q_min, m.q_max); dq = rng.normal(size=6); ddq = rng.normal(size=6)
            M = crba(q, m)
            np.testing.assert_allclose(M, M.T, atol=1e-12)
            self.assertGreater(np.linalg.eigvalsh(M).min(), 0)
            tau = rnea(q, dq, ddq, m)
            np.testing.assert_allclose(tau, M@ddq+velocity_bias(q, dq, m)+gravity(q, m), atol=1e-10)
            np.testing.assert_allclose(forward_dynamics(q, dq, tau, m), ddq, atol=1e-9)

    def test_regressor(self):
        q = np.array([.1, .2, -.3, .2, .1, -.2]); dq = q*.3; ddq = -q
        Y = regressor(q, dq, ddq, self.model)
        np.testing.assert_allclose(Y@inertial_parameters(self.model), rnea(q, dq, ddq, self.model), atol=1e-11)

    def test_single_pendulum_analytic(self):
        # 绕y旋转，质心在+x，m=2,l=.4,Iyy=.03，g沿-z。
        m, l, inertia = 2., .4, .03
        model = RobotModel(np.array([[0, 1, 0, 0, 0, 0]]).T, np.array([np.eye(4), np.eye(4)]),
            np.array([spatial_inertia(m, np.eye(3)*inertia, [l, 0, 0])]), np.array([0., 0., -9.81]),
            np.array([-3.]), np.array([3.]), np.array([1.]), np.array([10.]))
        for q in [-.6, 0, .8]:
            self.assertAlmostEqual(gravity([q], model)[0], -m*9.81*l*np.cos(q))
            self.assertAlmostEqual(crba([q], model)[0, 0], inertia+m*l*l)

    def test_gravity_potential_gradient(self):
        # 独立用势能差分核对重力符号/坐标系，而不只比较同算法移植。
        m = self.model; q = np.array([.2, .3, -.5, .1, .2, -.1])
        from piper_analysis.lie_group.transforms import exp_twist
        def potential(q):
            T = np.eye(4); E = 0.
            # POE前缀乘以零位基座到link变换。
            prefix = np.eye(4); home = np.eye(4)
            for i in range(6):
                prefix = prefix @ exp_twist(m.S[:, i]*q[i]); home = home @ m.Mlist[i]
                T = prefix @ home; G = m.Glist[i]; mass = np.trace(G[3:, 3:])/3
                H = G[:3, 3:]; com = np.array([H[2, 1], H[0, 2], H[1, 0]])/mass
                E -= mass*m.gravity @ (T[:3, :3]@com+T[:3, 3])
            return E
        gradient = np.array([(potential(q+np.eye(6)[j]*1e-6)-potential(q-np.eye(6)[j]*1e-6))/2e-6 for j in range(6)])
        np.testing.assert_allclose(gravity(q, m), gradient, atol=1e-8)

    def test_friction_control_and_fit(self):
        np.testing.assert_allclose(friction([1, -1], [1, 1], [2, 2], [.1, .1]), [3, -3], atol=1e-7)
        q = np.zeros(6); gain = np.ones(6)
        np.testing.assert_allclose(computed_torque(q, q, q, q, q, gain, gain, self.model), gravity(q, self.model))
        fit = fit_parameters(np.array([[1., 0], [0, 1], [1, 1]]), [2, 3, 5])
        np.testing.assert_allclose(fit['parameters'], [2, 3]); self.assertEqual(fit['rank'], 2)
        rank_deficient = fit_parameters(np.ones((3, 2)), [1, 1, 1]); self.assertEqual(rank_deficient['rank'], 1)

    def test_invalid_inputs(self):
        with self.assertRaises(ValueError): rnea([0], [0], [0], self.model)
        with self.assertRaises(ValueError): spatial_inertia(-1, np.eye(3), [0, 0, 0])
        with self.assertRaises(ValueError): friction([1], [-1], [1], [.1])

    def test_explicit_payload(self):
        model = self.model; before = model.Glist.copy()
        T = transform(p=[.1, 0, 0]); mass = .2
        loaded = with_payload(model, mass, np.eye(3)*.001, [0, 0, 0], T)
        np.testing.assert_array_equal(model.Glist, before)
        q = np.array([0, .3, -.4, .1, .1, .1]); eps = 1e-6
        def energy(q):
            pose = forward(model, q)@T
            return -mass*model.gravity @ pose[:3, 3]
        gradient = np.array([(energy(q+np.eye(6)[j]*eps)-energy(q-np.eye(6)[j]*eps))/(2*eps) for j in range(6)])
        np.testing.assert_allclose(gravity(q, loaded)-gravity(q, model), gradient, atol=1e-8)

    def test_batch_and_identification_stack(self):
        q = np.array([[0, .3, -.4, 0, 0, 0], [.1, .4, -.5, .1, 0, .1]])
        dq = q*.1; ddq = -q*.2
        predicted = torque_prediction(q, dq, ddq, self.model)['total']
        np.testing.assert_allclose(stack_regressors(q, dq, ddq, self.model)@inertial_parameters(self.model), predicted.reshape(-1), atol=1e-10)

    def test_project2_csv_gate_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); source = root/'states.csv'
            names = ['time_s']+[f'{prefix}{j}_{unit}' for prefix, unit in [('q', 'rad'), ('dq', 'rad_s'), ('ddq', 'rad_s2'), ('tau', 'Nm')] for j in range(1, 7)]
            values = np.zeros((8, 25)); values[:, 0] = np.arange(8)*.01
            np.savetxt(source, values, delimiter=',', header=','.join(names), comments='')
            with self.assertRaises(ValueError): analyze_csv(source, root/'rejected')
            self.assertFalse((root/'rejected').exists())
            analyze_csv(source, root/'accepted', measured_torque_verified=True)
            self.assertTrue((root/'accepted/residual_metrics.csv').exists())
            with self.assertRaises(FileExistsError): analyze_csv(source, root/'accepted', True)

if __name__ == '__main__':
    unittest.main()
