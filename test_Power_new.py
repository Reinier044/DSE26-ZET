import unittest
from unittest import TestCase
# for unit tests
from Power_new import stat_traction, init_acc, opt_force_ratio, v_t
# for system tests
from Power_new import EGTS_only_perf, static_power, car_power

class Test(TestCase):

    # unit tests

    def test_stat_traction(self):
        example_T1, example_N1, example_wheelrad1 = 400, 1000, .5
        example_T2, example_N2, example_wheelrad2 = 500, 200, .5
        example_T3, example_N3, example_wheelrad3 = 500, 1000, .5

        # expected True
        self.assertTrue(stat_traction(example_T1, example_N1, example_wheelrad1))

        # expected False
        self.assertFalse(stat_traction(example_T2, example_N2, example_wheelrad2))

        # boundary, expected True
        self.assertTrue(stat_traction(example_T3, example_N3, example_wheelrad3))

    def test_init_acc(self):
        # Roll_fric = 0.02  # [-] Rolling friction coefficient of airplane wheels
        # if a[1] >= 2.5 * Roll_fric:
        #     return True
        # else:
        #     return False

        # expected True
        example_a1 = [.5, .5]
        self.assertTrue(init_acc(example_a1))
        # expected False
        example_a2 = [.02, .02]
        self.assertFalse(init_acc(example_a2))
        # boundary, expected True
        example_a3 = [.05, .05]
        self.assertTrue(init_acc(example_a3))


    def test_opt_force_ratio(self):
        case1_f, case1_a = 10000, [1]
        self.assertAlmostEqual(opt_force_ratio(case1_f, case1_a), .07, 3)

        case2_f, case2_a = 100000, [.5]
        self.assertEqual(opt_force_ratio(case2_f, case2_a), 1)

    def test_v_t(self):
        test1_a, test1_t = [1, 1, 2, 2, 3], [0, 1, 2, 3, 4]
        self.assertListEqual(v_t(test1_a, test1_t), [0, 1, 3, 5, 8])

        test2_a, test2_t = [1, 0, -2, 2, 3], [0, 1, 2, 3, 4]
        self.assertListEqual(v_t(test2_a, test2_t), [0, 0, -2, 0, 3])

    # system tests

    def test_EGTS_only_perf(self):
        a, f, v, t = EGTS_only_perf(30)
        self.assertTrue(len(a) == len(v))
        self.assertTrue(len(a) == len(t))

    def test_static_power(self):
        # test that constant velocity gives constant power
        v1 = [2, 2]
        t1 = [0, 1]
        p1, c11, c12 = static_power(v1, t1, .5)
        self.assertTrue(p1[0] == p1[1])
        self.assertTrue(c11[0] == c11[1])
        self.assertTrue(c12[0] == c12[1])

        # test that if velocity increases, the power increases
        v2 = [1, 2]
        t2 = [0, 1]
        p2, c21, c22 = static_power(v2, t2, .5)
        self.assertTrue(p2[0] <= p2[1])
        self.assertTrue(c21[0] <= c21[1])
        self.assertTrue(c22[0] <= c22[1])

        # test that if velocity decreases, the power decreases
        v3 = [2, 1]
        t3 = [0, 1]
        p3, c31, c32 = static_power(v3, t3, .5)
        self.assertTrue(p3[0] >= p3[1])
        self.assertTrue(c31[0] >= c31[1])
        self.assertTrue(c32[0] >= c32[1])

        # test that if velocity == 0, power == 0
        v4 = [0, 0]
        t4 = [0, 1]
        p4, c41, c42 = static_power(v4, t4, .5)
        self.assertTrue(p4[0] == 0)
        self.assertTrue(c41[0] == 0)
        self.assertTrue(c42[0] == 0)

    def test_car_power(self):
        a = [0, 1, 2]
        v = [1, 2, 4]
        ratio = 0.5

        # power increases as velocity increases
        self.assertTrue(car_power(a[0], v[0], ratio)[0] < car_power(a[1], v[1], ratio)[0])
        self.assertTrue(car_power(a[1], v[1], ratio)[0] < car_power(a[2], v[2], ratio)[0])


if __name__ == '__main__':
    unittest.main()