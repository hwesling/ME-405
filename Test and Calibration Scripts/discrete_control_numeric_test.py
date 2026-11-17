from taskmotor import ClosedLoop

KP = 0.5
KI_S = 0.04
DT = 0.05
V_MIN = -7.2
V_MAX = 7.2
TOL = 0.00001


def check(name, voltage, expected_voltage, integral, expected_integral):
    voltage_ok = abs(voltage - expected_voltage) < TOL
    integral_ok = abs(integral - expected_integral) < TOL

    if voltage_ok and integral_ok:
        print(name + ": PASS")
    else:
        print(name + ": FAIL")
        print("  voltage:", voltage, "expected:", expected_voltage)
        print("  integral:", integral, "expected:", expected_integral)


controller = ClosedLoop(V_MIN, V_MAX)

# e = 2.0 rad/s; no saturation; I becomes 0.08 V.
controller.reset()
voltage = controller.run(2.0, 0.0, KP, KI_S, DT)
check("Unsaturated integration",
      voltage, 1.0,
      controller.integral_voltage, 0.08)

# e = +2.0; upper-limited command; hold I at 8.0 V.
controller.integral_voltage = 8.0
voltage = controller.run(2.0, 0.0, KP, KI_S, DT)
check("Upper saturation holds integral",
      voltage, 7.2,
      controller.integral_voltage, 8.0)

# e = -0.5; still upper-limited, but error unwinds I.
controller.integral_voltage = 8.0
voltage = controller.run(-0.5, 0.0, KP, KI_S, DT)
check("Upper saturation allows unwinding",
      voltage, 7.2,
      controller.integral_voltage, 7.98)

# e = -2.0; lower-limited command; hold I at -8.0 V.
controller.integral_voltage = -8.0
voltage = controller.run(-2.0, 0.0, KP, KI_S, DT)
check("Lower saturation holds integral",
      voltage, -7.2,
      controller.integral_voltage, -8.0)

# e = +0.5; still lower-limited, but error unwinds I.
controller.integral_voltage = -8.0
voltage = controller.run(0.5, 0.0, KP, KI_S, DT)
check("Lower saturation allows unwinding",
      voltage, -7.2,
      controller.integral_voltage, -7.98)

# Reset test.
controller.integral_voltage = 3.0
controller.reset()

if controller.integral_voltage == 0.0:
    print("Reset: PASS")
else:
    print("Reset: FAIL")