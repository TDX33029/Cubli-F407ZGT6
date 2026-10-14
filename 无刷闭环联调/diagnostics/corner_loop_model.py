"""Closed-loop linear stability model of the corner (single-point) balance loop.

Built directly from the firmware structure in
Cubli-ST/Hardware/lqr_balance.c (LQR_Balance_Loop, corner branch) so gain
sweeps here match what the board actually runs:

  err   = g x ref                      (cross3(g, ref, err), scaled to angle)
  u_i   = d_i*body_i - Kw*W_i - Ki*xi_i - Kti*tau_i        (clamped +-6V on hw)
  body_i= K_th*err[axis_i] + K_dt*rate_i + K_yaw*rate_yaw*ref[axis_i]
  rate_i= w[axis_i] - rate_yaw*ref[axis_i],  rate_yaw = w . ref

Plant (from diagnostics/2026-10-14/motor_pulse_identification.json):
  Wdot_i = c*sigma_i*u_i - drag*W_i - coulomb*sign(W_i)     [linear part only]
  wdot   = a*P*p - gamma*c*sum_i u_i*n_i                    [reaction wheels]
  pdot   = R x w                                            [gdot = g x w]
  xi_i   = integral of W_i,  tau_i = integral of err[axis_i]

  p = g - R,  P = I - R*R',  n_i = sigma_i * e[axis_i]  (wheel +axis)
  a = g/L_eff, gamma = J_w/J_b, c = voltage-to-acceleration gain, drag = 1/s.

The three sign factors d_i (firmware body_direction, -1 on hardware) and
n_i (which way a positive u spins the wheel about its IMU axis) only matter
through the product chain d_i*sigma_i; the working configuration is the one
that keeps the fast attitude modes stable, and the model resolves it at
startup.  Run: python diagnostics/corner_loop_model.py
"""
import numpy as np

R = np.array([-0.7028287890572913, 0.3977869310324257, 0.5897433770480731])
A_GRAV = 156.3      # g / L_eff [1/s^2]
GAMMA = 0.0510      # J_wheel / J_body
C_GAIN = 130.97     # rad/s^2 per volt (identification)
DRAG = 0.612        # 1/s (identification)
AXIS = [2, 3, 1]    # body_axis: motor 1 -> IMU Y, 2 -> Z, 3 -> X (1=X,2=Y,3=Z)
DIR = [-1, -1, -1]  # body_direction


def unit_axis(axis):
    v = np.zeros(3)
    v[axis - 1] = 1.0
    return v


E = [unit_axis(a) for a in AXIS]
P = np.eye(3) - np.outer(R, R)


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def build(d=None, s=None, kp=80.0, kd=5.0, kw=-0.03, ki=0.0, kti=0.0, kyaw=3.0):
    """15-state closed loop.  d = body_direction (firmware), s = the wheel
    +axis orientation in body frame (fixes the reaction-torque sign)."""
    if d is None:
        d = DIR
    if s is None:
        s = S_SIGN
    n = 15
    A = np.zeros((n, n))
    # pdot = R x w
    A[0:3, 3:6] = skew(R)
    # wdot = a*P*p - gamma*c*sum_i u_i*s_i*E_i  (reaction from +W_i torque)
    A[3:6, 0:3] = A_GRAV * P
    # wheel_int and tilt_int dynamics
    for i in range(3):
        A[6 + i, 6 + i] = -DRAG
        A[9 + i, 6 + i] = 1.0                     # xidot = W
        A[12 + i, 0:3] = (-skew(R))[AXIS[i] - 1, :]  # taudot = err[axis]
    # control rows: u_i = d_i*(Kth*err_i + Kd*rate_i + Kyaw*ry*R[axis])
    #                    - Kw*W_i - Ki*xi_i - Kti*tau_i,  err_i = (-skew(R)p)[axis]
    for i in range(3):
        axi = AXIS[i] - 1
        cu = np.zeros(n)
        cu[0:3] = d[i] * kp * (-skew(R))[axi, :]
        cu[3:6] = d[i] * kd * np.eye(3)[axi, :] + d[i] * (kyaw - kd) * R * R[axi]
        cu[6 + i] -= kw
        cu[9 + i] -= ki
        cu[12 + i] -= kti
        # wheel:  Wdot += c*u_i  (motor/encoder sign convention, +u spins +W)
        A[6 + i, :] += C_GAIN * cu
        # body reaction:  wdot += -gamma*c*u_i*s_i*E_i
        A[3:6, :] += -GAMMA * C_GAIN * s[i] * np.outer(E[i], cu)
    return A


def resolve_sign():
    """Pick the wheel-axis orientation s whose fast attitude modes are stable
    (the on-hardware loop balances, so its fast modes must be stable)."""
    best = None
    for s in (1.0, -1.0):
        ev = np.linalg.eigvals(build(s=[s, s, s]))
        worst = ev.real.max()
        print(f"  s={s:+.0f}: worst real part {worst:+.3f} "
              f"(fast poles: {', '.join(f'{v.real:+.2f}{v.imag:+.2f}j' for v in sorted(ev, key=lambda z: -abs(z.real))[:3])})")
        if best is None or worst < best[0]:
            best = (worst, s)
    print(f"  -> using s={best[1]:+.0f}")
    return [best[1]] * 3


def modes(A, top=6):
    ev = np.linalg.eigvals(A)
    ev = sorted(ev, key=lambda z: -z.real)
    return ev[:top]


def summary(label, A):
    ev = modes(A)
    worst = ev[0].real
    txt = ", ".join(f"{v.real:+.3f}{v.imag:+.3f}j" for v in ev)
    print(f"{label:34s} worst={worst:+7.3f}  {txt}")
    return worst


S_SIGN = resolve_sign()
print()

if __name__ == "__main__":
    print("== baseline: old committed config (80/5/-0.03/0) vs recommended (80/5/0/0) ==")
    summary("Kw=-0.03 Ki=0 Kti=0 Kyaw=3", build())
    summary("Kw= 0.00 Ki=0 Kti=0 Kyaw=3", build(kw=0.0))
    print()
    print("== Kw scan (Ki=0, Kti=0, Kyaw=3): slow mode peaks around 0 ==")
    for kw in (-0.06, -0.05, -0.04, -0.03, -0.02, -0.01, 0.0, 0.01, 0.02, 0.05, 0.08, 0.12):
        summary(f"Kw={kw:+.3f}", build(kw=kw))
    print()
    print("== Kp/Kd scan at Kw=0 ==")
    for kp, kd in ((80, 5), (100, 5), (120, 5), (80, 8), (120, 8), (150, 10)):
        summary(f"Kp={kp} Kd={kd}", build(kp=kp, kd=kd, kw=0.0))
    print()
    print("== Ki scan at Kw=0 (both signs destabilize) ==")
    for ki in (-0.02, -0.01, 0.01, 0.02):
        summary(f"Ki={ki:+.3f}", build(ki=ki, kw=0.0))
    print()
    print("== Kti scan at Kw=0 (mild, stable) ==")
    for kti in (0.5, 2.0, 5.0, 10.0):
        summary(f"Kti={kti:.1f}", build(kti=kti, kw=0.0))
    print()
    print("== Kyaw scan at Kw=0 (keep 3; at Kw=-0.03, Kyaw=0 was +3.3/s unstable) ==")
    for kyaw in (0.0, 1.0, 3.0, 10.0):
        summary(f"Kyaw={kyaw:.1f}", build(kyaw=kyaw, kw=0.0))
    print()
    print("Recommendation: corner gains 80/5/0.0/0 with BYAW=3; keep Ki=0.  The exact-zero")
    print("eigenvalues are the dormant Kti/Ki integrators when both are 0, not dynamics.")
