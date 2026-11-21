"""Joint path as an ABB controller runs it: MoveJ = straight lines in joint space between the targets, corner
paths in the zones (review 1.5, 21 Nov 2026; RobotStudio is not used, this rebuilds the controller behaviour).

RAPID the model stands for (loaded transfer, chain 2; the empty way back is the same with WorldAccLim off):

    WorldAccLim \\On := 4.96;                      ! tool + payload acceleration in world <= 4.96 m/s2 (grip sizing)
    MoveJ pPickUp,  v1000, fine, tGripper \\WObj:=wBelt;
    MoveJ pVia2,    v1000, zVIA, tGripper;          ! retracted mid-swing, corner path, no stop
    MoveJ pPlaceUp2,v1000, fine, tGripper \\WObj:=wChain2;
    WorldAccLim \\Off;

zone model: the corner path starts where the TCP comes within the zone radius of the via along the incoming MoveJ
and ends where it leaves that radius on the outgoing one (ABB halves the zone if it is larger than half a
segment). In between the joints follow a quadratic Bezier with the via joint values as control point: tangent to
both lines, inside their corner, no axis overshoots - the shape of the ABB joint corner path. TCP speed and
acceleration along it come from sim.TimeLaw (WorldAccLim = cap on the TCP acceleration, tangential + centripetal).
"""
import numpy as np

VMAX = np.radians([249.0, 180.0, 247.0, 500.0, 415.0, 720.0])


class MoveJPath:
    """q(u, nu) for u in 0 ... 1 through joint targets qs (rad); zones[i] = zone radius [mm] at the inner target i
    (0 = fine, passes through). tcp(q) -> TCP position [m] is needed to place the corner paths."""

    def __init__(self, qs, zones, tcp):
        qs = [np.asarray(q, float) for q in qs]
        self.pieces = []                                   # (kind, data, joint length for the knots)
        start = qs[0]
        for i in range(1, len(qs) - 1):
            z = zones[i - 1] / 1000.0
            if z <= 0:
                self._line(start, qs[i])
                start = qs[i]
                continue
            qa = self._at_radius(qs[i], qs[i - 1], z, tcp)
            qb = self._at_radius(qs[i], qs[i + 1], z, tcp)
            self._line(start, qa)
            self.pieces.append(("bez", (qa, qs[i], qb), np.max(np.abs(qb - qa) / VMAX)))
            start = qb
        self._line(start, qs[-1])
        w = np.array([max(p[2], 1e-6) for p in self.pieces])
        self.knots = np.concatenate([[0.0], np.cumsum(w)]) / w.sum()

    def _line(self, a, b):
        if np.max(np.abs(b - a)) > 1e-9:
            self.pieces.append(("line", (a, b), np.max(np.abs(b - a) / VMAX)))

    @staticmethod
    def _at_radius(qv, qo, z, tcp, n=200):
        """Point on the joint line qv -> qo where the TCP is z away from the via's TCP (at most half the segment)."""
        pv = tcp(qv)
        s = np.linspace(0.0, 1.0, n)
        p = np.array([tcp(qv + si * (qo - qv)) for si in s])
        arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
        z = min(z, 0.5 * arc[-1])
        dist = np.linalg.norm(p - pv, axis=1)
        k = int(np.argmax(dist >= z)) if np.any(dist >= z) else n - 1
        return qv + s[k] * (qo - qv)

    def __call__(self, u, nu=0):
        u = np.asarray(u, float)
        scalar = u.ndim == 0
        u = np.atleast_1d(u)
        out = np.zeros((len(u), len(self.pieces[0][1][0])))
        idx = np.clip(np.searchsorted(self.knots, u, side="right") - 1, 0, len(self.pieces) - 1)
        for j, (kind, data, _) in enumerate(self.pieces):
            m = idx == j
            if not m.any():
                continue
            du = self.knots[j + 1] - self.knots[j]
            s = ((u[m] - self.knots[j]) / du)[:, None]
            if kind == "line":
                a, b = data
                out[m] = a + s * (b - a) if nu == 0 else np.broadcast_to((b - a) / du, (int(m.sum()), len(a)))
            else:
                a, v, b = data
                if nu == 0:
                    out[m] = (1 - s) ** 2 * a + 2 * s * (1 - s) * v + s ** 2 * b
                else:
                    out[m] = (2 * (1 - s) * (v - a) + 2 * s * (b - v)) / du
        return out[0] if scalar else out
