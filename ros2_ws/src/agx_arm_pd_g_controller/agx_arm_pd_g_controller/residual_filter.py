"""Low-pass filter for the published external-torque estimate.

Why this exists: above the arm's ~15-19 Hz structural resonance the rigid-body
model behind the momentum observer stops being valid, and the observer's exact
cancellation degenerates into a phantom torque

    r_spurious ~ K_I * M_model(q) * q_dot

proportional to velocity, measured at 6.5 Nm.s/rad on joint2 and matching
`K_I * M22` to 3% with no free parameters. Reflected to a leader arm it is
negative damping, and it destabilised closed-loop teleoperation on joint2.

Filtering the residual is the cheap fix because the artefact and the signal are
separated by a decade: the observer's own bandwidth is `momentum_observer_gain`
(10 rad/s = 1.6 Hz by default), so it cannot carry contact information anywhere
near the resonance in the first place. Measured on a stable teleoperation
segment, 92% of the estimate's variance is already below 5 Hz; during the
instability 87-97% of it was above 8 Hz and pure artefact.

Lowering `momentum_observer_gain` instead would work, but trades 1:1 - the
phantom scales with K_I and so does contact-detection latency. A second-order
5 Hz low-pass buys ~10x attenuation at the resonance for ~45 ms of group delay,
where halving K_I buys 2x for 100 ms.
"""

import math

import numpy as np


class LowPassFilter:
    """Per-joint second-order Butterworth low-pass, direct form II transposed.

    Butterworth rather than a single pole because the useful band ends about a
    decade below the artefact: the maximally flat passband leaves the contact
    signal alone while the steeper rolloff does the work at the resonance.
    """

    def __init__(self, cutoff_hz: float, sample_rate_hz: float, n_channels: int):
        if cutoff_hz <= 0.0:
            raise ValueError(f"cutoff_hz must be positive, got {cutoff_hz}")
        if not 0.0 < cutoff_hz < sample_rate_hz / 2.0:
            raise ValueError(
                f"cutoff_hz ({cutoff_hz}) must be below Nyquist "
                f"({sample_rate_hz / 2.0}) for a {sample_rate_hz} Hz sample rate"
            )
        # bilinear transform with frequency pre-warping
        k = math.tan(math.pi * cutoff_hz / sample_rate_hz)
        norm = 1.0 / (1.0 + math.sqrt(2.0) * k + k * k)
        self.b = np.array([k * k * norm, 2.0 * k * k * norm, k * k * norm])
        self.a = np.array([2.0 * (k * k - 1.0) * norm,
                           (1.0 - math.sqrt(2.0) * k + k * k) * norm])
        self.cutoff_hz = cutoff_hz
        self.sample_rate_hz = sample_rate_hz
        self._x1 = np.zeros(n_channels)
        self._x2 = np.zeros(n_channels)
        self._y1 = np.zeros(n_channels)
        self._y2 = np.zeros(n_channels)
        self._primed = False

    def reset(self) -> None:
        self._primed = False

    def __call__(self, x) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        if not self._primed:
            # start from steady state at the first sample, so enabling the
            # filter does not emit a startup transient that reads as a contact
            self._x1 = self._x2 = self._y1 = self._y2 = x.copy()
            self._primed = True
            return x.copy()
        y = (self.b[0] * x + self.b[1] * self._x1 + self.b[2] * self._x2
             - self.a[0] * self._y1 - self.a[1] * self._y2)
        self._x2, self._x1 = self._x1, x.copy()
        self._y2, self._y1 = self._y1, y
        return y.copy()

    def gain_at(self, freq_hz: float) -> float:
        """|H(f)|, for reporting what the chosen cutoff actually does."""
        z = np.exp(-2j * np.pi * freq_hz / self.sample_rate_hz)
        num = self.b[0] + self.b[1] * z + self.b[2] * z * z
        den = 1.0 + self.a[0] * z + self.a[1] * z * z
        return float(abs(num / den))
