import numpy as np


class CommunicationLink:
    """
    Generic range-dependent communication model.
    Probability of receiving an update decreases with separation.
    """
    def __init__(self, nominal_range=55.0, seed=8):
        self.nominal_range = nominal_range
        self.rng = np.random.default_rng(seed)

    def success_probability(self, separation):
        if separation <= 0.5 * self.nominal_range:
            return 0.995
        if separation >= 1.4 * self.nominal_range:
            return 0.20
        s = (separation - 0.5*self.nominal_range) / (0.9*self.nominal_range)
        return float(np.clip(0.995 - 0.795*s, 0.20, 0.995))

    def transmit(self, separation):
        p = self.success_probability(separation)
        return bool(self.rng.random() < p), p
