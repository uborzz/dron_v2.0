import kalman_jc

class kalman_circular(kalman_jc.KalmanFilter):
    """Filtro de kalman para usar con grados sexagesimales, que tiene en cuenta que 0-360 es la misma cosa"""

    def __init__(self, q, r):
        self._last_value = 0
        super().__init__(q, r)

    def _enrollar(self, x):
        while (x - self._last_value) > 180:
            x -= 360
        while (x - self._last_value) <= -180:
            x += 360
        return x

    def _desenrollar(self):
        x = int(self._last_value)
        while x >= 360:
            x -= 360
        while x < 0:
            x += 360
        return x

    def predict_and_correct(self, x):
        self.predict()
        x_con_vueltas = self._enrollar(x)
        self._last_value = self.correct(x_con_vueltas)
        return self._desenrollar()

