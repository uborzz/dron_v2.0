class KalmanFilter:

    def __init__(self, q, r):
        self.q = q
        self.r = r
        self.xhat = 0
        self.xhat2 = 0
        self.xhatminus = None
        self.p = 1.0
        self.pminus = None
        self.kG = 0

    def predict(self):
        self.xhatminus = self.xhat
        self.pminus = self.p + self.q

    def correct(self, x):
        self.kG = self.pminus / (self.pminus + self.r)
        self.xhat = self.xhatminus + self.kG * (x - self.xhatminus)
        self.p = (1 - self.kG) * self.pminus
        return self.xhat

    def predict_and_correct(self, x):
        self.predict()
        return self.correct(x)

    def change_q(self, valor):
        self.q = valor

    def change_r(self, valor):
        self.r = valor