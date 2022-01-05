class PID:

    def __init__(self, Kp, Kd, Ki, ref, limit):
        self.Kd = Kd
        self.Kp = Kp
        self.Ki = Ki
        self.ref = ref
        self.prev_err = None
        self.err = None
        self.int_err = None
        self.dt = 1 / 1000
        self.output = None
        self.limit = limit

    def computeOutput(self, reading):
        self.err = self.ref - reading

        if self.prev_err is None:
            self.prev_err = self.err

        if self.int_err is None:
            self.int_err = self.err
        else:
            self.int_err = self.int_err + self.err

        self.output = (self.Kp * self.err) + (self.Kd * (self.err - self.prev_err) / self.dt) + (self.Ki * self.int_err)

        # Saturate
        if abs(self.output) > self.limit:
            if self.output < 0:
                self.output = -self.limit
            else:
                self.output = self.limit

        return self.output
