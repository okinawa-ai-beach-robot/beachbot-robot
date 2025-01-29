import time, math
import scipy

def sgn(val):
    return -1 if val<0 else (1 if val > 0 else 0)

def safe_div(a,b):
    return b and a / b or 0


class Timer:    
    # https://stackoverflow.com/questions/1058813/on-line-iterator-algorithms-for-estimating-statistical-median-mode-skewnes
    # https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#On-line_algorithm
    def __init__(self, p=0.95) -> None:
        self.K=0
        self.Ex=0
        self.Ex2=0
        self.n=0
        self.median=0
        self.quantile=0
        self.p=p
        self.interval=-1

    def _add_sample(self, x):

        if self.n == 0:
            self.K = x
            self.median = x
        self.n += 1
        self.Ex += x - self.K
        self.Ex2 += (x - self.K) ** 2
        self.median += 10e-3 * sgn(x-self.median)
        self.quantile += 10e-3 * (sgn(x - self.quantile) + 2.0 * self.p - 1.0)


    def get_mean(self):
        return self.K + safe_div(self.Ex, self.n) 
    
    def get_conf_interval(self, confidence=0.95):
        try:
            se = safe_div(math.sqrt(self.get_variance()), math.sqrt(self.n))
            return se * scipy.stats.t.ppf((1 + confidence) / 2., self.n-1)
        except ValueError as ex:
            return 0

   
    
    def get_variance(self):
        return safe_div(safe_div(self.Ex2 - self.Ex**2 , self.n), self.n - 1)
        # return (self.Ex2 - self.Ex**2 / self.n) / (self.n - 1)
    
    def get_quantile(self):
        return self.quantile
    
    def get_median(self):
        return self.median
    
    def get_last_interval(self):
        return self.interval
    

    def __enter__(self):
        self.start = time.perf_counter()
        return self

    def __exit__(self, *args):
        self.end = time.perf_counter()
        self.interval = self.end - self.start
        self._add_sample(self.interval)

    def __repr__(self):
        return f"<Timer median:{self.median} quantile:{self.quantile} n:{self.n} p:{self.p} K:{self.K} Ex:{self.Ex} Ex2:{self.Ex2}>"

    def __str__(self):
        return f"{self.get_mean()}±{self.get_quantile()} (median={self.get_median()}, n={self.n}, p={self.p})"
