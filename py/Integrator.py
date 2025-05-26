class Integrator:
    def __init__(self, x0, T):
        self.x0 = 0
        self.T = T # время итерации (шаг интегрирования)
        self.integral = x0
        self.first_update = True
    pass

    def update(self, val: float) -> float:
        if self.first_update:
            #Для первого шага используем метод прямоуг
            self.integral = 0 + val * self.T #Т - delta(t) шаг интегрирования
            self.first_update = False
        else:
            #Метод трапеций
            self.integral += (self.prev_val + val) * self.T / 2

        self.prev_val = val  # сохраняем текущее значение для следующей итерации
        return self.integral
