import math
import numpy as np

class DeltaRobot:
    def __init__(self, base_radius, platform_radius, upper_arm_length, forearm_length):
        self.base_radius = base_radius
        self.platform_radius = platform_radius
        self.upper_arm_length = upper_arm_length
        self.forearm_length = forearm_length

        base_angles = [0, 120, 240]
        platform_angles = [0, 120, 240]
        
        # Точки на неподвижной платформе
        self.base_points = self._calculate_points(base_angles, self.base_radius)
        # Точки на подвижной платформе
        self.platform_points = self._calculate_points(platform_angles, self.platform_radius)

    def _calculate_points(self, angles, radius):
        """Вычисление координат точек на платформе"""
        angles = [0, 120, 240]
        base_points = []
        for angle in angles:
            radians = math.radians(angle)
            x = radius * math.cos(radians)
            y = radius * math.sin(radians)
            base_points.append(np.array([x, y, 0]))
        return base_points

    def inverse_kinematics(self, target_position):
        """Решение обратной задачи кинематики для заданной позиции."""
        x, y, z = target_position
        theta = []

        for i in range(3):
            base_point = self.base_points[i]
            platform_point = self.platform_points[i]

            # Вычисление вектора от основания плеча к целевой точке
            vector = np.array([x, y, z]) + platform_point - base_point

            # Вычисление расстояния от основания плеча до целевой точки
            d = np.linalg.norm(vector)
            if d > (self.upper_arm_length + self.forearm_length):
                raise ValueError("Целевая точка вне досягаемости")

            # Проекция вектора на плоскость XY
            projected_length = np.linalg.norm(vector[:2])

            # Использование теоремы косинусов для вычисления угла
            cos_theta = (self.upper_arm_length**2 + projected_length**2 - self.forearm_length**2) / (2 * self.upper_arm_length * projected_length)
            if not (-1 <= cos_theta <= 1):
                raise ValueError("Целевая точка вне досягаемости")

            theta_i = math.acos(cos_theta)
            theta.append(math.degrees(theta_i))

        return theta
    
    def forward_kinematics(self, angles):
        """Решение прямой задачи кинематики для заданных углов приводов."""
        theta_rad = [math.radians(angle) for angle in angles]
        points = []

        for i in range(3):
            base_point = self.base_points[i]
            angle = theta_rad[i]
            
            # Вычисление положения конца плеча в пространстве
            x = base_point[0] + self.upper_arm_length * math.cos(angle)
            y = base_point[1] + self.upper_arm_length * math.sin(angle)
            z = 0
            points.append(np.array([x, y, z]))

        # Решение системы уравнений для нахождения пересечения трех сфер
        def equations(vars):
            x, y, z = vars
            eqs = []
            for i in range(3):
                arm_end = points[i]
                platform_point = self.platform_points[i]
                eq = (x - arm_end[0])**2 + (y - arm_end[1])**2 + (z - arm_end[2])**2 - self.forearm_length**2
                eqs.append(eq)
            return eqs

        from scipy.optimize import fsolve
        initial_guess = [0, 0, -self.forearm_length]
        solution = fsolve(equations, initial_guess)

        return solution


if __name__ == '__main__':
    # Пример использования
    base_radius = 200
    platform_radius = 50
    upper_arm_length = 100
    forearm_length = 100

    delta_robot = DeltaRobot(base_radius, platform_radius, upper_arm_length, forearm_length)
    target_position = [0, 10, -80]

    # Обратная задача кинематики
    try:
        angles = delta_robot.inverse_kinematics(target_position)
        print(f"Углы приводов для достижения позиции {target_position}: {angles}")
    except ValueError as e:
        print(f"Ошибка: {e}")

    # Прямая задача кинематики
    angles = [45, 45, 45]
    position = delta_robot.forward_kinematics(angles)
    print(f"Позиция для углов приводов {angles}: {position}")
