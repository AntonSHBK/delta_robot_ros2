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

            # Координаты целевой точки относительно базовой точки
            x_prime = x + platform_point[0] - base_point[0]
            y_prime = y + platform_point[1] - base_point[1]
            z_prime = z

            # Вычисление горизонтального расстояния до целевой точки
            horizontal_distance = np.sqrt(x_prime**2 + y_prime**2)

            # Вычисление угла плеча
            a = self.upper_arm_length
            b = self.forearm_length
            c = horizontal_distance

            # Проверка достижимости целевой точки
            if c > a + b:
                raise ValueError("Целевая точка вне досягаемости")

            # Использование теоремы косинусов для вычисления угла в горизонтальной плоскости
            cos_theta = (a**2 + c**2 - b**2) / (2 * a * c)
            theta_horizontal = math.acos(cos_theta)

            # Вычисление вертикального угла
            d = np.sqrt(c**2 + z_prime**2)
            cos_alpha = (a**2 + d**2 - b**2) / (2 * a * d)
            alpha = math.acos(cos_alpha)

            theta_i = theta_horizontal - alpha
            theta.append(theta_i)

        return theta

    def forward_kinematics(self, angles):
        """Решение прямой задачи кинематики для заданных углов приводов."""
        theta_rad = angles
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
    base_radius = 500
    platform_radius = 300
    upper_arm_length = 400
    forearm_length = 300

    delta_robot = DeltaRobot(base_radius, platform_radius, upper_arm_length, forearm_length)
    target_position = [-10, 10, 550]

    # Обратная задача кинематики
    try:
        angles = delta_robot.inverse_kinematics(target_position)
        print(f"Углы приводов для достижения позиции {target_position}: {angles}")
    except ValueError as e:
        print(f"Ошибка: {e}")
        
    # Прямая задача кинематики
    joint_angles = [0.1, 0.1, 0.1]  # Пример углов в радианах
    position = delta_robot.forward_kinematics(joint_angles)
    print(f"Позиция для углов приводов {joint_angles}: {position}")
