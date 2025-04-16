class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        """
        初始化 PID 控制器.

        参数:
        Kp: 比例增益
        Ki: 积分增益
        Kd: 微分增益
        setpoint: 目标值 (默认是 0)
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self.previous_error = 0
        self.integral = 0

    def update(self, feedback_value, dt):
        """
        更新 PID 控制器并计算输出.

        参数:
        feedback_value: 当前反馈值
        dt: 时间间隔

        返回:
        控制器输出
        """
        # 计算误差
        error = self.setpoint - feedback_value

        # 计算积分部分
        self.integral += error * dt

        # 计算微分部分
        derivative = (error - self.previous_error) / dt

        # 计算 PID 输出
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # 保存误差以备下次计算微分部分
        self.previous_error = error

        return output

    def reset(self):
        """
        清空积分项和上一次的误差
        """
        self.integral = 0
        self.previous_error = 0
    