from controller import Robot, Motor

class Motors():
    def __init__(self, robot: Robot):
        self.speed =3
        self.left_wheel: Motor = robot.getDevice('left wheel')
        self.right_wheel: Motor = robot.getDevice('right wheel')

        self.left_wheel.setPosition(float('inf'))
        self.right_wheel.setPosition(float('inf'))
        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)

    def avancer(self):
        self.left_wheel.setVelocity(self.speed+2)
        self.right_wheel.setVelocity(self.speed+2)
    def pente(self):
        self.left_wheel.setVelocity(self.speed+1.15)
        self.right_wheel.setVelocity(self.speed+1.05)

    def tourner_gauche(self): 
        self.left_wheel.setVelocity(-self.speed) 
        self.right_wheel.setVelocity(self.speed) 

    def tourner_droite(self): 
        self.left_wheel.setVelocity(self.speed+0.5) 
        self.right_wheel.setVelocity(-self.speed-0.5)
        print(-self.speed-0.5) 

    def stop(self):
        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)

class Pioneer3dx(Robot):
    def __init__(self):
        super().__init__()
        self.motors = Motors(self)
        self.time_step = int(self.getBasicTimeStep())
        
        self.sonar_sensors = []
        sonar_names = ['so0', 'so1', 'so2', 'so3', 'so4', 'so5', 'so6', 'so7', 'so12']
        for name in sonar_names:
            sensor = self.getDevice(name)
            sensor.enable(self.time_step)
            self.sonar_sensors.append(sensor)

    def read_sonars(self):
        return [sensor.getValue() for sensor in self.sonar_sensors]

    def run(self):
        s12 = self.getDevice('so12')
        s12.enable(self.time_step)
        sonar_values = self.read_sonars()
        
        obstacle_threshold = 876 
        
        left_obstacle = any(value > obstacle_threshold for value in sonar_values[0:4])
        right_obstacle = any(value > obstacle_threshold for value in sonar_values[4:8])

        if left_obstacle:
            self.motors.tourner_droite()
        elif right_obstacle:
            self.motors.tourner_gauche()
        else:
            self.motors.avancer()

        if all(value <10 for value in sonar_values[0:8]):

            self.motors.pente()
            #self.motors.avancer()

            print('yohoooooooooo')



        # if s12.getValue() > 900:
        #     self.speed=4
        #     print("s12 = ", s12.getValue())

robot = Pioneer3dx()

while robot.step(robot.time_step) != -1:
    robot.run()