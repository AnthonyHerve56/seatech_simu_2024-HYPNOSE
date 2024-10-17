from controller import Robot, Motor, DistanceSensor, GPS
import math

class BobRobot(Robot):

    
    def __init__(self):
        super().__init__()
        self.seeking = False
        self.compt = 0


        self.end_X = -43.5 
        self.end_Y = 48   
        self.left_wheel: Motor = self.getDevice('left wheel')
        self.right_wheel: Motor = self.getDevice('right wheel')

        self.sens0: DistanceSensor = self.getDevice('so0')
        self.sens1: DistanceSensor = self.getDevice('so1')
        self.sens2: DistanceSensor = self.getDevice('so2')
        self.sens3: DistanceSensor = self.getDevice('so3')
        self.sens4: DistanceSensor = self.getDevice('so4')
        self.sens5: DistanceSensor = self.getDevice('so5')
        self.sens6: DistanceSensor = self.getDevice('so6')
        self.sens7: DistanceSensor = self.getDevice('so7')

        self.compass = self.getDevice('compass')
        self.compass.enable(int(self.getBasicTimeStep()))

        self.gps = self.getDevice('gps')
        self.gps.enable(int(self.getBasicTimeStep()))

        self.sens0.enable(int(self.getBasicTimeStep()))
        self.sens1.enable(int(self.getBasicTimeStep()))
        self.sens2.enable(int(self.getBasicTimeStep()))
        self.sens3.enable(int(self.getBasicTimeStep()))
        self.sens4.enable(int(self.getBasicTimeStep()))
        self.sens5.enable(int(self.getBasicTimeStep()))
        self.sens6.enable(int(self.getBasicTimeStep()))
        self.sens7.enable(int(self.getBasicTimeStep()))

        self.left_wheel.setPosition(float('inf'))
        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setPosition(float('inf'))
        self.right_wheel.setVelocity(0.0)

    def straight(self):
        self.left_wheel.setVelocity(5.0)
        self.right_wheel.setVelocity(5.0)

    def right(self):
        self.left_wheel.setVelocity(5.0)
        self.right_wheel.setVelocity(-5.0)

    def left(self):
        self.left_wheel.setVelocity(-5.0)
        self.right_wheel.setVelocity(5.0)

    def stop(self):
        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)
    
    def run(self):
        c = self.compass.getValues()
    
        g = self.gps.getValues()
        current_x = g[0]
        current_y = g[1]
        print (current_x , " : " , current_y)

        direction_fin = math.atan2(self.end_Y - current_y, self.end_X - current_x)

        current_angle = math.atan2(c[0], c[1])

        angle_diff = direction_fin - current_angle

        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
 
        s0 = self.sens0.getValue()
        s1 = self.sens1.getValue()
        s2 = self.sens2.getValue()
        s3 = self.sens3.getValue()
        s4 = self.sens4.getValue()
        s5 = self.sens5.getValue()
        s6 = self.sens6.getValue()
        s7 = self.sens7.getValue()

        if self.seeking:
            if abs(angle_diff) > 0.6:  
                if angle_diff > 0:
                    self.left()  
                else:
                    self.right()  
            else:
                self.straight()
                self.seeking = False
        else:
            if s3 < 900 and s4 < 900 and s2 < 900 and s5 < 900 and s1 < 500 and s6 < 500:
                self.compt += 1
                self.straight()
                if self.compt > 150:
                    self.seeking = True
            else:
                if ((s1 + s2 + s0) / 3) >= ((s5 + s6 + s7) / 3):  
                    self.right()
                else:  
                    self.left()

        print("Compass: ", current_angle)
        print("Angle diff: ", angle_diff)
        print("sensors: ", s3, s4, s2, s5)



robot = BobRobot()

#timestep = int(robot.getBasicTimeStep())
timestep = 40



while robot.step(timestep) != -1:
    robot.run()