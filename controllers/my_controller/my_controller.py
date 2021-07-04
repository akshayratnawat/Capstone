from controller import Robot
import pandas as pd

from controller import Supervisor

def run_robot(robot):
    timestamp = 64
    max_speed = 6

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')

    left_motor.setPosition(float(10))
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float(10))
    right_motor.setVelocity(0.0)

    left_ps = robot.getPositionSensor('left wheel sensor')
    left_ps.enable(timestamp)

    right_ps = robot.getPositionSensor('right wheel sensor')
    right_ps.enable(timestamp)

    ps_values = [0,0]

    df = pd.DataFrame()

    while robot.step(timestamp) != -1:
        ps_values[0] = left_ps.getValue()
        ps_values[1] = right_ps.getValue()
        print(ps_values[0], ps_values[1])

        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(max_speed)

        df = df.append({'Left_Motor_Position':ps_values[0], 'Right_Motor_Position':ps_values[1]}, ignore_index=True)
    
    df.to_csv('df.csv')



if __name__ == '__main__':
    my_robot = Robot()
    run_robot(my_robot)