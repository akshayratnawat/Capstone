# # You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, CameraRecognitionObject, Camera
import numpy as np

def get_sensor_values(robot):
    time_step = int(robot.getBasicTimeStep())
    # Enabling Proximity Sensors
    prox_sensors = []
    for i in range(8):
        sensor_name = 'ps'+ str(i)
        prox_sensors.append(robot.getDevice(sensor_name))
        prox_sensors[i].enable(time_step)
    
    return prox_sensors

def run_robot(robot):
    time_step = int(robot.getBasicTimeStep())
    max_speed = 6.28

    # Enable Motors
    left_motor = robot.getDevice('left wheel motor')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor = robot.getDevice('right wheel motor')
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    # Enable Camera
    camera = robot.getDevice("camera_top")
    camera.enable(time_step)
    camera.recognitionEnable(time_step)
    camera.enableRecognitionSegmentation()


    # Enabling Proximity Sensors
    prox_sensors = get_sensor_values(robot)

    while robot.step(time_step) != -1:

        left_speed = -max_speed
        right_speed = max_speed
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        # PRinting the proximity sensor values
        sensor_values_print = []
        for i in range(8):
            sensor_values_print.append(prox_sensors[i].getValue())
        print(sensor_values_print)


        if len(camera.getRecognitionObjects()) > 0:
            print('Object Detected')
            firstObject = camera.getRecognitionObjects()[0]
            position_on_image = firstObject.get_position_on_image()
            x = position_on_image[0]
            y = position_on_image[1]
            print(x/y)
            # if 20<= x <=30:
            if ( x/y >= 0.5) :
                print('Moving Towards the object')
                left_speed = max_speed
                right_speed = max_speed
                if max(sensor_values_print) >= 78:
                    left_speed = 0
                    right_speed = 0
                    camera.disableRecognitionSegmentation()
                    print('Stopped')
                   # flag = False
                    move_around_object(robot)
        else:
            print('Object not Detected')

        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        #print(flag)
    #return flag 
    
def move_around_object(robot):
    time_step = int(robot.getBasicTimeStep())
    max_speed = 6.28

    # Enable Motors
    left_motor = robot.getDevice('left wheel motor')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor = robot.getDevice('right wheel motor')
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    # Enabling IR Sensors
    left_ir = robot.getDevice('IR_left')
    left_ir.enable(time_step)

    right_ir = robot.getDevice('IR_right')
    right_ir.enable(time_step)
    

    while robot.step(time_step) != -1:

        left_speed = -max_speed
        right_speed = max_speed
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        # Enabling Proximity Sensors
        prox_sensors = get_sensor_values(robot)
        sensor_values_print = []
        for i in range(8):
            sensor_values_print.append(prox_sensors[i].getValue())
        #print(sensor_values_print)
        
        print(left_ir.getValue(), right_ir.getValue())

        # Defining the obstacles 
        left_obstacle = (prox_sensors[5].getValue() > 80)
        left_corner_obstacle = prox_sensors[6].getValue() > 80

        right_obstacle = prox_sensors[2].getValue() > 80
        #right_corner_obstacle = prox_sensors[1].getValue() > 80

        front_obstacle = (prox_sensors[7].getValue() >= 80)
        #back_obstacle = prox_sensors[3].getValue() > 80
        
        left_speed = max_speed
        right_speed = max_speed

        if front_obstacle:
            print("Turn Right in place")
            left_speed = max_speed
            right_speed = -max_speed
        else:
            if left_obstacle:
                    print("Drive Forward")
                    left_speed = max_speed
                    right_speed = max_speed
            else:
                print("Turn Left")
                left_speed = max_speed/8
                right_speed = max_speed 
            if left_corner_obstacle:
                print('Came too close, drive right')
                left_speed = max_speed
                right_speed = max_speed/4
            if front_obstacle:
                print('Came too close, drive back')
                left_speed = -max_speed
                right_speed = -max_speed    
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
                        

if __name__ == '__main__':
    my_robot = Robot()
    move_around_object(my_robot)

        