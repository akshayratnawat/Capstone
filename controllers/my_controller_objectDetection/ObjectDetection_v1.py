# # You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor, CameraRecognitionObject
from controller import Robot, Camera
# sensor = robot.getDevice("my_distance_sensor")


def run_robot(robot):
    time_step = 32
    max_speed = 6.28

    # Motors
    left_motor = robot.getDevice('left wheel motor')
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor = robot.getDevice('right wheel motor')
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    camera = robot.getDevice("camera")
    camera.enable(time_step)
    camera.recognitionEnable(time_step)
    camera.enableRecognitionSegmentation()
    

    while robot.step(time_step) != -1:
        left_speed = max_speed
        right_speed = max_speed

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        image = camera.getImage()
        image_width = camera.getWidth
        image_height = camera.getHeight

        
        print("Camera Image value is: ", image_width)
        print("Camera Image value is: ", image_height)

if __name__ == '__main__':
    my_robot = Robot()
    run_robot(my_robot)