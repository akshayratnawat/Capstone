# # """my_controller_objectDetection controller."""

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

        value = camera.getImageArray()
        recog = camera.hasRecognition()
        print("Sensor value is: ", recog)
        # firstObject = camera.getRecognitionObjects()
        # # id = firstObject.get_id()
        # # position = firstObject.get_position()
        # print("Sensor value is: ", firstObject)

if __name__ == '__main__':
    my_robot = Robot()
    run_robot(my_robot)


# # from controller import Robot, DistanceSensor

# # TIME_STEP = 32

# # robot = Robot()

# # camera = robot.getDevice("camera")
# # camera.enable(TIME_STEP)

# # while robot.step(TIME_STEP) != -1:
# #     hasRecognition
# #     value = camera.getImageArray()
# #     print("Sensor value is: ", value)

# import cv2
# import numpy as np
# from controller import Robot


# # P value for P controller
# # High P value makes the robot more agressive
# # Low P values makes the robot more sluggish
# P_COEFFICIENT = 0.1

# # Initialize the robot
# robot = Robot()
# timestep = int(robot.getBasicTimeStep())

# # Initialize camera
# camera = robot.getDevice('camera')
# camera.enable(timestep)

# # Initialize motors
# motor_left = robot.getDevice('left wheel motor')
# motor_right = robot.getDevice('right wheel motor')
# motor_left.setPosition(float('inf'))
# motor_right.setPosition(float('inf'))
# motor_left.setVelocity(0)
# motor_right.setVelocity(0)


# def get_image_from_camera():
#     """
#     Take an image from the camera device and prepare it for OpenCV processing:
#     - convert data type,
#     - convert to RGB format (from BGRA), and
#     - rotate & flip to match the actual image.
#     """
#     img = camera.getImageArray()
#     img = np.asarray(img, dtype=np.uint8)
#     img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
#     img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
#     return cv2.flip(img, 1)


# # Main control loop
# while robot.step(timestep) != -1:
#     img = get_image_from_camera()

#     # Segment the image by color in HSV color space
#     img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
#     mask = cv2.inRange(img, np.array([50, 150, 0]), np.array([200, 230, 255]))

#     # Find the largest segmented contour (red ball) and it's center
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#     print(contours)
#     largest_contour = max(contours, key=cv2.contourArea)
#     largest_contour_center = cv2.moments(largest_contour)
#     center_x = int(largest_contour_center['m10'] / largest_contour_center['m00'])

#     # Find error (ball distance from image center)
#     error = camera.getWidth() / 2 - center_x

#     # Use simple proportional controller to follow the ball
#     motor_left.setVelocity(- error * P_COEFFICIENT)
#     motor_right.setVelocity(error * P_COEFFICIENT)