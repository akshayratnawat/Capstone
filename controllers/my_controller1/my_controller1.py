from controller import Supervisor
import sys

TIME_STEP = 32

supervisor = Supervisor()

# do this once only
robot_node = supervisor.getFromDef("epuck")

trans_field = robot_node.getField("translation")

while supervisor.step(TIME_STEP) != -1:
    # this is done repeatedly
    values = trans_field.getSFVec3f()
    print("MY_ROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))