# Importing necessary libraries
from djitellopy import Tello  # Import the Tello class from the djitellopy library to control the drone
import time  # Import time library for controlling timing-related tasks (though it's not used here)

# Creating a Tello object (this represents the drone)
tello = Tello()

# Establishing a connection to the Tello drone
tello.connect()

# Commanding the drone to take off
tello.takeoff()

# Rotate the drone 90 degrees clockwise
tello.rotate_clockwise(90)

# Move the drone 60 cm upwards
tello.move_up(60)

# Rotate the drone 90 degrees clockwise again
tello.rotate_clockwise(90)

# Move the drone 100 cm to the right
tello.move_right(100)

# Move the drone 60 cm downwards
tello.move_down(60)

# Rotate the drone 180 degrees clockwise (to face the opposite direction)
tello.rotate_clockwise(180)

# Move the drone 100 cm to the right again
tello.move_right(100)

# Command the drone to land after completing the movement commands
tello.land()
