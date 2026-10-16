import json
import math as m
import time

import keyboard
import pygame
from simple_pid import PID


P = 1
D = 0.1
pidTurn = PID(0.022, 0.0, 0.004, setpoint=0)
pidTurn.output_limits = (-1.0, 1.0)
pidTurn.sample_time = 0.01
pidX = PID(P, 0, D, setpoint=0)
pidX.output_limits = (-1.0, 1.0)
pidY = PID(P, 0, D, setpoint=0)
pidY.output_limits = (-1.0, 1.0)


def turn(start_x, start_y, angle):
    magnitude = m.hypot(start_x, start_y)
    current_angle = m.degrees(m.atan2(start_x, start_y))
    rotation = m.radians(angle - current_angle)
    return [
        m.cos(rotation) * magnitude,
        m.sin(rotation) * magnitude,
    ]


def go_to(target_x, target_y, current_x, current_y):
    pidX.setpoint = target_x
    pidY.setpoint = target_y
    return [pidX(current_x), pidY(current_y)]


def turn_to(current_rotation, target_rotation):
    pidTurn.setpoint = target_rotation
    return pidTurn(current_rotation)


def write_controls(turn_value, x_movement, y_movement, outtake, intake, shoot, stop, reset, lockElevator, elevatorUp, elevatorDown, rotateHoodForward, rotateHoodBackward):
    controls = {
        "a": 0,
        "b": int(outtake), #outtake
        "x": int(intake), #intake
        "y": int(shoot), #shoot
        "dpad_left": int(lockElevator),
        "dpad_right": int(elevatorUp), 
        "dpad_up": int(elevatorUp), #elevator up
        "dpad_down": int(elevatorDown), #elevator down
        "bumper_l": 0,
        "bumper_r": 0,
        "stop": int(stop),
        "restart": int(reset),
        "right_y": 0,
        "right_x": turn_value,
        "left_y": y_movement,
        "left_x": x_movement,
        "trigger_l": int(rotateHoodForward), #hood forward
        "trigger_r": int(rotateHoodBackward), #hood backward
        "precision": 0,
    }
    with open("Controls.txt", "w") as controls_file:
        controls_file.write(
            "\n".join(f"{name}={value}" for name, value in controls.items())
        )


pygame.init()
joysticks = []
for joystick_index in range(pygame.joystick.get_count()):
    joystick = pygame.joystick.Joystick(joystick_index)
    joystick.init()
    joysticks.append(joystick)

controller_connected = bool(joysticks)
x_movement = 0
y_movement = 0
turn_value = 0
precision_drive = False
steer_to_hub = False

while True:
    start = time.time()

    try:
        with open("myRobot.txt", "r") as robot_file:
            robot = json.load(robot_file)
    except (OSError, json.JSONDecodeError):
        continue

    robot_position = robot["myrobot"][1]["global pos"]
    robot_velocity = robot["myrobot"][1]["velocity"]
    robot_rotation = robot["myrobot"][1]["global rot"]

    distance_to_hub = m.hypot(robot_position[0], robot_position[2])
    hub_angle = m.degrees(m.atan2(robot_position[0], robot_position[2])) + 90
    if hub_angle < 0:
        hub_angle += 360

    speed = m.hypot(robot_velocity[0], robot_velocity[2])
    velocity_angle = m.degrees(
        m.atan2(robot_velocity[0], robot_velocity[2])
    ) + 90
    if velocity_angle < 0:
        velocity_angle += 360

    difference = (hub_angle - velocity_angle) % 360
    y_velocity = m.cos(m.radians(difference)) * speed
    x_velocity = m.sin(m.radians(difference)) * speed
    lead_time = 1.25
    x_distance = x_velocity * lead_time
    y_distance = y_velocity * lead_time
    relative_distance = m.hypot(y_distance + distance_to_hub, x_distance)
    if relative_distance == 0:
        lead_rotation = 0
    else:
        lead_rotation = m.degrees(
            m.sin(x_distance * 1.4 / relative_distance)
        )

    for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 0:
                steer_to_hub = True
            elif event.button == 4:
                precision_drive = True
        elif event.type == pygame.JOYBUTTONUP:
            if event.button == 0:
                steer_to_hub = False
            elif event.button == 4:
                precision_drive = False
        elif event.type == pygame.JOYAXISMOTION:
            if event.axis == 0:
                x_movement = event.value
                if abs(event.value) < 0.1:
                    x_movement = 0
            elif event.axis == 1:
                y_movement = event.value
                if abs(event.value) < 0.1:
                    y_movement = 0
            elif event.axis == 2:
                turn_value = event.value * 0.7
                if abs(event.value) < 0.1:
                    turn_value = 0

    if keyboard.is_pressed("w"):
        y_movement = -1
    elif keyboard.is_pressed("s"):
        y_movement = 1
    elif not controller_connected:
        y_movement = 0

    if keyboard.is_pressed("a"):
        x_movement = -1
    elif keyboard.is_pressed("d"):
        x_movement = 1
    elif not controller_connected:
        x_movement = 0

    if keyboard.is_pressed("j"):
        turn_value = -0.7
    elif keyboard.is_pressed("l"):
        turn_value = 0.7
    elif not controller_connected:
        turn_value = 0

    current_rotation = robot_rotation[1]
    target_rotation = 0
    auto_steering = False

    if keyboard.is_pressed("shift") or precision_drive:
        if relative_distance < 4:
            y_movement *= 0.4
        elif relative_distance < 6:
            y_movement *= 0.6
        else:
            y_movement *= 0.8
        if distance_to_hub > 3.2 and abs(y_movement) < 0.1:
            y_movement -= 0.2
        turn_value *= 0.3
        x_movement *= 0.5
        target_rotation = hub_angle - lead_rotation
        auto_steering = True
        if x_movement != 0 or y_movement != 0:
            x_movement, y_movement = turn(
                x_movement,
                y_movement,
                hub_angle - current_rotation + 90,
            )

    if keyboard.is_pressed(";") or steer_to_hub:
        target_rotation = hub_angle
        auto_steering = True

    target_rotation %= 360
    if target_rotation < 90:
        target_rotation += 360
        if current_rotation < 230:
            current_rotation += 360
    if target_rotation > 270 and current_rotation < 135:
        current_rotation += 360

    if auto_steering:
        turn_value = turn_to(
            current_rotation,
            target_rotation + 2,
        )

    if keyboard.is_pressed("'"):
        movement = go_to(2, 0, robot_position[0], robot_position[2])
        x_movement, y_movement = turn(movement[0], movement[1], 0)

    write_controls(
        turn_value,
        x_movement,
        y_movement,
        keyboard.is_pressed("o"), #outtake
        keyboard.is_pressed("i"), #intake
        keyboard.is_pressed("k"), #shoot
        keyboard.is_pressed("["), #stop
        keyboard.is_pressed("]"), #reset
        keyboard.is_pressed("f"), #lock elevator
        keyboard.is_pressed("t"), #elevator up
        keyboard.is_pressed("g"), #elevator down
        keyboard.is_pressed("q"), #rotate hood backward
        keyboard.is_pressed("e")  #rotate hood forward
    )

    if keyboard.is_pressed("esc"):
        break

    # this ensures 60 fps
    time.sleep(max(1.0 / 60 - (time.time() - start), 0))
