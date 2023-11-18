from tkgpio import TkCircuit

configuration = {
    "width": 210,
    "height": 350,
    "servos": [
        {"x": 25, "y": 50, "name": "chassis_shoulder_FR_joint", "pin": 1, "min_angle": -60, "max_angle": 60, "initial_angle": 20},
        {"x": 25, "y": 150, "name": "shoulder_upper_leg_FR_joint", "pin": 2, "min_angle": -60, "max_angle": 60, "initial_angle": 20},
        {"x": 25, "y": 250, "name": "upper_leg_lower_leg_FR_joint", "pin": 3, "min_angle": -60, "max_angle": 60, "initial_angle": 20}
    ]
}
def run (main_function):
    circuit = TkCircuit(configuration)
    circuit.run(main_function)