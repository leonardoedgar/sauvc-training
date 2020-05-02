from sauvc_control.pid_controller import PIDController
import numpy as np


def setup_questions():
    # type: () -> (np.ndarray, np.ndarray, np.ndarray)
    """A function to setup the given question.
    Returns:
        time: the timestamp
        desired_value: the desired system output
        actual_value: the actual system output
    """
    time = np.arange(0, 20, 0.001)
    desired_value = np.array([])
    for atime in time:
        if atime >= 2:
            desired_value = np.append(desired_value, 5)
        else:
            desired_value = np.append(desired_value, 0)
    actual_value = np.zeros(time.size)
    return time, desired_value, actual_value


def get_user_input(instruction_msg):
    # type: (str) -> int
    """A function to get the user input.
    Args:
        instruction_msg: instruction message for the user
    Returns:
        user_resp: the user response
    """
    while True:
        try:
            user_resp = int(input(instruction_msg))
        except ValueError:
            continue
        else:
            return user_resp


if __name__ == '__main__':
    time, desired_speed, actual_speed = setup_questions()
    # p = gain = 1
    # i = gain / time_constant = 1.582
    # d = gain * dead_time = 0.1
    p = get_user_input("Enter P: ")
    i = get_user_input("Enter I: ")
    d = get_user_input("Enter D: ")
    stabilised_speed = PIDController(p=p, i=i, d=d).get_stabilised_values(
        time=time, actual_values=actual_speed, desired_values=desired_speed)
    PIDController.plot_result(time=time,
                              actual_values=actual_speed,
                              desired_values=desired_speed,
                              stabilised_values=stabilised_speed)
