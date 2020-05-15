import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint


class PIDController(object):
    """ A class to represent a PID Controller.
    """
    def __init__(self, p, i, d):
        # type: (float, float, float)
        """Initialise the class.
        Args:
            p: the proportional controller constant
            i: the integral controller constant
            d: the derivative controller constant
        """
        self._P = p
        self._I = i
        self._D = d

    def get_stabilised_values(self, time, actual_values, desired_values):
        # type: (np.ndarray, np.ndarray, np.ndarray) -> np.ndarray
        """A function to get the stabilised output values.
        Args:
            time: the timestamp
            actual_values: the actual system output
            desired_values: the desired system output
        Returns:
            stabilised_values: the stabilised system output
        """
        stabilised_values = np.copy(actual_values)
        controller_output = np.zeros(time.size)
        I_error = 0
        initial_stabilised_value = 0
        for index in range(1, len(actual_values)):
            stabilised_values[index] = odeint(PIDController.get_system_output_change_rate, initial_stabilised_value,
                                              [time[index-1], time[index]],
                                              args=(controller_output[index-1],))[1]
            initial_stabilised_value = stabilised_values[index]
            error = desired_values[index] - stabilised_values[index]
            delta_time = time[index] - time[index-1]
            P_error = self._P * error
            I_error += self._I * error * delta_time
            D_error = self._D * -(stabilised_values[index] - stabilised_values[index-1]) / delta_time
            controller_output[index] = P_error + I_error + D_error
        return stabilised_values

    @staticmethod
    def get_system_output_change_rate(actual_output, time, system_input):
        """A function to get the system output change rate.
        Args:
            actual_output: the actual output of the system
            time: the timestamp
            system_input: the system input
        Returns:
            output_change_rate: the output system change rate
        """
        # type: (float, float, float) -> float
        gain = 1
        time_constant = 0.632
        dead_time = 0.001
        if time < dead_time:
            output_change_rate = 0.0
        else:
            output_change_rate = (1.0 / time_constant) * (-actual_output + gain * system_input)
        return output_change_rate

    @staticmethod
    def plot_result(time, desired_values, actual_values, stabilised_values):
        """A function to plot the stabilised result.
        Args:
            time: the plot timestamp
            desired_values: the desired values of the system output
            actual_values: the actual system output
            stabilised_values: the stabilised values of the actual system output
        """
        # type: (np.ndarray, np.ndarray, np.ndarray, np.ndarray) -> None
        plt.figure("PID Control")
        plt.plot(time, desired_values, color="red", linewidth=1.5, label='Desired thruster speed')
        plt.plot(time, actual_values, color="blue", linewidth=2.5, linestyle="dashed", label="Actual thruster speed")
        plt.plot(time, stabilised_values, color="green", linewidth=2, linestyle=":", label="Stabilised thruster speed")
        plt.title('Thruster Speed Transient Response')
        plt.xlabel('Velocity (m/s)')
        plt.ylabel('Time (s)')
        plt.grid(alpha=0.4, linestyle='--')
        plt.legend()
        plt.xlim(0, 20)
        plt.ylim(0, 6)
        plt.show()
