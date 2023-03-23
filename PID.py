# PID FOR THE AUTONOMOUS NAVIGATION

# Create object class for PID_Controller. Object contains the PID cofficients, a method for calculating
# output signal based on system state, a method for running a system simulation based on the controller signal
# and a method for autotuning the attributes to optimize controller performance for an input system. 
class PID_Controller:
    
    # Initialize controller attributes when object is created. kp, ki, and kd are the constants for the
    # proportional, integral, and derivative elements of the controller, respectively. Target is an input attribute
    # which establishes the goal the controller is trying to achieve. Target is not set at object intiliazation, but
    # is rather set as part of "running" the controller. Signal is effectively an output attribute which represents 
    # the "current" signal being emitted from the controller. Accumulator is a attribute used to execute the integral 
    # calculation, and last_reading is using to determine the slope for application of the derivative calculation.
    # Max_signal is used to limit the signal request that the controller can make, and the sample_rate establishes how
    # frequently the controller can adjust the signal. In a practical application this would equate to a 
    # maximum speed/torque/force of a motor or actuator element and the sampling rate of a digital controller device.
    kp = 0
    ki = 0
    kd = 0
    sample_rate = 1.0/100
    max_signal = 12 # da capire
    target = 0 # target de pid, valore agnostico
    

    def __init__(self, kp, ki, kd, max_signal, sample_rate, target):
        
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if target is not None:
            self.target = target #distanza in cm
        if sample_rate is not None:
            self.sample_rate = sample_rate
        if max_signal is not None:
            self.max_signal = max_signal #initial assumption

        self.signal = 0
        self.accumulator = 0
        self.last_reading = 0
        self.xa = 0.140 #in m 



    # Set_new_target sets the target attribute and resets the held accumulator value. Resetting the accumulator
    # is necessary for the integral controller to function properly.
    def set_new_target(self, target):
        self.accumulator = 0
        self.target = target

        # return target, accumulator;
    

    # Adjust_signal is the method that performs the actual function of the controller. This method calculates a 
    # new signal based on the feedback_value, accumulator value, last_reading, and target. It then sets the new
    # signal to the self.signal attribute.
    def adjust_signal(self, feedback_value):
        # kp = 100 #initial assumption
        # ki = 0
        # kd = 0
        # Calculate the error - difference between target and feedback_value (measurement)
        error = self.target - feedback_value

        # Add error to accumulator
        self.accumulator += error * self.sample_rate

        # Calculate signal based on PID coefficients
        self.signal = self.kp * error + self.ki * self.accumulator + self.kd * (feedback_value - self.last_reading)/self.sample_rate

        # max_signal = 10000000000
        # If calculated signal exceeds max_signal, then set signal to max_signal value. Do this in both positive
        # and negative directions
        if self.signal > self.max_signal:
            self.signal = self.max_signal
        elif self.signal < -self.max_signal:
            self.signal = -self.max_signal

        # Save current feedback_value as last_reading attribute for use in calulating next signal.
        self.last_reading = feedback_value

        return self.signal

        # Run_simulation is a controller method which will simulate the output of a system object "system_simulator"
        # over a time_period "duration."

    def get_force_z(self, signal):
        force_z = signal
        return force_z

    def get_force_lateral(self, signal_distance, signal_psi):
        force_L = (signal_distance*self.xa + signal_psi)/(2 * self.xa)
        force_R = (signal_distance*self.xa - signal_psi)/(2 * self.xa)
        return force_L, force_R

    def pwm_z_motor(self, force_z):
        m_coefficient = 11.99/100 #  da specificare per ogni motore
        pwm_z = (force_z / m_coefficient) 
        return pwm_z

    def pwm_L_motor(self, force_L):
        m_coefficient = 11.99/100 # da specificare per ogni motore
        pwm_L = (force_L * m_coefficient) 
        return pwm_L

    def pwm_R_motor(self, force_R):
        m_coefficient = 11.99/100 # da specificare per ogni motore
        pwm_R = (force_R * m_coefficient) 
        return pwm_R