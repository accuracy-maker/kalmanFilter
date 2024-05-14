# Implement a kalman filter to process the data from sensors
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

class KalmanFilter():
    def __init__(self, state_dim=3, time_resolution=1):
        """
        Initialize the Kalman Filter.
        
        Parameters:
        state_dim (int): Dimension of the state vector.
        time_resolution (float): Time step for the state transition.
        """
        self.n = state_dim
        self.dt = time_resolution
        
        # Define the state transition matrix F
        self.F = np.eye(self.n)
        for i in range(1, self.n):
            self.F[i-1, i] = self.dt
        if self.n > 2:
            self.F[0, 2] = 0.5 * self.dt ** 2
        
        # Define the control matrix B (optional)
        self.B = np.zeros((self.n, 1))
        
        # Define the initial state estimate and covariance matrix
        self.x = np.zeros((self.n, 1))
        self.P = np.eye(self.n)
        
        # Process noise covariance matrix (Q) and measurement noise covariance matrix (R)
        self.Q = np.eye(self.n)
        self.R = np.eye(self.n)
        
        # Define measurement matrix H
        self.H = np.eye(self.n)
        
    def predict(self, u=np.zeros((1, 1))):
        """
        Predict the next state and covariance.
        
        Parameters:
        u (float or array): Control input (default is 0).
        
        Returns:
        numpy array: Predicted state estimate.
        """
        self.x = self.F @ self.x + self.B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x
        
    def update(self, z):
        """
        Update the state estimate with a new measurement.
        
        Parameters:
        z (array): Measurement vector.
        
        Returns:
        tuple: Updated state estimate and covariance matrix.
        """
        assert len(z) == self.n
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ inv(S)
        
        self.x = self.x + K @ y
        self.P = (np.eye(self.n) - K @ self.H) @ self.P
        
        return self.x, self.P

if __name__ == '__main__':
    kf = KalmanFilter()
    
    # Simulate some data
    true_state = np.array([0, 1, 0.1]).reshape(-1, 1)
    measurements = []
    predicted_states = []
    
    for t in range(10):
        # Simulate measurement
        measurement = true_state + np.random.normal(0, 1, (3, 1))
        measurements.append(measurement.flatten())
        
        # Predict step
        kf.predict()
        
        # Update step
        estimated_state, _ = kf.update(measurement)
        predicted_states.append(estimated_state.flatten())
        
        # Update true state (for simulation purposes)
        true_state = kf.F @ true_state

    measurements = np.array(measurements)
    predicted_states = np.array(predicted_states)
    
    # Plot results
    plt.figure()
    for i in range(3):
        plt.subplot(3, 1, i+1)
        plt.plot(measurements[:, i], label='Measurements')
        plt.plot(predicted_states[:, i], label='Kalman Filter Prediction')
        plt.legend()
    plt.show()

    
    