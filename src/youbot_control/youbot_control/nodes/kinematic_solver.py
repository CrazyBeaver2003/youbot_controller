import numpy as np
from scipy.optimize import minimize, BFGS
import warnings

class KukaYouBotKinematic:
    def __init__(self):
        # DH parameters: [a, alpha, d, theta] for each joint
        self.dh_params = [
            [0.033, np.pi/2, 0.253, 0], # Joint 1
            [0.155, 0, 0, 0],      # Joint 2
            [0.135, 0, 0, 0],      # Joint 3
            [0, np.pi/2, 0, np.pi/2],    # Joint 4
            [0, 0, 0.205, 0]       # Joint 5
        ]
        # Angle limits for each joint
        self.joint_limits = [
            (-2.96, 2.96),  # Joint 1
            (-1.57, 1.13),  # Joint 2
            (-2.09, 2.09),  # Joint 3
            (-1.92, 1.92),  # Joint 4
            (-2.96, 2.96)   # Joint 5
        ]
        
    def dh_transform(self,a, alpha, d, theta):
        """
        Denavit-Hartenberg transform for a single joint.

        This function returns a 4x4 transformation matrix representing the
        forward kinematics of a single joint, given its DH parameters.

        Parameters:
        a (float): The length of the joint's link.
        alpha (float): The angle of the joint's axis.
        d (float): The offset of the joint's axis.
        theta (float): The angle of the joint.

        Returns:
        np.ndarray: A 4x4 transformation matrix representing the forward kinematics of the joint.
        """
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        return np.array([
            [cos_theta,             -sin_theta,             0,          alpha       ],
            [sin_theta*cos_alpha,   cos_theta*cos_alpha,  -sin_alpha,   -d*sin_alpha],
            [sin_theta*sin_alpha,   cos_theta*sin_alpha,    cos_alpha,   cos_alpha*d],
            [0,                     0,                      0,                     1]
        ])
    
    def forward_kinematics(self, joint_angles):
        T = np.eye(4)
        for i, (a, alpha, d, theta_offset) in enumerate(self.dh_params):
            theta = joint_angles[i] + theta_offset
            T_i = self.dh_transform(a, alpha, d, theta)
            T = T @ T_i
        return T
    def pose_error(self, current_pose, target_pose):
        position_error = np.linalg.norm(current_pose[:3, 3] - target_pose[:3, 3])

        #Orientation error using rotation matrices
        R_current = current_pose[:3, :3]
        R_target = target_pose[:3, :3]
        R_diff = R_current.T @ R_target

        trace = np.trace(R_diff)
        angle_error = np.arccos(np.clip((trace - 1) / 2.0, -1.0, 1.0))
        angle_error = np.abs(angle_error)

        return 0.9*position_error + 0.1*angle_error
    
    def _objective_function(self, joint_angles, target_pose):
        current_pose = self.forward_kinematics(joint_angles)
        return self.pose_error(current_pose, target_pose)
    
    def inverse_kinematics(self, target_pose, initial_guess=None, max_iterations=200):
        if initial_guess is None:
            initial_guess = np.zeros(5)
        
        def objective(joint_angles):
            return self._objective_function(joint_angles, target_pose)
        
        result = minimize(
            objective, # Функция для минимизации
            initial_guess, # Начальная точка
            method ='L-BFGS-B', # Метод оптимизации
            bounds = self.joint_limits, # Ограничения углов
            options={
                'maxiter': max_iterations, # Макс. итераций (200)
                'ftol': 1e-6,  # Точность функции
                'eps': 1e-8 # Шаг для численного градиента
            }
        )
        final_pose = self.forward_kinematics(result.x)
        final_error = self.pose_error(final_pose, target_pose)
        success = final_error < 1e-4
        return result.x, success, final_error