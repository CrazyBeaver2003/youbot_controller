import numpy as np
from scipy.optimize import minimize, BFGS
import warnings

class KukaYouBotKinematic:
    def __init__(self):
        # DH parameters: [a, alpha, d, theta] for each joint
        self.dh_params = [
            [0.033, np.pi/2, 0.253, 0], # Joint 1
            [0.155, 0, 0, np.pi/2],      # Joint 2
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
        
        # ✅ НОВОЕ: Фиксация последнего сустава
        self.fix_last_joint =  True
        self.fixed_joint5_angle = 0.0  # Угол для фиксации (0 = горизонтально)
        
    def dh_transform(self, a, alpha, d, theta):
        """
        Denavit-Hartenberg transform for a single joint.
        """
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        return np.array([
            [cos_theta,             -sin_theta*cos_alpha,   sin_theta*sin_alpha,          a*cos_theta],
            [sin_theta,             cos_theta*cos_alpha,    -cos_theta*sin_alpha,         a*sin_theta],
            [0,                     sin_alpha,              cos_alpha,                              d],
            [0,                     0,                      0,                                      1]
        ])
    
    def forward_kinematics(self, joint_angles):
        """
        Forward kinematics с учетом фиксации последнего сустава
        """
        T = np.eye(4)
        for i, (a, alpha, d, theta_offset) in enumerate(self.dh_params):
            # ✅ Если последний сустав зафиксирован
            if self.fix_last_joint and i == 4:
                theta = self.fixed_joint5_angle + theta_offset
            else:
                theta = joint_angles[i] + theta_offset
            
            T_i = self.dh_transform(a, alpha, d, theta)
            T = T @ T_i
        return T
    
    def pose_error(self, current_pose, target_pose):
        position_error = np.linalg.norm(current_pose[:3, 3] - target_pose[:3, 3])

        # Orientation error using rotation matrices
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
    
    def inverse_kinematics(self, target_pose, initial_guess=None, max_iterations=500):
        if initial_guess is None:
            initial_guess = np.zeros(5)
        
        # ✅ Если последний сустав зафиксирован, оптимизируем только первые 4
        if self.fix_last_joint:
            # Оптимизация только для первых 4 суставов
            initial_guess_reduced = initial_guess[:4]
            bounds_reduced = self.joint_limits[:4]
            
            def objective(joint_angles_4):
                # Добавляем фиксированный угол 5-го сустава
                joint_angles_5 = np.append(joint_angles_4, self.fixed_joint5_angle)
                return self._objective_function(joint_angles_5, target_pose)
            
            result = minimize(
                objective,
                initial_guess_reduced,
                method='L-BFGS-B',
                bounds=bounds_reduced,
                options={
                    'maxiter': max_iterations,
                    'ftol': 1e-6,
                    'eps': 1e-8
                }
            )
            
            # Добавляем фиксированный угол в результат
            final_angles = np.append(result.x, self.fixed_joint5_angle)
            
        else:
            # Стандартная оптимизация всех 5 суставов
            def objective(joint_angles):
                return self._objective_function(joint_angles, target_pose)
            
            result = minimize(
                objective,
                initial_guess,
                method='L-BFGS-B',
                bounds=self.joint_limits,
                options={
                    'maxiter': max_iterations,
                    'ftol': 1e-6,
                    'eps': 1e-8
                }
            )
            final_angles = result.x
        
        final_pose = self.forward_kinematics(final_angles)
        final_error = self.pose_error(final_pose, target_pose)
        success = final_error < 1e-4
        
        return final_angles, success, final_error