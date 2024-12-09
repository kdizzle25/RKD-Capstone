import sys
sys.path.append('../config')
import numpy as np
from frankapy import FrankaArm
from robot_config import RobotConfig
from task_config import TaskConfig

class Robot:
    def __init__(self):
        """Initializing the motion planner with the robot controller"""
        self.dof = 7
    
    def forward_kinematcis(self, dh_parameters, thetas):
        """
        Computing the foward kinematics
        
        Your implementation should:
        1. Compute transformation matrices for each frame using DH parameters
        2. Compute end-effector pose
        
        Parameters
        ----------
        dh_parameters: np.ndarray
            DH parameters (you can choose to apply the offset to the tool flange, center of gripper, or the pen tip)
        thetas : np.ndarray
            All joint angles
            
        Returns
        -------
        np.ndarray
            End-effector pose
        """
        if thetas.ndim != 1:
            raise ValueError('Expecting a 1D array of joint angles.')

        if thetas.shape[0] != self.dof:
            raise ValueError(f'Invalid number of joints: {thetas.shape[0]} found, expecting {self.dof}')
        
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        # TODO
        # frames = np.zeros((4, 4, len(dh_parameters)+1))
        """Computing the forward kinematics using DH parameters"""
        # Initializing the transformation matrix
        T = np.eye(4)
        frames = np.zeros((4, 4, len(dh_parameters)+1))
        frames[:,:,0] = T

        # For each joint
        for i in range(len(dh_parameters)):
            # Extracting the DH parameters
            a = dh_parameters[i,0]      # Link length
            alpha = dh_parameters[i,1]  # Link twist
            d = dh_parameters[i,2]      # Link offset
            theta = thetas[i]           # Joint angle

            # Computing transformation matrix for this joint
            ct = np.cos(theta)
            st = np.sin(theta)
            ca = np.cos(alpha)
            sa = np.sin(alpha)
        
            # DH transformation matrix
            Ti = np.array([
                [ct, -st*ca, st*sa, a*ct],
                [st, ct*ca, -ct*sa, a*st],
                [0, sa, ca, d],
                [0, 0, 0, 1]
            ])

            # Updating the total transformation
            T = T @ Ti
            frames[:,:,i+1] = T

        return T
        
        raise NotImplementedError("Implement forward kinematics")
        # --------------- END STUDENT SECTION --------------------------------------------------
    
    def jacobians(self, thetas):
        """
        Computing the Jacobians for each frame.

        Parameters
        ----------
        thetas : np.ndarray
            All joint angles
            
        Returns
        -------
        np.ndarray
            Jacobians
        """
        if thetas.shape != (self.dof,):
            raise ValueError(f'Invalid thetas: Expected shape ({self.dof},), got {thetas.shape}.')

        jacobians = np.zeros((6, self.dof, self.dof + 1))
        epsilon = 0.001

        # --------------- BEGIN STUDENT SECTION ----------------------------------------
        # TODO: Implement the numerical computation of the Jacobians

        # Hints:
        # - Perturb each joint angle by epsilon and compute the forward kinematics.
        # - Compute numerical derivatives for x, y, and positions.
        # - Determine the rotational component based on joint contributions.
        # Getting the base transformation
        base_T = self.forward_kinematics(thetas)
        base_pos = base_T[:3, 3]
    
        # For each joint
        for i in range(self.dof):
            # Perturbation
            perturbed_thetas = thetas.copy()
            perturbed_thetas[i] += epsilon
        
            # Getting the perturbed transformation
            perturbed_T = self.forward_kinematics(perturbed_thetas)
            perturbed_pos = perturbed_T[:3, 3]
        
            # Linear velocity component (position derivative)
            linear_vel = (perturbed_pos - base_pos) / epsilon
        
            # Angular velocity component
            # For revolute joints, axis of rotation is the z-axis of the previous frame
            rotation_axis = base_T[:3, 2]  # z-axis of current frame
        
            # Combining into the Jacobian
            jacobians[:3, i, :] = linear_vel
            jacobians[3:, i, :] = rotation_axis
    
        return jacobians

        raise NotImplementedError("Implement jacobians")
        # --------------- END STUDENT SECTION --------------------------------------------
    
    def _inverse_kinematics(self, target_pose, seed_joints):
        """
        Computing the inverse kinematics using the Jacobian pseudo-inverse method.
        
        Your implementation should:
        1. Start from seed joints
        2. Iterate until convergence or max iterations
        3. Check joint limits and singularities
        4. Return None if no valid solution
        
        Parameters
        ----------
        target_pose : 4x4 np.ndarray
            Desired end-effector pose
        seed_joints : np.ndarray
            Initial joint configuration
            
        Returns
        -------
        np.ndarray or None
            Joint angles that achieve target pose, or None if not found
            
        Hints
        -----
        - Use get_pose() from robot arm
        - Implement a helper function to track pose error magnitude for convergence
        - The iteration parameters are defined in RobotConfig and TaskConfig, feel free to update them
        """
        
        if seed_joints.shape != (self.dof):
            raise ValueError(f'Invalid initial_thetas: Expected shape ({self.dof},), got {seed_joints.shape}.')
        
        if seed_joints is None:
            seed_joints = self.robot.arm.get_joints()
        
        # --------------- BEGIN STUDENT SECTION ------------------------------------------------
        # TODO: Implement gradient inverse kinematics
        # Initializing the parameters
        max_iterations = 100
        convergence_threshold = 1e-3
        damping = 0.1
        current_joints = seed_joints.copy()
        
        for iteration in range(max_iterations):
            # Getting the current pose
            current_pose = self.forward_kinematics(current_joints)
            
            # Calculating the pose error
            position_error = target_pose[:3, 3] - current_pose[:3, 3]
            rotation_error = self._rotation_error(target_pose[:3, :3], current_pose[:3, :3])
            pose_error = np.concatenate([position_error, rotation_error])
            
            # Checking convergence
            if np.linalg.norm(pose_error) < convergence_threshold:
                return current_joints
                
            # Calculating the Jacobian
            J = self.jacobians(current_joints)[:, :, -1]  # Getting the end-effector Jacobian
            
            # Calculating the damped pseudo-inverse
            J_pinv = J.T @ np.linalg.inv(J @ J.T + damping * np.eye(6))
            
            # Updating joint angles
            delta_theta = J_pinv @ pose_error
            current_joints = current_joints + delta_theta
            
            # Checking joint limits
            if not self._check_joint_limits(current_joints):
                return None
                
        return None
    
    def _rotation_error(self, R_desired, R_current):
        """Calculating rotation error between two rotation matrices"""
        R_error = R_desired @ R_current.T
        angle_axis = np.array([R_error[2,1] - R_error[1,2],
                              R_error[0,2] - R_error[2,0],
                              R_error[1,0] - R_error[0,1]]) / 2
        return angle_axis
    
    def _check_joint_limits(self, joints):
        """Checking if joints are within limits"""
        joint_limits = np.array([[-np.pi, np.pi]] * self.dof) #guessed limits
        return np.all(joints >= joint_limits[:, 0]) and np.all(joints <= joint_limits[:, 1])
        raise NotImplementedError("Implement inverse kinematics")
            # --------------- END STUDENT SECTION --------------------------------------------------
