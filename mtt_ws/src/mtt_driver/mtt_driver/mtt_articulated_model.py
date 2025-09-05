#!/usr/bin/env python3
"""
MTT Articulated Vehicle Dynamics Model
Implements realistic dynamics for tracked articulated vehicle with central steering joint.
"""

import math
import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class ArticulatedVehicleParams:
    """Physical parameters of the articulated vehicle."""
    
    # Vehicle geometry (Real MTT-154 specifications from URDF)
    front_wheelbase: float = 1.051      # Distance from front axle to articulation joint (m)
    rear_wheelbase: float = 1.27        # Distance from articulation joint to rear axle (m)
    track_width: float = 0.605          # Track width (m)
    
    # Track dynamics
    track_slip_coeff: float = 0.12      # Lateral slip coefficient
    track_grip_coeff: float = 0.85      # Forward grip coefficient
    carving_factor: float = 0.25        # Track digging effect in turns
    
    # Articulation limits from URDF joint limits
    max_articulation_angle: float = math.radians(60)  # Maximum joint angle (rad)
    articulation_response: float = 0.7  # Joint response rate
    
    # Speed-dependent factors
    slip_speed_factor: float = 0.08     # Slip increases with speed  
    min_speed_for_steering: float = 0.05  # Minimum speed for effective steering (m/s)


class ArticulatedVehicleDynamics:
    """
    Articulated vehicle dynamics model for MTT.
    
    This model simulates:
    1. Front tractor propulsion
    2. Articulated joint steering
    3. Track slip and carving
    4. Trailer following dynamics
    """
    
    def __init__(self, params: Optional[ArticulatedVehicleParams] = None):
        self.params = params or ArticulatedVehicleParams()
        
        # Vehicle state
        self.x = 0.0              # Position X (m)
        self.y = 0.0              # Position Y (m)
        self.heading = 0.0        # Vehicle heading (rad)
        self.articulation_angle = 0.0  # Joint angle (rad)
        
        # Velocities
        self.linear_velocity = 0.0     # Forward velocity (m/s)
        self.angular_velocity = 0.0    # Heading change rate (rad/s)
        
        # Track forces and slip
        self.front_slip_angle = 0.0    # Front track slip angle
        self.rear_slip_angle = 0.0     # Rear track slip angle
        
    def update(self, 
               throttle_input: float,        # Forward/reverse command (-1 to 1)
               steering_input: float,        # Steering command (-1 to 1)
               dt: float,                    # Time step (s)
               terrain_grip: float = 1.0     # Terrain grip factor (0-1)
               ) -> Tuple[float, float, float]:
        """
        Update vehicle dynamics for one time step.
        
        Returns:
            (x, y, heading) - New vehicle position and orientation
        """
        
        # 1. Calculate target articulation angle from steering input
        target_articulation = steering_input * self.params.max_articulation_angle
        
        # 2. Update articulation angle with response dynamics
        articulation_error = target_articulation - self.articulation_angle
        self.articulation_angle += articulation_error * self.params.articulation_response * dt
        
        # Limit articulation angle
        self.articulation_angle = max(-self.params.max_articulation_angle,
                                     min(self.params.max_articulation_angle,
                                         self.articulation_angle))
        
        # 3. Calculate forward velocity from throttle (simplified)
        max_velocity = 5.0  # m/s - adjust based on your vehicle
        target_velocity = throttle_input * max_velocity
        
        # Apply terrain grip to acceleration
        accel_factor = terrain_grip * self.params.track_grip_coeff
        velocity_error = target_velocity - self.linear_velocity
        self.linear_velocity += velocity_error * accel_factor * dt
        
        # 4. Calculate vehicle kinematics using articulated bicycle model
        if abs(self.linear_velocity) > self.params.min_speed_for_steering:
            # Effective steering based on articulation and speed
            effective_steering_angle = self.articulation_angle
            
            # Speed-dependent slip adjustment
            speed_factor = min(1.0, abs(self.linear_velocity) / 2.0)  # Normalize to 2 m/s
            slip_factor = self.params.track_slip_coeff * (1.0 + speed_factor * self.params.slip_speed_factor)
            
            # Calculate angular velocity using articulated vehicle model
            # This is derived from the kinematic model of an articulated vehicle
            wheelbase_effective = self.params.front_wheelbase + self.params.rear_wheelbase
            
            # The articulated joint creates a virtual steering angle
            virtual_steering_angle = math.atan(
                wheelbase_effective * math.sin(effective_steering_angle) /
                (self.params.front_wheelbase + self.params.rear_wheelbase * math.cos(effective_steering_angle))
            )
            
            # Apply track carving effect (tracks dig into turns)
            carving_effect = self.params.carving_factor * abs(virtual_steering_angle)
            enhanced_steering = virtual_steering_angle * (1.0 + carving_effect)
            
            # Calculate angular velocity
            self.angular_velocity = (self.linear_velocity * math.tan(enhanced_steering) / 
                                   wheelbase_effective) * (1.0 - slip_factor * abs(enhanced_steering))
            
            # Calculate slip angles for front and rear tracks
            self.front_slip_angle = effective_steering_angle - self.angular_velocity * self.params.front_wheelbase / self.linear_velocity
            self.rear_slip_angle = -self.angular_velocity * self.params.rear_wheelbase / self.linear_velocity
            
        else:
            # At very low speeds, no effective steering
            self.angular_velocity = 0.0
            self.front_slip_angle = 0.0
            self.rear_slip_angle = 0.0
        
        # 5. Update position and heading
        self.heading += self.angular_velocity * dt
        
        # Normalize heading
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))
        
        # Update position
        self.x += self.linear_velocity * math.cos(self.heading) * dt
        self.y += self.linear_velocity * math.sin(self.heading) * dt
        
        return self.x, self.y, self.heading
    
    def get_state(self) -> dict:
        """Get current vehicle state."""
        return {
            'x': self.x,
            'y': self.y,
            'heading': self.heading,
            'linear_velocity': self.linear_velocity,
            'angular_velocity': self.angular_velocity,
            'articulation_angle': self.articulation_angle,
            'front_slip_angle': self.front_slip_angle,
            'rear_slip_angle': self.rear_slip_angle
        }
    
    def set_state(self, x: float, y: float, heading: float):
        """Set vehicle position and heading."""
        self.x = x
        self.y = y
        self.heading = heading
    
    def reset(self):
        """Reset vehicle to origin."""
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.articulation_angle = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.front_slip_angle = 0.0
        self.rear_slip_angle = 0.0


def calculate_instantaneous_center_of_rotation(vehicle_state: dict, params: ArticulatedVehicleParams) -> Tuple[float, float]:
    """Calculate instantaneous center of rotation for articulated vehicle."""
    if abs(vehicle_state['angular_velocity']) < 1e-6:
        return float('inf'), float('inf')
    
    radius = vehicle_state['linear_velocity'] / vehicle_state['angular_velocity']
    
    icr_x = -radius * math.sin(vehicle_state['heading'])
    icr_y = radius * math.cos(vehicle_state['heading'])
    
    global_icr_x = vehicle_state['x'] + icr_x
    global_icr_y = vehicle_state['y'] + icr_y
    
    return global_icr_x, global_icr_y
