import mujoco
import numpy as np
from dataclasses import dataclass

@dataclass
class ContactPatch:
    """Raw physics data at 1 kHz"""
    timestamp_us: int
    normal_force_N: float
    shear_force_N: float
    slip_speed_mms: float
    contact_pos: np.ndarray
    contact_normal: np.ndarray
    in_contact: bool
    
    # Material properties
    mu_static: float
    mu_dynamic: float
    solref_damping: float

class MuJoCoEngine:
    def __init__(self, model_path: str):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Verify timestep
        assert abs(self.model.opt.timestep - 0.001) < 1e-6, \
            f"Model timestep must be 0.001 (1 kHz), got {self.model.opt.timestep}"
        
        self.dt = self.model.opt.timestep
        self.time_us = 0
        
        # Pre-allocate contact force array
        self.contact_force = np.zeros(6)
        
    def step(self) -> ContactPatch:
        """Run one physics step (1 ms) and extract contact data"""
        # Step simulation
        mujoco.mj_step(self.model, self.data)
        self.time_us += int(self.dt * 1e6)
        
        # Check for contact
        in_contact = self.data.ncon > 0
        
        if in_contact:
            # Get first contact (should only be one in this simple scene)
            contact = self.data.contact[0]
            
            # Calculate contact force using MuJoCo's contact Jacobian
            # This gives us the actual constraint forces
            mujoco.mj_contactForce(self.model, self.data, 0, self.contact_force)
            
            # Extract normal and tangential forces
            # contact_force contains [normal, tangent1, tangent2, torque1, torque2, torque3]
            normal_force = abs(self.contact_force[0])
            shear_force = np.linalg.norm(self.contact_force[1:3])
            
            # Get velocity at contact point for slip calculation
            # Use the body's linear velocity as proxy
            body_id = self.model.body("fingertip").id
            body_vel = self.data.body(body_id).cvel[:3]  # Linear velocity
            slip_speed_mms = np.linalg.norm(body_vel) * 1000  # m/s to mm/s
            
            # Get material properties from contact geoms
            geom1_id = contact.geom1
            geom2_id = contact.geom2
            
            # Use the finger geom (non-floor geom)
            finger_geom_id = geom1_id if geom1_id != 0 else geom2_id
            
            mu_static = self.model.geom_friction[finger_geom_id, 0]
            mu_dynamic = self.model.geom_friction[finger_geom_id, 1]
            solref_damping = self.model.geom_solref[finger_geom_id, 1]
            
            return ContactPatch(
                timestamp_us=self.time_us,
                normal_force_N=normal_force,
                shear_force_N=shear_force,
                slip_speed_mms=slip_speed_mms,
                contact_pos=contact.pos.copy(),
                contact_normal=contact.frame[:3].copy(),
                in_contact=True,
                mu_static=mu_static,
                mu_dynamic=mu_dynamic,
                solref_damping=solref_damping
            )
        else:
            # No contact
            return ContactPatch(
                timestamp_us=self.time_us,
                normal_force_N=0.0,
                shear_force_N=0.0,
                slip_speed_mms=0.0,
                contact_pos=np.zeros(3),
                contact_normal=np.array([0, 0, 1]),
                in_contact=False,
                mu_static=0.0,
                mu_dynamic=0.0,
                solref_damping=0.0
            )
    
    def reset(self):
        """Reset simulation to initial state"""
        mujoco.mj_resetData(self.model, self.data)
        self.time_us = 0