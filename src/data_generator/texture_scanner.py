import mujoco
import numpy as np
import time
from dataclasses import dataclass
from typing import List, Tuple, Optional

@dataclass
class TextureSample:
    """Data from a single texture scan step"""
    timestamp_us: int
    stylus_z_acc: float
    shear_force: float
    velocity_mms: float
    surface_height: float

class TextureScanner:
    """
    The Virtual Phonograph.
    Generates procedural surfaces and drags a stylus across them.
    """
    
    def __init__(self, model_path: str = "assets/texture_scan.xml"):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Get hfield ID
        self.hfield_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_HFIELD, "surface_map")
        if self.hfield_id == -1:
            raise ValueError("Hfield 'surface_map' not found in model")
            
        # Hfield dimensions
        self.nrow = self.model.hfield_nrow[self.hfield_id]
        self.ncol = self.model.hfield_ncol[self.hfield_id]
        
        # Stylus body ID
        self.stylus_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "stylus")
        
    def generate_surface(self, roughness_scale: float = 1.0, pattern: str = "perlin"):
        """
        Fill the hfield with procedural noise.
        """
        # Get address of hfield data
        hfield_adr = self.model.hfield_adr[self.hfield_id]
        
        # Generate height map (normalized 0-1)
        if pattern == "white_noise":
            # Gaussian grit
            height_map = np.random.normal(0.5, 0.1 * roughness_scale, (self.nrow, self.ncol))
        elif pattern == "perlin":
            # Simple approximation of Perlin-like noise (smoothed random)
            # We'll just upscale a smaller random grid
            small_h, small_w = self.nrow // 8, self.ncol // 8
            small_grid = np.random.rand(small_h, small_w)
            
            # Bilinear upsampling (crude but fast)
            import scipy.ndimage
            height_map = scipy.ndimage.zoom(small_grid, 8, order=1)
            
            # Crop to exact size
            height_map = height_map[:self.nrow, :self.ncol]
            
            # Scale
            height_map = (height_map - 0.5) * roughness_scale + 0.5
            
        else:
            # Flat
            height_map = np.ones((self.nrow, self.ncol)) * 0.5
            
        # Clip to valid range
        height_map = np.clip(height_map, 0.0, 1.0)
        
        # Update MuJoCo model data
        # Flatten and assign
        self.model.hfield_data[hfield_adr : hfield_adr + self.nrow * self.ncol] = height_map.flatten()
        
        # Notify MuJoCo that assets changed (if using viewer, but we are headless)
        # mujoco.mjr_uploadHField(self.model, self.context, self.hfield_id) 

    def run_scan(self, velocity_cms: float = 10.0, duration_s: float = 1.0) -> List[TextureSample]:
        """
        Drag the stylus across the surface at constant velocity.
        """
        mujoco.mj_resetData(self.model, self.data)
        
        # Initial position: Start at x=-0.4, y=0, z=0.06 (above surface)
        # Hfield is size 1x1, centered at 0
        start_x = -0.4
        self.data.qpos[0] = start_x
        self.data.qpos[1] = 0.0
        self.data.qpos[2] = 0.05 # Drop height
        
        # Run simulation
        dt = self.model.opt.timestep
        n_steps = int(duration_s / dt)
        velocity_ms = velocity_cms / 100.0
        
        history = []
        
        for i in range(n_steps):
            # 1. Force constant velocity on X axis (Servo control)
            # We overwrite the velocity state directly for the "infinite torque" motor effect
            # Or we can use a slide joint actuator. 
            # Direct state manipulation is cleaner for a "perfect" scanner.
            
            # Current X position
            current_x = self.data.qpos[0]
            
            # Desired X position (move forward)
            # Actually, let's just set velocity and let physics integrate, 
            # but we need to cancel drag.
            # Simplest: Set qvel[0] = target_v every step.
            self.data.qvel[0] = velocity_ms
            self.data.qvel[1] = 0.0 # Constrain Y
            # Z is free to bounce
            
            # 2. Step Physics
            mujoco.mj_step(self.model, self.data)
            
            # 3. Record Data
            # Z-acceleration (vibration)
            # qacc is calculated during forward dynamics, but mj_step integrates it.
            # To get 'sensor' acceleration, we should use an accelerometer sensor or 
            # just use qacc[2] (vertical acceleration of the free joint)
            z_acc = self.data.qacc[2]
            
            # Shear Force
            # We need to find the contact force.
            shear = 0.0
            if self.data.ncon > 0:
                # Sum shear forces from all contacts on the stylus
                for j in range(self.data.ncon):
                    contact = self.data.contact[j]
                    # Check if stylus is involved
                    # (We assume it is, since it's the only moving thing)
                    
                    # Get force in contact frame
                    c_force = np.zeros(6)
                    mujoco.mj_contactForce(self.model, self.data, j, c_force)
                    
                    # Shear is magnitude of tangential components (indices 1, 2)
                    shear += np.linalg.norm(c_force[1:3])
            
            # Sample
            sample = TextureSample(
                timestamp_us=int(self.data.time * 1e6),
                stylus_z_acc=z_acc,
                shear_force=shear,
                velocity_mms=velocity_cms * 10.0,
                surface_height=0.0 # Placeholder, hard to query exact height efficiently without raycast
            )
            history.append(sample)
            
        return history

if __name__ == "__main__":
    # Quick test
    scanner = TextureScanner()
    scanner.generate_surface(roughness_scale=1.0, pattern="white_noise")
    data = scanner.run_scan(velocity_cms=10.0, duration_s=0.5)
    print(f"Generated {len(data)} samples.")
    print(f"Avg Shear: {np.mean([s.shear_force for s in data]):.4f} N")
    print(f"Max Z-Acc: {np.max([abs(s.stylus_z_acc) for s in data]):.4f} m/s^2")
