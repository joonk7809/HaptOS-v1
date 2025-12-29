"""
Generates different simulation scenarios for diverse training data.
"""

import numpy as np
from typing import Dict, List
from dataclasses import dataclass

@dataclass
class ScenarioConfig:
    """Configuration for a simulation scenario"""
    name: str
    initial_height: float      # meters
    initial_velocity: float    # m/s (negative = downward)
    sphere_mass: float         # kg
    sphere_radius: float       # meters
    floor_friction: float      # coefficient
    floor_stiffness: float     # solref damping
    duration_s: float          # simulation duration
    
class ScenarioGenerator:
    """
    Generates diverse simulation scenarios to create varied training data.
    """
    
    def __init__(self):
        self.scenarios = []
        
    def generate_drop_scenarios(self, n_scenarios: int = 10) -> List[ScenarioConfig]:
        """
        Generate N different sphere drop scenarios with varying parameters.
        
        Variations:
        - Drop height (0.2m to 2.0m)
        - Sphere mass (0.05kg to 0.5kg)
        - Sphere size (0.02m to 0.1m)
        - Surface friction (0.3 to 0.9)
        - Surface stiffness (soft to hard)
        """
        scenarios = []
        
        for i in range(n_scenarios):
            # Randomize parameters
            height = np.random.uniform(0.3, 1.5)
            mass = np.random.uniform(0.05, 0.3)
            radius = np.random.uniform(0.03, 0.08)
            friction = np.random.uniform(0.4, 0.9)
            stiffness = np.random.uniform(0.5, 2.0)
            
            scenario = ScenarioConfig(
                name=f"drop_{i:03d}",
                initial_height=height,
                initial_velocity=0.0,
                sphere_mass=mass,
                sphere_radius=radius,
                floor_friction=friction,
                floor_stiffness=stiffness,
                duration_s=2.0
            )
            
            scenarios.append(scenario)
        
        self.scenarios.extend(scenarios)
        return scenarios
    
    def generate_tap_scenarios(self, n_scenarios: int = 10) -> List[ScenarioConfig]:
        """
        Generate quick tap scenarios (small drops, light impacts).
        """
        scenarios = []
        
        for i in range(n_scenarios):
            height = np.random.uniform(0.1, 0.4)
            mass = np.random.uniform(0.03, 0.1)
            radius = np.random.uniform(0.02, 0.05)
            friction = np.random.uniform(0.5, 0.8)
            stiffness = np.random.uniform(0.8, 1.5)
            
            scenario = ScenarioConfig(
                name=f"tap_{i:03d}",
                initial_height=height,
                initial_velocity=0.0,
                sphere_mass=mass,
                sphere_radius=radius,
                floor_friction=friction,
                floor_stiffness=stiffness,
                duration_s=1.0
            )
            
            scenarios.append(scenario)
        
        self.scenarios.extend(scenarios)
        return scenarios
    
    def generate_throw_scenarios(self, n_scenarios: int = 10) -> List[ScenarioConfig]:
        """
        Generate throw scenarios (initial downward velocity).
        """
        scenarios = []
        
        for i in range(n_scenarios):
            height = np.random.uniform(0.5, 1.0)
            velocity = -np.random.uniform(1.0, 3.0)  # Downward
            mass = np.random.uniform(0.1, 0.4)
            radius = np.random.uniform(0.04, 0.08)
            friction = np.random.uniform(0.4, 0.9)
            stiffness = np.random.uniform(0.5, 2.0)
            
            scenario = ScenarioConfig(
                name=f"throw_{i:03d}",
                initial_height=height,
                initial_velocity=velocity,
                sphere_mass=mass,
                sphere_radius=radius,
                floor_friction=friction,
                floor_stiffness=stiffness,
                duration_s=1.5
            )
            
            scenarios.append(scenario)
        
        self.scenarios.extend(scenarios)
        return scenarios
    
    def create_xml_for_scenario(self, scenario: ScenarioConfig, output_path: str):
        """
        Create a MuJoCo XML file for a specific scenario.
        """
        xml_content = f"""<mujoco model="{scenario.name}">
  <option timestep="0.001" gravity="0 0 -9.81"/>
  
  <visual>
    <headlight ambient="0.5 0.5 0.5"/>
  </visual>
  
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1="0.2 0.3 0.4" rgb2="0.3 0.4 0.5"/>
    <material name="grid" texture="grid" texrepeat="1 1" reflectance="0.2"/>
    <material name="finger" rgba="0.8 0.3 0.3 1.0"/>
  </asset>
  
  <worldbody>
    <!-- Ground plane -->
    <geom name="floor" type="plane" size="2 2 0.1" material="grid" 
          condim="6" friction="{scenario.floor_friction} 0.005 0.0001" solref="0.02 {scenario.floor_stiffness}"/>
    
    <!-- Sphere -->
    <body name="fingertip" pos="0 0 {scenario.initial_height}">
      <freejoint/>
      <geom name="finger" type="sphere" size="{scenario.sphere_radius}" mass="{scenario.sphere_mass}" material="finger"
            condim="6" friction="{scenario.floor_friction} 0.005 0.0001" solref="0.02 {scenario.floor_stiffness}"/>
      <site name="contact_site" pos="0 0 0" size="0.001"/>
    </body>
  </worldbody>
  
  <sensor>
    <touch name="finger_touch" site="contact_site"/>
    <framepos name="finger_pos" objtype="site" objname="contact_site"/>
    <framequat name="finger_quat" objtype="site" objname="contact_site"/>
    <framelinvel name="finger_vel" objtype="site" objname="contact_site"/>
  </sensor>
</mujoco>
"""
        
        with open(output_path, 'w') as f:
            f.write(xml_content)
        
        return output_path