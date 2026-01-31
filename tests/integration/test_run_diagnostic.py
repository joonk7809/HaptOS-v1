import sys
sys.path.append('src')

import mujoco
import numpy as np

def test_diagnostic():
    """Diagnose why contact isn't detected"""
    print("Loading model...")
    model = mujoco.MjModel.from_xml_path("assets/contact_scene.xml")
    data = mujoco.MjData(model)
    
    print(f"\nModel info:")
    print(f"  Timestep: {model.opt.timestep}s ({1/model.opt.timestep:.0f} Hz)")
    print(f"  Gravity: {model.opt.gravity}")
    print(f"  Number of bodies: {model.nbody}")
    print(f"  Number of geoms: {model.ngeom}")
    print(f"  Number of sensors: {model.nsensor}")
    
    print(f"\nInitial state:")
    print(f"  Sphere position: {data.qpos[:3]}")
    print(f"  Sphere velocity: {data.qvel[:3]}")
    
    print(f"\nRunning simulation...")
    
    for step in range(1000):
        mujoco.mj_step(model, data)
        
        t = step * model.opt.timestep
        z_pos = data.qpos[2]
        z_vel = data.qvel[2]
        n_contacts = data.ncon
        
        # Print every 100ms
        if step % 100 == 0:
            print(f"  t={t:.3f}s: z={z_pos:.4f}m, v_z={z_vel:.3f}m/s, contacts={n_contacts}")
        
        # Detailed print when contact happens
        if n_contacts > 0 and step > 0 and data.ncon == 1:
            print(f"\n  *** CONTACT DETECTED at t={t:.3f}s ***")
            contact = data.contact[0]
            print(f"      Position: {contact.pos}")
            print(f"      Distance: {contact.dist:.6f}")
            print(f"      Geoms: {contact.geom1}, {contact.geom2}")
            
            # Get sensor data
            if model.nsensor >= 2:
                touch = data.sensordata[0]
                force = data.sensordata[1:4]
                print(f"      Touch sensor: {touch:.4f}")
                print(f"      Force sensor: {force}")
    
    print(f"\n✓ Diagnostic complete")

if __name__ == "__main__":
    test_diagnostic()