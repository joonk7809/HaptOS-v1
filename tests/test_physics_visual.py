import sys
sys.path.append('src')

import mujoco
import mujoco.viewer
import time

def test_visual():
    """
    Visualize the MuJoCo simulation with interactive viewer.
    You should see a red sphere fall and bounce on the ground.
    """
    print("Loading MuJoCo model...")
    model = mujoco.MjModel.from_xml_path("assets/contact_scene.xml")
    data = mujoco.MjData(model)
    
    print("Opening viewer...")
    print("\nControls:")
    print("  - Left mouse: rotate view")
    print("  - Right mouse: zoom")
    print("  - Middle mouse: pan")
    print("  - Space: pause/resume")
    print("  - Backspace: reset simulation")
    print("  - ESC: close viewer\n")
    
    # Launch interactive viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Run simulation
        start_time = time.time()
        while viewer.is_running() and time.time() - start_time < 30:
            step_start = time.time()
            
            # Step physics
            mujoco.mj_step(model, data)
            
            # Update viewer
            viewer.sync()
            
            # Maintain real-time (1 kHz physics -> 1ms per step)
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    
    print("\n✓ Visual test complete")

if __name__ == "__main__":
    test_visual()