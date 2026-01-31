import sys
sys.path.append('src')

from physics.mujoco_engine import MuJoCoEngine
import matplotlib.pyplot as plt

def test_physics_engine():
    engine = MuJoCoEngine("assets/contact_scene.xml")
    
    # Run for 1 second (1000 steps at 1 kHz)
    forces = []
    times = []
    
    for i in range(1000):
        patch = engine.step()
        forces.append(patch.normal_force_N)
        times.append(patch.timestamp_us / 1e6)  # Convert to seconds
        
        if i % 100 == 0:
            print(f"t={patch.timestamp_us/1e3:.1f}ms: "
                  f"F_n={patch.normal_force_N:.3f}N, "
                  f"contact={patch.in_contact}")
    
    # Plot
    plt.figure(figsize=(10, 4))
    plt.plot(times, forces)
    plt.xlabel('Time (s)')
    plt.ylabel('Normal Force (N)')
    plt.title('Physics Simulation: Contact Force Over Time')
    plt.grid(True)
    plt.savefig('logs/physics_test.png')
    plt.close()
    
    print("\n✓ Physics engine test complete. Check logs/physics_test.png")

if __name__ == "__main__":
    test_physics_engine()