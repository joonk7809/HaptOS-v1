import sys
sys.path.append('src')

from data_generator.phase2_dataset import Phase2DatasetManager

def test_phase2_dataset():
    """Convert JSON tunings to HDF5 format"""
    
    manager = Phase2DatasetManager()
    
    # Convert synthetic expert tunings
    dataset_path = manager.convert_json_to_hdf5(
        json_path="data/phase2_tunings/synthetic_expert_001_tunings.json",
        dataset_name="phase2_deltas_v1"
    )
    
    print(f"\n✓ Phase 2 dataset ready for training!")
    print(f"  Location: {dataset_path}")
    print(f"\nNext: Train NN_v1 on these deltas")

if __name__ == "__main__":
    test_phase2_dataset()