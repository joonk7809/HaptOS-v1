import sys
sys.path.append('src')

from data_generator.batch_generator import BatchDataGenerator

def test_batch_generation():
    """Test complete batch data generation"""
    
    generator = BatchDataGenerator(output_dir="data/processed")
    
    # Generate a small test dataset
    print("Generating Phase 1 training dataset...")
    print("(This will take a few minutes)\n")
    
    dataset_name = generator.generate_dataset(
        n_drop=3,    # 3 drop scenarios
        n_tap=2,     # 2 tap scenarios
        n_throw=2,   # 2 throw scenarios
        dataset_name="phase1_test"
    )
    
    print(f"\n{'='*60}")
    print("DATASET GENERATION COMPLETE!")
    print(f"{'='*60}")
    print(f"\nDataset saved as: {dataset_name}")
    print("\nYou can now use this dataset to train your neural network!")
    
    # Test loading
    print("\nTesting dataset loading...")
    data = generator.dataset_manager.load_dataset(dataset_name)
    
    print(f"\nLoaded data shapes:")
    print(f"  X['normal_force_log']: {data['X']['normal_force_log'].shape}")
    print(f"  Y['impact']['A']: {data['Y']['impact']['A'].shape}")
    print(f"  Y['weight']['A']: {data['Y']['weight']['A'].shape}")
    
    print("\n✓ All tests passed!")

if __name__ == "__main__":
    test_batch_generation()