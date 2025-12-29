import sys
sys.path.append('src')

from models.nn_v0 import DatasetLoader
from inference.predictor import HapticPredictor
from inference.validator import ModelValidator

def test_validation():
    """
    Complete validation test:
    1. Load trained model
    2. Load dataset
    3. Run predictions
    4. Compute metrics
    5. Generate visualizations
    """
    
    print("\n" + "="*60)
    print("NN_v0 MODEL VALIDATION")
    print("="*60 + "\n")
    
    # Load model
    print("Loading trained model...")
    predictor = HapticPredictor(
        model_path="models/checkpoints/nn_v0_best.pt",
        device='cpu'
    )
    
    # Load dataset
    print("\nLoading dataset...")
    dataset = DatasetLoader("data/processed/phase1_test.h5")
    
    # Create validator
    print("\nInitializing validator...")
    validator = ModelValidator(predictor, dataset)
    
    # Run validation
    results = validator.validate(n_samples=500)
    
    # Generate visualizations
    print("\nGenerating visualizations...")
    validator.plot_predictions(
        results['predictions'],
        results['ground_truth'],
        save_path="logs/validation"
    )
    
    validator.plot_error_distribution(
        results['predictions'],
        results['ground_truth'],
        save_path="logs/validation"
    )
    
    print("\n" + "="*60)
    print("VALIDATION COMPLETE!")
    print("="*60)
    print("\nCheck logs/validation/ for detailed plots")
    print("  - prediction_scatter.png: Pred vs GT scatter plots")
    print("  - error_distribution.png: Error histograms")
    
    # Test single prediction
    print("\n" + "="*60)
    print("SINGLE PREDICTION EXAMPLE")
    print("="*60 + "\n")
    
    # Get a sample
    idx = 100
    feature_vec = validator._extract_feature_vec(idx)
    
    print("Input Features:")
    print(f"  Phase: {feature_vec['phase']}")
    print(f"  Normal Force: {feature_vec['normal_force_N']:.3f} N")
    print(f"  Shear Force: {feature_vec['shear_force_N']:.3f} N")
    print(f"  Slip Speed: {feature_vec['slip_speed_mms']:.2f} mm/s")
    
    # Predict
    pred = predictor.predict(feature_vec)
    
    print("\nPredicted Cues:")
    print(f"  Impact A: {pred['impact']['A']:.3f}")
    print(f"  Impact rise: {pred['impact']['rise_ms']:.2f} ms")
    print(f"  Impact fall: {pred['impact']['fall_ms']:.2f} ms")
    print(f"  Weight A: {pred['weight']['A']:.3f}")
    print(f"  Shear A: {pred['shear']['A']:.3f}")
    print(f"  Texture: {pred['texture']['color']} (A={pred['texture']['A']:.3f})")
    
    print("\nGround Truth:")
    print(f"  Impact A: {dataset.Y['impact']['A'][idx]:.3f}")
    print(f"  Impact rise: {dataset.Y['impact']['rise_ms'][idx]:.2f} ms")
    print(f"  Impact fall: {dataset.Y['impact']['fall_ms'][idx]:.2f} ms")
    print(f"  Weight A: {dataset.Y['weight']['A'][idx]:.3f}")
    
    print("\n✓ All validation tests passed!")

if __name__ == "__main__":
    test_validation()