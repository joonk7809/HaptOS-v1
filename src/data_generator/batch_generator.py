"""
Batch generator: Runs multiple scenarios and creates training datasets.
"""

import sys
sys.path.append('src')

from pathlib import Path
from typing import List
import tempfile
import os

from sync_runner import SynchronizedRunner
from converter.converter import Converter
from data_generator.dataset_manager import DatasetManager
from data_generator.scenario_generator import ScenarioGenerator, ScenarioConfig
from data_generator.noise_injector import NoiseInjector

class BatchDataGenerator:
    """
    Orchestrates large-scale data generation.
    Runs multiple scenarios, generates training samples, saves datasets.
    """
    
    def __init__(self, output_dir: str = "data/processed"):
        self.dataset_manager = DatasetManager(output_dir)
        self.scenario_generator = ScenarioGenerator()
        self.converter = Converter()
        self.noise_injector = NoiseInjector()
        
        # Create temp directory for scenario XMLs
        self.temp_dir = Path("data/temp_scenarios")
        self.temp_dir.mkdir(parents=True, exist_ok=True)
        
    def generate_dataset(self, 
                        n_drop: int = 5,
                        n_tap: int = 5,
                        n_throw: int = 5,
                        dataset_name: str = "phase1_v0") -> str:
        """
        Generate a complete training dataset.
        
        Args:
            n_drop: Number of drop scenarios
            n_tap: Number of tap scenarios  
            n_throw: Number of throw scenarios
            dataset_name: Name for the output dataset
            
        Returns:
            Path to saved dataset
        """
        print(f"\n{'='*60}")
        print(f"PHASE 1 DATA GENERATION: {dataset_name}")
        print(f"{'='*60}")
        
        # Generate scenarios
        print(f"\nGenerating scenarios...")
        scenarios = []
        scenarios.extend(self.scenario_generator.generate_drop_scenarios(n_drop))
        scenarios.extend(self.scenario_generator.generate_tap_scenarios(n_tap))
        scenarios.extend(self.scenario_generator.generate_throw_scenarios(n_throw))
        
        print(f"✓ Created {len(scenarios)} scenarios")
        
        # Run each scenario and collect samples
        all_samples = []
        
        for i, scenario in enumerate(scenarios):
            print(f"\n--- Scenario {i+1}/{len(scenarios)}: {scenario.name} ---")
            print(f"  Height: {scenario.initial_height:.2f}m, "
                  f"Mass: {scenario.sphere_mass:.3f}kg, "
                  f"Radius: {scenario.sphere_radius:.3f}m")
            
            # Create XML for this scenario
            xml_path = self.temp_dir / f"{scenario.name}.xml"
            self.scenario_generator.create_xml_for_scenario(scenario, str(xml_path))
            
            # Run simulation
            samples = self._run_scenario(scenario, str(xml_path))
            all_samples.extend(samples)
            
            print(f"  ✓ Generated {len(samples)} samples (Total: {len(all_samples)})")
        
        # Save complete dataset
        print(f"\n{'='*60}")
        metadata = {
            'n_scenarios': len(scenarios),
            'n_drop': n_drop,
            'n_tap': n_tap,
            'n_throw': n_throw,
            'version': 'phase1_v0'
        }
        
        self.dataset_manager.save_dataset(all_samples, dataset_name, metadata)
        self.dataset_manager.get_statistics()
        
        print(f"{'='*60}\n")
        
        return dataset_name
    
    def _run_scenario(self, scenario: ScenarioConfig, xml_path: str) -> List:
        """Run a single scenario and return training samples"""
        # Reset converter for new scenario
        self.converter.reset()
        
        # Run simulation
        runner = SynchronizedRunner(xml_path)
        clean_history, audio = runner.run_episode(duration_s=scenario.duration_s)
        
        # Inject Sim-to-Real Noise
        contact_history = self.noise_injector.apply_noise(clean_history)
        
        # Process with converter
        samples_per_window_physics = 10
        samples_per_window_audio = 480
        n_windows = len(contact_history) // samples_per_window_physics
        
        samples = []
        for i in range(n_windows):
            phys_start = i * samples_per_window_physics
            phys_end = phys_start + samples_per_window_physics
            physics_window = contact_history[phys_start:phys_end]
            
            audio_start = i * samples_per_window_audio
            audio_end = audio_start + samples_per_window_audio
            
            if audio_end > len(audio):
                break
                
            audio_window = audio[audio_start:audio_end]
            window_start_us = physics_window[0].timestamp_us
            
            sample = self.converter.process_window(
                physics_window,
                audio_window,
                window_start_us
            )
            samples.append(sample)
        
        return samples