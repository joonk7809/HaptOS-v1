"""
Manages creation and storage of training datasets.
Handles batching, saving, and loading of (X, Y) pairs.
"""

import numpy as np
import json
import h5py
from pathlib import Path
from typing import List, Dict
from dataclasses import asdict
import time

class DatasetManager:
    """
    Manages training dataset storage in HDF5 format.
    HDF5 is much more efficient than JSON for large datasets.
    """
    
    def __init__(self, output_dir: str = "data/processed"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Statistics
        self.total_samples = 0
        self.datasets_created = 0
        
    def save_dataset(self, samples: List, dataset_name: str, metadata: Dict = None):
        """
        Save training samples to HDF5 file.
        
        Args:
            samples: List of TrainingSample objects
            dataset_name: Name for this dataset (e.g., "sphere_drop_001")
            metadata: Optional metadata dict
        """
        if len(samples) == 0:
            print("Warning: No samples to save")
            return
        
        filepath = self.output_dir / f"{dataset_name}.h5"
        
        print(f"\nSaving {len(samples)} samples to {filepath}...")
        
        with h5py.File(filepath, 'w') as f:
            # Create groups for inputs (X) and targets (Y)
            grp_x = f.create_group('X')
            grp_y = f.create_group('Y')
            
            # Extract and stack all X features
            timestamps = [s.X['timestamp_us'] for s in samples]
            
            # Handle One-Hot Phase (reconstruct from string if needed, or use one-hot if available)
            # The Converter now returns 'phase' as a string in the dict, but we need one-hot for training.
            # Let's reconstruct one-hot here to be safe and consistent.
            phases_one_hot = []
            for s in samples:
                p_map = {
                    "PHASE_NO_CONTACT": 0,
                    "PHASE_IMPACT": 1,
                    "PHASE_HOLD": 2,
                    "PHASE_SLIP": 3,
                    "PHASE_RELEASE": 4
                }
                idx = p_map.get(s.X['phase'], 0)
                vec = [0.0] * 5
                vec[idx] = 1.0
                phases_one_hot.append(vec)
            
            phase_confidences = [s.X['phase_confidence_01'] for s in samples]
            
            # Forces are passed as linear in the dict (for JSON compat), so we re-log them for training
            normal_forces_log = [np.log10(s.X['normal_force_N'] + 1.0) for s in samples]
            shear_forces_log = [np.log10(s.X['shear_force_N'] + 1.0) for s in samples]
            
            slip_speeds = [s.X['slip_speed_mms'] for s in samples]
            uncertainties = [s.X['uncertainty_pct'] for s in samples]
            
            # Material features
            hardness = [s.X['material_features']['hardness_01'] for s in samples]
            mu = [s.X['material_features']['mu_01'] for s in samples]
            roughness = [s.X['material_features']['roughness_rms_um'] for s in samples]
            
            # Save X data
            grp_x.create_dataset('timestamp_us', data=np.array(timestamps, dtype=np.int64))
            grp_x.create_dataset('phase_one_hot', data=np.array(phases_one_hot, dtype=np.float32))
            grp_x.create_dataset('phase_confidence', data=np.array(phase_confidences, dtype=np.float32))
            grp_x.create_dataset('normal_force_log', data=np.array(normal_forces_log, dtype=np.float32))
            grp_x.create_dataset('shear_force_log', data=np.array(shear_forces_log, dtype=np.float32))
            grp_x.create_dataset('slip_speed_mms', data=np.array(slip_speeds, dtype=np.float32))
            grp_x.create_dataset('hardness', data=np.array(hardness, dtype=np.float32))
            grp_x.create_dataset('mu', data=np.array(mu, dtype=np.float32))
            grp_x.create_dataset('roughness', data=np.array(roughness, dtype=np.float32))
            grp_x.create_dataset('uncertainty', data=np.array(uncertainties, dtype=np.float32))
            
            # Extract and stack all Y targets
            # Impact
            impact_A = [s.Y['impact']['A'] for s in samples]
            impact_rise = [s.Y['impact']['rise_ms'] for s in samples]
            impact_fall = [s.Y['impact']['fall_ms'] for s in samples]
            impact_hf = [s.Y['impact']['hf_weight'] for s in samples]
            
            grp_y_impact = grp_y.create_group('impact')
            grp_y_impact.create_dataset('A', data=np.array(impact_A, dtype=np.float32))
            grp_y_impact.create_dataset('rise_ms', data=np.array(impact_rise, dtype=np.float32))
            grp_y_impact.create_dataset('fall_ms', data=np.array(impact_fall, dtype=np.float32))
            grp_y_impact.create_dataset('hf_weight', data=np.array(impact_hf, dtype=np.float32))
            
            # Ring (variable length - store as ragged arrays)
            ring_data = self._pack_ring_data(samples)
            grp_y_ring = grp_y.create_group('ring')
            grp_y_ring.create_dataset('f_Hz', data=ring_data['f_Hz'])
            grp_y_ring.create_dataset('tau_ms', data=ring_data['tau_ms'])
            grp_y_ring.create_dataset('a', data=ring_data['a'])
            grp_y_ring.create_dataset('n_modes', data=ring_data['n_modes'])
            
            # Shear
            shear_A = [s.Y['shear']['A'] for s in samples]
            shear_band = [s.Y['shear']['band_Hz'][0] if len(s.Y['shear']['band_Hz']) > 0 else 100.0 
                         for s in samples]
            
            grp_y_shear = grp_y.create_group('shear')
            grp_y_shear.create_dataset('A', data=np.array(shear_A, dtype=np.float32))
            grp_y_shear.create_dataset('band_Hz', data=np.array(shear_band, dtype=np.float32))
            
            # Weight
            weight_A = [s.Y['weight']['A'] for s in samples]
            weight_rate = [s.Y['weight']['rate_ms'] for s in samples]
            
            grp_y_weight = grp_y.create_group('weight')
            grp_y_weight.create_dataset('A', data=np.array(weight_A, dtype=np.float32))
            grp_y_weight.create_dataset('rate_ms', data=np.array(weight_rate, dtype=np.float32))
            
            # Texture
            texture_A = [s.Y['texture']['A'] for s in samples]
            texture_color = [self._encode_color(s.Y['texture']['color']) for s in samples]
            
            grp_y_texture = grp_y.create_group('texture')
            grp_y_texture.create_dataset('A', data=np.array(texture_A, dtype=np.float32))
            grp_y_texture.create_dataset('color', data=np.array(texture_color, dtype=np.int32))
            
            # Save metadata
            if metadata:
                meta_grp = f.create_group('metadata')
                for key, value in metadata.items():
                    if isinstance(value, (int, float, str)):
                        meta_grp.attrs[key] = value
            
            # Default metadata
            f.attrs['n_samples'] = len(samples)
            f.attrs['created_timestamp'] = time.time()
            f.attrs['dataset_name'] = dataset_name
        
        self.total_samples += len(samples)
        self.datasets_created += 1
        
        # Get file size
        file_size_mb = filepath.stat().st_size / (1024 * 1024)
        print(f"✓ Saved {len(samples)} samples ({file_size_mb:.2f} MB)")
        
    def _encode_phase(self, phase_str: str) -> int:
        """Convert phase string to integer"""
        phase_map = {
            "PHASE_NO_CONTACT": 0,
            "PHASE_IMPACT": 1,
            "PHASE_HOLD": 2,
            "PHASE_SLIP": 3,
            "PHASE_RELEASE": 4
        }
        return phase_map.get(phase_str, 0)
    
    def _encode_color(self, color_str: str) -> int:
        """Convert color string to integer"""
        color_map = {
            "COLOR_WHITE": 0,
            "COLOR_PINK": 1,
            "COLOR_BROWN": 2
        }
        return color_map.get(color_str, 1)
    
    def _pack_ring_data(self, samples: List) -> Dict:
        """
        Pack variable-length ring data into fixed arrays.
        Pads with zeros up to max 3 modes.
        """
        max_modes = 3
        n_samples = len(samples)
        
        f_Hz = np.zeros((n_samples, max_modes), dtype=np.float32)
        tau_ms = np.zeros((n_samples, max_modes), dtype=np.float32)
        a = np.zeros((n_samples, max_modes), dtype=np.float32)
        n_modes = np.zeros(n_samples, dtype=np.int32)
        
        for i, sample in enumerate(samples):
            ring = sample.Y['ring']
            n = len(ring['f_Hz'])
            n_modes[i] = n
            
            if n > 0:
                f_Hz[i, :n] = ring['f_Hz'][:max_modes]
                tau_ms[i, :n] = ring['tau_ms'][:max_modes]
                a[i, :n] = ring['a'][:max_modes]
        
        return {
            'f_Hz': f_Hz,
            'tau_ms': tau_ms,
            'a': a,
            'n_modes': n_modes
        }
    
    def load_dataset(self, dataset_name: str) -> Dict:
        """
        Load a dataset from HDF5.
        
        Returns:
            Dict with 'X' and 'Y' numpy arrays
        """
        filepath = self.output_dir / f"{dataset_name}.h5"
        
        if not filepath.exists():
            raise FileNotFoundError(f"Dataset not found: {filepath}")
        
        print(f"Loading dataset: {filepath}")
        
        with h5py.File(filepath, 'r') as f:
            # Load X
            X = {
                'timestamp_us': f['X/timestamp_us'][:],
                'phase_one_hot': f['X/phase_one_hot'][:],
                'phase_confidence': f['X/phase_confidence'][:],
                'normal_force_log': f['X/normal_force_log'][:],
                'shear_force_log': f['X/shear_force_log'][:],
                'slip_speed_mms': f['X/slip_speed_mms'][:],
                'hardness': f['X/hardness'][:],
                'mu': f['X/mu'][:],
                'roughness': f['X/roughness'][:],
                'uncertainty': f['X/uncertainty'][:]
            }
            
            # Load Y
            Y = {
                'impact': {
                    'A': f['Y/impact/A'][:],
                    'rise_ms': f['Y/impact/rise_ms'][:],
                    'fall_ms': f['Y/impact/fall_ms'][:],
                    'hf_weight': f['Y/impact/hf_weight'][:]
                },
                'ring': {
                    'f_Hz': f['Y/ring/f_Hz'][:],
                    'tau_ms': f['Y/ring/tau_ms'][:],
                    'a': f['Y/ring/a'][:],
                    'n_modes': f['Y/ring/n_modes'][:]
                },
                'shear': {
                    'A': f['Y/shear/A'][:],
                    'band_Hz': f['Y/shear/band_Hz'][:]
                },
                'weight': {
                    'A': f['Y/weight/A'][:],
                    'rate_ms': f['Y/weight/rate_ms'][:]
                },
                'texture': {
                    'A': f['Y/texture/A'][:],
                    'color': f['Y/texture/color'][:]
                }
            }
            
            n_samples = f.attrs['n_samples']
            print(f"✓ Loaded {n_samples} samples")
            
            return {'X': X, 'Y': Y, 'metadata': dict(f.attrs)}
    
    def list_datasets(self) -> List[str]:
        """List all available datasets"""
        datasets = list(self.output_dir.glob("*.h5"))
        return [d.stem for d in datasets]
    
    def get_statistics(self):
        """Print dataset statistics"""
        datasets = self.list_datasets()
        print(f"\n=== Dataset Statistics ===")
        print(f"Total datasets: {len(datasets)}")
        print(f"Datasets created this session: {self.datasets_created}")
        print(f"Samples created this session: {self.total_samples}")
        print(f"\nAvailable datasets:")
        for ds in datasets:
            print(f"  - {ds}")