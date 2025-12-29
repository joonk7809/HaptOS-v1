"""
Generate all architecture diagrams for documentation.
Requires: matplotlib, numpy
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle, Rectangle
import numpy as np

def create_system_architecture():
    """Overall system architecture diagram"""
    fig, ax = plt.subplots(figsize=(14, 8))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 8)
    ax.axis('off')
    
    # Title
    ax.text(7, 7.5, 'Two-Phase Haptic Synthesis Architecture', 
            fontsize=18, fontweight='bold', ha='center')
    
    # Phase 1 Section
    phase1_box = FancyBboxPatch((0.5, 4), 6, 2.5, 
                                boxstyle="round,pad=0.1", 
                                edgecolor='blue', facecolor='lightblue', 
                                linewidth=2, alpha=0.3)
    ax.add_patch(phase1_box)
    ax.text(3.5, 6.3, 'Phase 1: Physics-Driven Baseline', 
            fontsize=12, fontweight='bold', ha='center')
    
    # Phase 1 Components
    # Physics Sim
    physics = FancyBboxPatch((0.8, 4.8), 1.5, 1, 
                            boxstyle="round,pad=0.05",
                            edgecolor='darkblue', facecolor='lightblue', linewidth=1.5)
    ax.add_patch(physics)
    ax.text(1.55, 5.5, 'Physics', ha='center', fontsize=9, fontweight='bold')
    ax.text(1.55, 5.2, '1 kHz', ha='center', fontsize=7)
    
    # Audio Synth
    audio = FancyBboxPatch((0.8, 4.3), 1.5, 0.4,
                          boxstyle="round,pad=0.05",
                          edgecolor='darkblue', facecolor='lightcyan', linewidth=1.5)
    ax.add_patch(audio)
    ax.text(1.55, 4.5, 'Audio 48kHz', ha='center', fontsize=7)
    
    # Converter
    converter = FancyBboxPatch((2.8, 4.5), 1.2, 1,
                              boxstyle="round,pad=0.05",
                              edgecolor='darkgreen', facecolor='lightgreen', linewidth=1.5)
    ax.add_patch(converter)
    ax.text(3.4, 5.2, 'Converter', ha='center', fontsize=9, fontweight='bold')
    ax.text(3.4, 4.9, '100 Hz', ha='center', fontsize=7)
    
    # NN_v0
    nn_v0 = FancyBboxPatch((4.5, 4.5), 1.5, 1,
                          boxstyle="round,pad=0.05",
                          edgecolor='purple', facecolor='plum', linewidth=2)
    ax.add_patch(nn_v0)
    ax.text(5.25, 5.2, 'NN_v0', ha='center', fontsize=10, fontweight='bold')
    ax.text(5.25, 4.9, 'Baseline', ha='center', fontsize=8)
    
    # Phase 2 Section
    phase2_box = FancyBboxPatch((0.5, 0.5), 6, 2.5,
                                boxstyle="round,pad=0.1",
                                edgecolor='red', facecolor='mistyrose',
                                linewidth=2, alpha=0.3)
    ax.add_patch(phase2_box)
    ax.text(3.5, 2.8, 'Phase 2: Expert Refinement',
            fontsize=12, fontweight='bold', ha='center')
    
    # Expert Tuning
    expert = FancyBboxPatch((0.8, 1.3), 1.5, 0.8,
                           boxstyle="round,pad=0.05",
                           edgecolor='darkred', facecolor='lightcoral', linewidth=1.5)
    ax.add_patch(expert)
    ax.text(1.55, 1.7, 'Expert', ha='center', fontsize=9, fontweight='bold')
    ax.text(1.55, 1.5, 'Tuning', ha='center', fontsize=8)
    
    # Delta Dataset
    delta_data = FancyBboxPatch((2.8, 1.3), 1.2, 0.8,
                               boxstyle="round,pad=0.05",
                               edgecolor='darkgreen', facecolor='palegreen', linewidth=1.5)
    ax.add_patch(delta_data)
    ax.text(3.4, 1.7, 'Δ Dataset', ha='center', fontsize=9, fontweight='bold')
    
    # NN_v1
    nn_v1 = FancyBboxPatch((4.5, 1.3), 1.5, 0.8,
                          boxstyle="round,pad=0.05",
                          edgecolor='purple', facecolor='thistle', linewidth=2)
    ax.add_patch(nn_v1)
    ax.text(5.25, 1.7, 'NN_v1', ha='center', fontsize=10, fontweight='bold')
    ax.text(5.25, 1.5, 'Delta', ha='center', fontsize=8)
    
    # Runtime Section
    runtime_box = FancyBboxPatch((7.5, 2), 6, 4,
                                boxstyle="round,pad=0.1",
                                edgecolor='green', facecolor='honeydew',
                                linewidth=2, alpha=0.3)
    ax.add_patch(runtime_box)
    ax.text(10.5, 5.7, 'Runtime Inference',
            fontsize=12, fontweight='bold', ha='center')
    
    # Input Features
    features = FancyBboxPatch((8, 4.5), 1.8, 0.8,
                             boxstyle="round,pad=0.05",
                             edgecolor='black', facecolor='lightgray', linewidth=1.5)
    ax.add_patch(features)
    ax.text(8.9, 5, 'Features', ha='center', fontsize=9, fontweight='bold')
    ax.text(8.9, 4.7, '(9D)', ha='center', fontsize=7)
    
    # Combined Predictor
    combined = FancyBboxPatch((10.3, 4), 2, 1.5,
                             boxstyle="round,pad=0.05",
                             edgecolor='darkviolet', facecolor='lavender', linewidth=2)
    ax.add_patch(combined)
    ax.text(11.3, 5, 'NN_v0 + NN_v1', ha='center', fontsize=10, fontweight='bold')
    ax.text(11.3, 4.6, 'Combined', ha='center', fontsize=8)
    ax.text(11.3, 4.3, 'Predictor', ha='center', fontsize=8)
    
    # Output Cues
    output = FancyBboxPatch((10.3, 2.5), 2, 1,
                           boxstyle="round,pad=0.05",
                           edgecolor='darkgreen', facecolor='lightgreen', linewidth=1.5)
    ax.add_patch(output)
    ax.text(11.3, 3.2, 'Haptic Cues', ha='center', fontsize=9, fontweight='bold')
    ax.text(11.3, 2.85, '5 types', ha='center', fontsize=7)
    
    # Arrows - Phase 1
    arrow1 = FancyArrowPatch((2.3, 5.3), (2.8, 5.2),
                            arrowstyle='->', mutation_scale=20, lw=2, color='blue')
    ax.add_patch(arrow1)
    
    arrow2 = FancyArrowPatch((4, 5), (4.5, 5),
                            arrowstyle='->', mutation_scale=20, lw=2, color='blue')
    ax.add_patch(arrow2)
    
    # Arrows - Phase 2
    arrow3 = FancyArrowPatch((2.3, 1.7), (2.8, 1.7),
                            arrowstyle='->', mutation_scale=20, lw=2, color='red')
    ax.add_patch(arrow3)
    
    arrow4 = FancyArrowPatch((4, 1.7), (4.5, 1.7),
                            arrowstyle='->', mutation_scale=20, lw=2, color='red')
    ax.add_patch(arrow4)
    
    # Arrows - Runtime
    arrow5 = FancyArrowPatch((9.8, 4.9), (10.3, 4.9),
                            arrowstyle='->', mutation_scale=20, lw=2, color='green')
    ax.add_patch(arrow5)
    
    arrow6 = FancyArrowPatch((11.3, 4), (11.3, 3.5),
                            arrowstyle='->', mutation_scale=20, lw=2, color='green')
    ax.add_patch(arrow6)
    
    # Connection from Phase 1 to Runtime
    arrow7 = FancyArrowPatch((6, 5), (8, 4.9),
                            arrowstyle='->', mutation_scale=15, lw=1.5, 
                            color='blue', linestyle='dashed', alpha=0.6)
    ax.add_patch(arrow7)
    ax.text(7, 5.2, 'trained', fontsize=7, ha='center', style='italic')
    
    # Connection from Phase 2 to Runtime
    arrow8 = FancyArrowPatch((6, 1.7), (10.3, 4.5),
                            arrowstyle='->', mutation_scale=15, lw=1.5,
                            color='red', linestyle='dashed', alpha=0.6)
    ax.add_patch(arrow8)
    ax.text(8, 2.8, 'trained', fontsize=7, ha='center', style='italic')
    
    # Performance annotation
    perf_box = FancyBboxPatch((8, 2.2), 2, 0.6,
                             boxstyle="round,pad=0.05",
                             edgecolor='gold', facecolor='lightyellow', linewidth=2)
    ax.add_patch(perf_box)
    ax.text(9, 2.65, '89.1%', ha='center', fontsize=12, fontweight='bold', color='darkred')
    ax.text(9, 2.35, 'Improvement', ha='center', fontsize=8)
    
    plt.tight_layout()
    plt.savefig('docs/diagrams/system_architecture.png', dpi=300, bbox_inches='tight')
    print("✓ Created: system_architecture.png")
    plt.close()


def create_data_flow():
    """Data flow through the pipeline"""
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 10)
    ax.axis('off')
    
    ax.text(6, 9.5, 'Data Flow Pipeline', fontsize=18, fontweight='bold', ha='center')
    
    # Level 1: Physics
    physics_box = Rectangle((1, 8), 2, 0.8, facecolor='lightblue', edgecolor='darkblue', linewidth=2)
    ax.add_patch(physics_box)
    ax.text(2, 8.4, 'MuJoCo Sim', ha='center', fontsize=10, fontweight='bold')
    ax.text(2, 8.1, '1000 Hz', ha='center', fontsize=8)
    
    # Data representation
    ax.text(4.5, 8.4, 'ContactPatch:', ha='left', fontsize=8, style='italic')
    ax.text(4.5, 8.15, '• normal_force_N', ha='left', fontsize=7)
    ax.text(4.5, 7.95, '• shear_force_N', ha='left', fontsize=7)
    ax.text(4.5, 7.75, '• slip_speed_mms', ha='left', fontsize=7)
    
    # Arrow down
    ax.arrow(2, 8, 0, -0.8, head_width=0.2, head_length=0.1, fc='black', ec='black')
    
    # Level 2: Audio
    audio_box = Rectangle((1, 6.5), 2, 0.8, facecolor='lightcyan', edgecolor='darkcyan', linewidth=2)
    ax.add_patch(audio_box)
    ax.text(2, 6.9, 'Audio Synth', ha='center', fontsize=10, fontweight='bold')
    ax.text(2, 6.6, '48000 Hz', ha='center', fontsize=8)
    
    ax.text(4.5, 6.9, 'Audio Signal:', ha='left', fontsize=8, style='italic')
    ax.text(4.5, 6.65, '• 480 samples/window', ha='left', fontsize=7)
    ax.text(4.5, 6.45, '• Modal synthesis', ha='left', fontsize=7)
    
    # Arrow down
    ax.arrow(2, 6.5, 0, -0.8, head_width=0.2, head_length=0.1, fc='black', ec='black')
    
    # Level 3: Converter
    converter_box = Rectangle((0.5, 5), 3, 0.8, facecolor='lightgreen', edgecolor='darkgreen', linewidth=2)
    ax.add_patch(converter_box)
    ax.text(2, 5.4, 'Converter (100 Hz)', ha='center', fontsize=10, fontweight='bold')
    
    ax.text(4.5, 5.4, 'Training Pair:', ha='left', fontsize=8, style='italic')
    ax.text(4.5, 5.15, 'X: FeatureVec (9D)', ha='left', fontsize=7)
    ax.text(4.5, 4.95, 'Y: CueParams', ha='left', fontsize=7)
    
    # Split into two paths
    # Path 1: Phase 1
    ax.arrow(1.2, 5, 0, -1.3, head_width=0.2, head_length=0.1, fc='blue', ec='blue', lw=2)
    ax.text(0.5, 4.2, 'Phase 1', fontsize=9, fontweight='bold', color='blue')
    
    nn_v0_box = Rectangle((0.3, 2.8), 2, 1, facecolor='plum', edgecolor='purple', linewidth=2)
    ax.add_patch(nn_v0_box)
    ax.text(1.3, 3.5, 'NN_v0', ha='center', fontsize=11, fontweight='bold')
    ax.text(1.3, 3.2, 'Baseline', ha='center', fontsize=8)
    ax.text(1.3, 2.95, '~27K params', ha='center', fontsize=7)
    
    # Path 2: Phase 2
    ax.arrow(2.8, 5, 0, -1.3, head_width=0.2, head_length=0.1, fc='red', ec='red', lw=2)
    ax.text(3.5, 4.2, 'Phase 2', fontsize=9, fontweight='bold', color='red')
    
    expert_box = Rectangle((2.6, 3.3), 1.6, 0.5, facecolor='lightcoral', edgecolor='darkred', linewidth=2)
    ax.add_patch(expert_box)
    ax.text(3.4, 3.55, 'Expert Tuning', ha='center', fontsize=9, fontweight='bold')
    
    ax.arrow(3.4, 3.3, 0, -0.4, head_width=0.15, head_length=0.08, fc='red', ec='red')
    
    nn_v1_box = Rectangle((2.6, 2.2), 1.6, 0.6, facecolor='thistle', edgecolor='purple', linewidth=2)
    ax.add_patch(nn_v1_box)
    ax.text(3.4, 2.65, 'NN_v1', ha='center', fontsize=11, fontweight='bold')
    ax.text(3.4, 2.35, 'Delta', ha='center', fontsize=8)
    
    # Combine
    ax.arrow(2.3, 3.3, 0.5, -0.6, head_width=0, head_length=0, fc='black', ec='black', lw=1.5, linestyle='dashed')
    ax.arrow(3.5, 2.2, -0.5, 0.5, head_width=0, head_length=0, fc='black', ec='black', lw=1.5, linestyle='dashed')
    
    combined_box = Rectangle((1.8, 1.2), 2, 0.8, facecolor='lavender', edgecolor='darkviolet', linewidth=3)
    ax.add_patch(combined_box)
    ax.text(2.8, 1.75, 'Combined', ha='center', fontsize=10, fontweight='bold')
    ax.text(2.8, 1.45, 'v0 + v1', ha='center', fontsize=9)
    
    # Output
    ax.arrow(2.8, 1.2, 0, -0.5, head_width=0.2, head_length=0.1, fc='green', ec='green', lw=2)
    
    output_box = Rectangle((1.5, 0.1), 2.6, 0.5, facecolor='lightgreen', edgecolor='darkgreen', linewidth=2)
    ax.add_patch(output_box)
    ax.text(2.8, 0.35, 'Final Haptic Cues', ha='center', fontsize=10, fontweight='bold')
    
    # Performance metrics on the right
    metrics_box = Rectangle((7, 4), 4, 5, facecolor='lightyellow', edgecolor='gold', linewidth=2)
    ax.add_patch(metrics_box)
    
    ax.text(9, 8.5, 'Performance', ha='center', fontsize=12, fontweight='bold')
    
    metrics = [
        ('Impact Amplitude', '76.4%'),
        ('Weight (Weber\'s Law)', '93.1%'),
        ('Fall Time', '97.7%'),
        ('Average', '89.1%')
    ]
    
    y_pos = 7.8
    for metric, value in metrics:
        ax.text(7.3, y_pos, metric + ':', ha='left', fontsize=9)
        ax.text(10.7, y_pos, value, ha='right', fontsize=10, fontweight='bold', color='darkgreen')
        y_pos -= 0.6
    
    # Add improvement arrow
    ax.annotate('', xy=(9, 5.5), xytext=(9, 6.5),
                arrowprops=dict(arrowstyle='->', lw=3, color='green'))
    ax.text(9, 5.3, 'Improvement', ha='center', fontsize=9, fontweight='bold', color='darkgreen')
    
    plt.tight_layout()
    plt.savefig('docs/diagrams/data_flow.png', dpi=300, bbox_inches='tight')
    print("✓ Created: data_flow.png")
    plt.close()


def create_results_comparison():
    """Before/after comparison visualization"""
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    metrics = ['Impact\nAmplitude', 'Weight\nAmplitude', 'Fall Time\n(ms)']
    baseline_errors = [0.0657, 0.1831, 4.34]
    final_errors = [0.0155, 0.0125, 0.10]
    improvements = [76.4, 93.1, 97.7]
    
    for idx, (ax, metric, baseline, final, improvement) in enumerate(
        zip(axes, metrics, baseline_errors, final_errors, improvements)):
        
        x = [0, 1]
        values = [baseline, final]
        colors = ['#FF6B6B', '#4ECDC4']
        
        bars = ax.bar(x, values, color=colors, width=0.6, edgecolor='black', linewidth=1.5)
        
        ax.set_xticks(x)
        ax.set_xticklabels(['NN_v0\nalone', 'NN_v0 +\nNN_v1'], fontsize=10)
        ax.set_ylabel('Error (MAE)', fontsize=11, fontweight='bold')
        ax.set_title(metric, fontsize=12, fontweight='bold')
        ax.grid(axis='y', alpha=0.3)
        
        # Add value labels on bars
        for i, (bar, val) in enumerate(zip(bars, values)):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{val:.3f}' if idx < 2 else f'{val:.2f}',
                   ha='center', va='bottom', fontsize=9, fontweight='bold')
        
        # Add improvement annotation
        ax.annotate('', xy=(1, final), xytext=(0, baseline),
                   arrowprops=dict(arrowstyle='->', lw=2, color='green', 
                                  linestyle='dashed', alpha=0.7))
        
        mid_x = 0.5
        mid_y = (baseline + final) / 2
        ax.text(mid_x, mid_y, f'↓ {improvement:.1f}%',
               ha='center', va='center', fontsize=11, fontweight='bold',
               color='darkgreen',
               bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen', alpha=0.8))
    
    plt.suptitle('Phase 2 Performance Improvements', fontsize=16, fontweight='bold', y=1.02)
    plt.tight_layout()
    plt.savefig('docs/diagrams/results_comparison.png', dpi=300, bbox_inches='tight')
    print("✓ Created: results_comparison.png")
    plt.close()


def create_network_architecture():
    """Neural network architecture diagram"""
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 8)
    ax.axis('off')
    
    ax.text(6, 7.5, 'Neural Network Architecture', fontsize=16, fontweight='bold', ha='center')
    
    # Input layer
    input_box = Rectangle((1, 6), 1.5, 1, facecolor='lightgray', edgecolor='black', linewidth=2)
    ax.add_patch(input_box)
    ax.text(1.75, 6.7, 'Input', ha='center', fontsize=10, fontweight='bold')
    ax.text(1.75, 6.3, '9D', ha='center', fontsize=9)
    
    # Input features list
    features = ['phase', 'confidence', 'force_n', 'force_s', 'slip',
                'hardness', 'μ', 'roughness', 'uncertainty']
    y_start = 5.5
    for i, feat in enumerate(features):
        ax.text(1.75, y_start - i*0.15, feat, ha='center', fontsize=6)
    
    # Trunk layers
    trunk_layers = [
        {'pos': (3.5, 6), 'size': (1, 1), 'label': 'Linear', 'sublabel': '9→128'},
        {'pos': (5, 6), 'size': (1, 1), 'label': 'ReLU +\nDropout', 'sublabel': '0.2'},
        {'pos': (6.5, 6), 'size': (1, 1), 'label': 'Linear', 'sublabel': '128→128'},
        {'pos': (8, 6), 'size': (1, 1), 'label': 'ReLU +\nDropout', 'sublabel': '0.2'},
        {'pos': (9.5, 6), 'size': (0.8, 1), 'label': 'Linear', 'sublabel': '128→64'}
    ]
    
    for layer in trunk_layers:
        box = Rectangle(layer['pos'], layer['size'][0], layer['size'][1],
                       facecolor='lightblue', edgecolor='darkblue', linewidth=1.5)
        ax.add_patch(box)
        ax.text(layer['pos'][0] + layer['size'][0]/2, layer['pos'][1] + 0.65,
               layer['label'], ha='center', fontsize=8, fontweight='bold')
        ax.text(layer['pos'][0] + layer['size'][0]/2, layer['pos'][1] + 0.25,
               layer['sublabel'], ha='center', fontsize=7)
    
    # Arrows between trunk layers
    for i in range(len(trunk_layers) - 1):
        start_x = trunk_layers[i]['pos'][0] + trunk_layers[i]['size'][0]
        end_x = trunk_layers[i+1]['pos'][0]
        y = trunk_layers[i]['pos'][1] + 0.5
        ax.arrow(start_x, y, end_x - start_x - 0.1, 0,
                head_width=0.15, head_length=0.08, fc='blue', ec='blue')
    
    # Arrow from input to trunk
    ax.arrow(2.5, 6.5, 0.9, 0, head_width=0.15, head_length=0.08, fc='black', ec='black')
    
    ax.text(6, 7, 'Shared Trunk', fontsize=11, fontweight='bold', ha='center',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='lightblue', alpha=0.7))
    
    # Output heads
    heads = [
        {'pos': (1, 3.5), 'label': 'Impact\nHead', 'outputs': '4'},
        {'pos': (2.8, 3.5), 'label': 'Ring\nHead', 'outputs': '9'},
        {'pos': (4.6, 3.5), 'label': 'Shear\nHead', 'outputs': '2'},
        {'pos': (6.4, 3.5), 'label': 'Weight\nHead', 'outputs': '2'},
        {'pos': (8.2, 3.5), 'label': 'Texture\nHead', 'outputs': '4'}
    ]
    
    for head in heads:
        box = Rectangle(head['pos'], 1.5, 0.8, facecolor='plum',
                       edgecolor='purple', linewidth=1.5)
        ax.add_patch(box)
        ax.text(head['pos'][0] + 0.75, head['pos'][1] + 0.55,
               head['label'], ha='center', fontsize=8, fontweight='bold')
        ax.text(head['pos'][0] + 0.75, head['pos'][1] + 0.2,
               f'{head["outputs"]} outputs', ha='center', fontsize=7)
        
        # Arrow from trunk to head
        ax.arrow(9.9, 6.3, head['pos'][0] + 0.75 - 9.9, head['pos'][1] + 0.8 - 6.3,
                head_width=0.1, head_length=0.08, fc='purple', ec='purple',
                linestyle='dashed', alpha=0.6)
    
    # Output descriptions
    outputs = [
        {'x': 1.75, 'text': 'A, rise_ms\nfall_ms, hf_wt'},
        {'x': 3.55, 'text': 'f_Hz×3\nτ_ms×3, a×3'},
        {'x': 5.35, 'text': 'A\nband_Hz'},
        {'x': 7.15, 'text': 'A\nrate_ms'},
        {'x': 8.95, 'text': 'A\ncolor'}
    ]
    
    for out in outputs:
        ax.text(out['x'], 2.7, out['text'], ha='center', fontsize=7,
               bbox=dict(boxstyle='round,pad=0.2', facecolor='lightyellow', alpha=0.7))
    
    # Total parameters
    ax.text(6, 1.5, 'Total Parameters', fontsize=11, fontweight='bold', ha='center')
    ax.text(6, 1.1, 'NN_v0: ~27K  |  NN_v1: ~27K  |  Combined: ~54K',
           fontsize=9, ha='center')
    
    # Key difference annotation
    key_box = Rectangle((0.3, 0.2), 5, 0.6, facecolor='lightcyan',
                       edgecolor='darkcyan', linewidth=2)
    ax.add_patch(key_box)
    ax.text(2.8, 0.65, 'NN_v0: Sigmoid/ReLU outputs (0-1, positive)',
           fontsize=8, ha='center', fontweight='bold')
    ax.text(2.8, 0.35, 'NN_v1: Linear outputs (can be negative for deltas)',
           fontsize=8, ha='center', fontweight='bold', color='red')
    
    plt.tight_layout()
    plt.savefig('docs/diagrams/network_architecture.png', dpi=300, bbox_inches='tight')
    print("✓ Created: network_architecture.png")
    plt.close()


def create_all_diagrams():
    """Generate all diagrams"""
    import os
    os.makedirs('docs/diagrams', exist_ok=True)
    
    print("\nGenerating architecture diagrams...")
    print("="*50)
    
    create_system_architecture()
    create_data_flow()
    create_results_comparison()
    create_network_architecture()
    
    print("="*50)
    print("✓ All diagrams created successfully!")
    print("\nFiles saved in: docs/diagrams/")
    print("  - system_architecture.png")
    print("  - data_flow.png")
    print("  - results_comparison.png")
    print("  - network_architecture.png")


if __name__ == "__main__":
    create_all_diagrams()