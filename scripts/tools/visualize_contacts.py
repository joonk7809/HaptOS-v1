#!/usr/bin/env python3
"""
Contact Visualization Tool
Creates detailed visualizations of contact information.
"""

import sys
sys.path.append('src')

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from pathlib import Path


class ContactVisualizer:
    """Visualize contact information on hand diagram."""

    def __init__(self):
        """Initialize visualizer."""
        self.fig = None
        self.ax = None

    def create_hand_diagram(self):
        """Create 2D hand diagram for contact visualization."""
        self.fig, self.ax = plt.subplots(figsize=(10, 12))
        self.ax.set_xlim(-0.1, 0.3)
        self.ax.set_ylim(-0.15, 0.15)
        self.ax.set_aspect('equal')
        self.ax.set_title("Hand Contact Map", fontsize=16, fontweight='bold')
        self.ax.axis('off')

        # Palm
        palm = patches.Rectangle((-0.04, -0.065), 0.08, 0.13,
                                 linewidth=2, edgecolor='black',
                                 facecolor='#f0d0b0', alpha=0.5)
        self.ax.add_patch(palm)
        self.ax.text(0, 0, 'PALM', ha='center', va='center',
                    fontsize=10, fontweight='bold')

        # Fingers (simplified representation)
        fingers = [
            ('Thumb', -0.05, 0.05, 0.08, 25),
            ('Index', 0.06, 0.045, 0.10, 0),
            ('Middle', 0.06, 0.02, 0.11, 0),
            ('Ring', 0.06, -0.005, 0.10, 0),
            ('Pinky', 0.06, -0.03, 0.08, 0)
        ]

        self.contact_positions = {}

        for name, x_start, y, length, angle in fingers:
            # Draw finger segments
            n_segments = 3
            segment_length = length / n_segments
            colors = ['#f0d0b0', '#e8c8a8', '#e0c0a0']

            for i in range(n_segments):
                x = x_start + i * segment_length
                rect = patches.Rectangle((x, y - 0.008), segment_length, 0.016,
                                        linewidth=1, edgecolor='black',
                                        facecolor=colors[i], alpha=0.5,
                                        angle=angle)
                self.ax.add_patch(rect)

                # Store contact position for this segment
                segment_name = ['proximal', 'middle', 'tip'][i]
                contact_key = f"{name.lower()}_{segment_name}"
                center_x = x + segment_length / 2
                self.contact_positions[contact_key] = (center_x, y)

            # Finger label
            label_x = x_start + length + 0.01
            self.ax.text(label_x, y, name, ha='left', va='center',
                        fontsize=9, style='italic')

        # Palm contact zones
        self.contact_positions['palm_center'] = (0, 0)
        self.contact_positions['palm_thumb_base'] = (-0.035, 0.03)
        self.contact_positions['palm_base'] = (0, -0.05)

        return self.fig, self.ax

    def plot_contact(self, body_part, force, color='red', max_force=10.0):
        """
        Plot a contact point on the hand diagram.

        Args:
            body_part: Name of body part in contact
            force: Contact force in Newtons
            color: Color for contact marker
            max_force: Maximum force for scaling
        """
        if body_part not in self.contact_positions:
            print(f"Warning: Unknown body part: {body_part}")
            return

        x, y = self.contact_positions[body_part]

        # Scale circle size by force
        size = 50 + (force / max_force) * 500

        # Plot contact
        self.ax.scatter(x, y, s=size, c=color, alpha=0.6,
                       edgecolors='darkred', linewidths=2, zorder=10)

        # Add force label
        self.ax.text(x, y, f'{force:.1f}N',
                    ha='center', va='center',
                    fontsize=8, fontweight='bold', color='white')

    def plot_force_vector(self, body_part, force_vector, scale=0.02):
        """
        Plot force vector at contact point.

        Args:
            body_part: Name of body part
            force_vector: [fx, fy, fz] force components
            scale: Scaling factor for arrow length
        """
        if body_part not in self.contact_positions:
            return

        x, y = self.contact_positions[body_part]

        # Project 3D force to 2D (use x and y components)
        fx, fy, fz = force_vector
        dx = fx * scale
        dy = fy * scale

        self.ax.arrow(x, y, dx, dy,
                     head_width=0.01, head_length=0.01,
                     fc='blue', ec='blue', alpha=0.7, linewidth=2)

    def add_legend(self):
        """Add legend explaining visualization."""
        legend_elements = [
            patches.Patch(facecolor='red', alpha=0.6, label='Contact point'),
            patches.Patch(facecolor='blue', alpha=0.7, label='Force direction'),
            patches.Patch(facecolor='#f0d0b0', alpha=0.5, label='Hand surface')
        ]
        self.ax.legend(handles=legend_elements, loc='upper left',
                      fontsize=9, framealpha=0.9)

    def save(self, filename):
        """Save figure to file."""
        output_dir = Path("data/visualizations")
        output_dir.mkdir(parents=True, exist_ok=True)

        output_path = output_dir / filename
        self.fig.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"✓ Saved visualization to {output_path}")

    def show(self):
        """Display figure."""
        plt.tight_layout()
        plt.show()


def visualize_contact_data(contact_dict, title="Contact Visualization"):
    """
    Visualize contact data on hand diagram.

    Args:
        contact_dict: Dict[body_part, ContactPatch]
        title: Title for visualization
    """
    viz = ContactVisualizer()
    viz.create_hand_diagram()
    viz.ax.set_title(title, fontsize=16, fontweight='bold')

    # Plot each contact
    max_force = max([c.normal_force_N for c in contact_dict.values()]) if contact_dict else 10.0

    for body_part, contact in contact_dict.items():
        if contact.normal_force_N > 0.01:
            viz.plot_contact(body_part, contact.normal_force_N, max_force=max_force)

    viz.add_legend()

    # Add statistics
    n_contacts = len(contact_dict)
    total_force = sum(c.normal_force_N for c in contact_dict.values())
    avg_force = total_force / n_contacts if n_contacts > 0 else 0

    stats_text = f"Contacts: {n_contacts}\nTotal Force: {total_force:.2f}N\nAvg Force: {avg_force:.2f}N"
    viz.ax.text(0.25, -0.12, stats_text, fontsize=10,
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    return viz


def demo():
    """Demo visualization with fake contact data."""
    print("=" * 70)
    print(" " * 20 + "CONTACT VISUALIZATION DEMO")
    print("=" * 70)
    print()

    # Create demo contact data
    from dataclasses import dataclass
    import numpy as np

    @dataclass
    class DemoContact:
        normal_force_N: float

    # Simulate contacts
    contact_dict = {
        'palm_center': DemoContact(5.2),
        'thumb_tip': DemoContact(2.1),
        'index_tip': DemoContact(3.4),
        'middle_tip': DemoContact(2.8),
        'index_middle': DemoContact(1.2),
        'middle_proximal': DemoContact(0.8)
    }

    print("Creating visualization with demo contact data...")
    viz = visualize_contact_data(contact_dict, "Demo Contact Map")

    save = input("\nSave visualization? (y/n): ").strip().lower()
    if save == 'y':
        viz.save("demo_contacts.png")

    show = input("Display visualization? (y/n): ").strip().lower()
    if show == 'y':
        viz.show()


if __name__ == "__main__":
    demo()
