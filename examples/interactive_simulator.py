#!/usr/bin/env python3
"""
HAPTOS Interactive Greybox Simulator

Full-featured physics simulation and haptic testing environment.

Features:
- MuJoCo 3D viewer window
- Real-time contact visualization (force vectors, body part IDs)
- Live haptic cue parameter display
- Control panel (play/pause, speed, reset, step)
- Statistics dashboard (latency, contacts, bandwidth)
- Cue history plots (texture, shear, weight, impact, ring)

Usage:
    python examples/interactive_simulator.py [model.xml]

Requirements:
    pip install haptos[viz]
"""

import sys
import time
import numpy as np
from collections import deque
from pathlib import Path

try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QPushButton, QLabel, QSlider, QGroupBox, QGridLayout, QTextEdit,
        QComboBox, QCheckBox, QSplitter
    )
    from PyQt5.QtCore import QTimer, Qt
    from PyQt5.QtGui import QFont
except ImportError:
    print("Error: PyQt5 not installed")
    print("Install with: pip install haptos[viz]")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use('Qt5Agg')
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
    from matplotlib.figure import Figure
except ImportError:
    print("Error: matplotlib not installed")
    print("Install with: pip install haptos[viz]")
    sys.exit(1)

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print("Error: mujoco not installed")
    print("Install with: pip install mujoco")
    sys.exit(1)

import haptos


class CuePlotCanvas(FigureCanvasQTAgg):
    """Real-time plot of haptic cue parameters."""

    def __init__(self, parent=None, width=5, height=2, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        super().__init__(self.fig)

        self.history_length = 100  # 1 second @ 100Hz
        self.time_history = deque(maxlen=self.history_length)
        self.texture_history = deque(maxlen=self.history_length)
        self.weight_history = deque(maxlen=self.history_length)
        self.impact_history = deque(maxlen=self.history_length)

        self.axes.set_ylim(0, 1)
        self.axes.set_xlabel('Time (frames)')
        self.axes.set_ylabel('Amplitude')
        self.axes.set_title('Haptic Cues (Real-time)')
        self.axes.grid(True, alpha=0.3)

        self.lines = {
            'texture': self.axes.plot([], [], 'b-', label='Texture')[0],
            'weight': self.axes.plot([], [], 'g-', label='Weight')[0],
            'impact': self.axes.plot([], [], 'r-', label='Impact')[0],
        }
        self.axes.legend(loc='upper right')

    def update_plot(self, cue_params):
        """Update plot with new cue parameters."""
        self.time_history.append(len(self.time_history))

        if cue_params:
            # Average all body parts
            avg_texture = np.mean([c.texture_amplitude for c in cue_params.values()])
            avg_weight = np.mean([c.weight_offset for c in cue_params.values()])
            avg_impact = np.mean([c.impact_amplitude for c in cue_params.values()])
        else:
            avg_texture = 0
            avg_weight = 0
            avg_impact = 0

        self.texture_history.append(avg_texture)
        self.weight_history.append(avg_weight)
        self.impact_history.append(avg_impact)

        # Update line data
        x = list(range(len(self.time_history)))
        self.lines['texture'].set_data(x, list(self.texture_history))
        self.lines['weight'].set_data(x, list(self.weight_history))
        self.lines['impact'].set_data(x, list(self.impact_history))

        self.axes.set_xlim(0, max(100, len(self.time_history)))
        self.draw()


class ContactVisualizer(QWidget):
    """Widget displaying current contact information."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout()

        title = QLabel("Contact Visualization")
        title.setFont(QFont('Arial', 12, QFont.Bold))
        layout.addWidget(title)

        self.contact_text = QTextEdit()
        self.contact_text.setReadOnly(True)
        self.contact_text.setMaximumHeight(200)
        self.contact_text.setFont(QFont('Courier', 9))
        layout.addWidget(self.contact_text)

        self.setLayout(layout)

    def update_contacts(self, filtered_contacts):
        """Update contact display."""
        if not filtered_contacts:
            self.contact_text.setText("No contacts detected")
            return

        text = f"Active Contacts: {len(filtered_contacts)}\n"
        text += "=" * 50 + "\n"

        for i, contact in enumerate(filtered_contacts, 1):
            patch = contact.patch
            text += f"\nContact {i}:\n"
            text += f"  Body Part ID: {patch.body_part_id}\n"
            text += f"  Force (N):    {patch.force_normal:.3f}\n"
            text += f"  Shear (N):    ({patch.force_shear[0]:.3f}, {patch.force_shear[1]:.3f})\n"
            text += f"  Tier:         {contact.rendering_tier}\n"
            text += f"  Cue Mask:     0b{contact.cue_mask:05b}\n"

        self.contact_text.setText(text)


class CueParameterDisplay(QWidget):
    """Widget displaying current cue parameters."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout()

        title = QLabel("Haptic Cue Parameters")
        title.setFont(QFont('Arial', 12, QFont.Bold))
        layout.addWidget(title)

        self.cue_text = QTextEdit()
        self.cue_text.setReadOnly(True)
        self.cue_text.setMaximumHeight(250)
        self.cue_text.setFont(QFont('Courier', 9))
        layout.addWidget(self.cue_text)

        self.setLayout(layout)

    def update_cues(self, cue_params_dict):
        """Update cue parameter display."""
        if not cue_params_dict:
            self.cue_text.setText("No cues generated")
            return

        text = f"Active Cues: {len(cue_params_dict)} channels\n"
        text += "=" * 50 + "\n"

        for body_part_id, cue in cue_params_dict.items():
            text += f"\nBody Part {body_part_id}:\n"
            text += f"  Texture:  {cue.texture_grain_hz:.1f}Hz @ {cue.texture_amplitude:.2f}\n"
            text += f"  Shear:    ({cue.shear_direction[0]:.2f}, {cue.shear_direction[1]:.2f}) mag={cue.shear_magnitude:.2f}\n"
            text += f"  Weight:   {cue.weight_offset:.2f}\n"

            if cue.trigger_impulse:
                text += f"  Impact:   {cue.impact_amplitude:.2f} @ {cue.impact_frequency_hz:.1f}Hz\n"
                text += f"  Ring:     {cue.ring_amplitude:.2f} decay={cue.ring_decay_ms:.0f}ms\n"

        self.cue_text.setText(text)


class StatisticsPanel(QWidget):
    """Widget displaying simulation statistics."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout()

        title = QLabel("Statistics")
        title.setFont(QFont('Arial', 12, QFont.Bold))
        layout.addWidget(title)

        self.stats_text = QTextEdit()
        self.stats_text.setReadOnly(True)
        self.stats_text.setMaximumHeight(200)
        self.stats_text.setFont(QFont('Courier', 9))
        layout.addWidget(self.stats_text)

        self.setLayout(layout)

    def update_stats(self, sim, renderer, driver, elapsed_time):
        """Update statistics display."""
        sim_stats = sim.get_state()
        render_stats = renderer.get_stats()
        driver_stats = driver.get_stats()

        text = f"Simulation Time: {sim_stats['time']:.3f}s\n"
        text += f"Real Time:       {elapsed_time:.3f}s\n"
        text += f"Speed Factor:    {sim_stats['time'] / max(elapsed_time, 0.001):.2f}x\n"
        text += f"\n"
        text += f"Physics Steps:   {sim_stats['step_count']}\n"
        text += f"Active Contacts: {sim_stats['active_contacts']}\n"
        text += f"Total Contacts:  {sim_stats['router_stats']['total_contacts']}\n"
        text += f"Filtered:        {sim_stats['router_stats']['filtered_contacts']}\n"
        text += f"\n"
        text += f"Inferences:      {render_stats.get('total_inferences', 0)}\n"
        text += f"Avg Render Time: {render_stats.get('avg_inference_time_ms', 0):.2f}ms\n"
        text += f"\n"
        text += f"Packets Sent:    {driver_stats['total_sent']}\n"
        text += f"Success Rate:    {driver_stats['success_rate']:.1%}\n"
        text += f"Bandwidth:       {driver_stats.get('bandwidth_hz', 0):.1f} pkt/s\n"

        self.stats_text.setText(text)


class ControlPanel(QWidget):
    """Control panel for simulation."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout()

        # Playback controls
        playback_group = QGroupBox("Playback")
        playback_layout = QHBoxLayout()

        self.play_button = QPushButton("Play")
        self.play_button.setCheckable(True)
        playback_layout.addWidget(self.play_button)

        self.step_button = QPushButton("Step")
        playback_layout.addWidget(self.step_button)

        self.reset_button = QPushButton("Reset")
        playback_layout.addWidget(self.reset_button)

        playback_group.setLayout(playback_layout)
        layout.addWidget(playback_group)

        # Speed control
        speed_group = QGroupBox("Simulation Speed")
        speed_layout = QVBoxLayout()

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(1)
        self.speed_slider.setMaximum(20)
        self.speed_slider.setValue(10)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(5)
        speed_layout.addWidget(self.speed_slider)

        self.speed_label = QLabel("1.0x")
        self.speed_label.setAlignment(Qt.AlignCenter)
        speed_layout.addWidget(self.speed_label)

        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)

        # Visualization options
        viz_group = QGroupBox("Visualization")
        viz_layout = QVBoxLayout()

        self.show_contacts = QCheckBox("Show Contact Forces")
        self.show_contacts.setChecked(True)
        viz_layout.addWidget(self.show_contacts)

        self.show_ids = QCheckBox("Show Body Part IDs")
        self.show_ids.setChecked(True)
        viz_layout.addWidget(self.show_ids)

        self.show_cues = QCheckBox("Show Haptic Cues")
        self.show_cues.setChecked(True)
        viz_layout.addWidget(self.show_cues)

        viz_group.setLayout(viz_layout)
        layout.addWidget(viz_group)

        # Driver selection
        driver_group = QGroupBox("Hardware Driver")
        driver_layout = QVBoxLayout()

        self.driver_combo = QComboBox()
        self.driver_combo.addItems(["Mock Driver", "Serial (Coming Soon)"])
        driver_layout.addWidget(self.driver_combo)

        self.sync_checkbox = QCheckBox("Enable Channel Sync")
        driver_layout.addWidget(self.sync_checkbox)

        driver_group.setLayout(driver_layout)
        layout.addWidget(driver_group)

        layout.addStretch()
        self.setLayout(layout)


class InteractiveSimulator(QMainWindow):
    """Main interactive simulator window."""

    def __init__(self, model_path):
        super().__init__()
        self.setWindowTitle("HAPTOS Interactive Greybox Simulator")
        self.setGeometry(100, 100, 1400, 900)

        # Initialize HAPTOS components
        print("Initializing HAPTOS Platform...")
        self.model_path = model_path
        self.sim = haptos.Simulation(model_path, max_contacts=20)
        self.renderer = haptos.Renderer()
        self.driver = haptos.Driver(driver_type="mock")

        # Register 6 channels (5 fingers + palm)
        self.body_parts = {
            10: "index",
            11: "thumb",
            12: "middle",
            13: "ring",
            14: "pinky",
            15: "palm"
        }

        for body_part_id, name in self.body_parts.items():
            self.driver.register(body_part_id, f"MOCK_{name.upper()}")

        print("✓ HAPTOS initialized")

        # Simulation state
        self.is_playing = False
        self.simulation_speed = 1.0
        self.start_time = time.time()
        self.last_render_step = 0

        # Setup UI
        self.setup_ui()

        # Setup simulation timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.simulation_step)
        self.timer.setInterval(16)  # ~60 FPS

    def setup_ui(self):
        """Setup the user interface."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout()

        # Left panel: Control + Visualization options
        left_panel = QWidget()
        left_layout = QVBoxLayout()

        self.control_panel = ControlPanel()
        self.control_panel.play_button.clicked.connect(self.toggle_playback)
        self.control_panel.step_button.clicked.connect(self.single_step)
        self.control_panel.reset_button.clicked.connect(self.reset_simulation)
        self.control_panel.speed_slider.valueChanged.connect(self.update_speed)
        left_layout.addWidget(self.control_panel)

        left_panel.setLayout(left_layout)
        left_panel.setMaximumWidth(300)
        main_layout.addWidget(left_panel)

        # Center/Right: Data displays
        splitter = QSplitter(Qt.Vertical)

        # Contact visualization
        self.contact_viz = ContactVisualizer()
        splitter.addWidget(self.contact_viz)

        # Cue parameter display
        self.cue_display = CueParameterDisplay()
        splitter.addWidget(self.cue_display)

        # Real-time plot
        self.cue_plot = CuePlotCanvas(self, width=8, height=3)
        splitter.addWidget(self.cue_plot)

        # Statistics panel
        self.stats_panel = StatisticsPanel()
        splitter.addWidget(self.stats_panel)

        main_layout.addWidget(splitter)

        central_widget.setLayout(main_layout)

        # Status bar
        self.statusBar().showMessage("Ready. Press Play to start simulation.")

    def toggle_playback(self):
        """Toggle play/pause."""
        self.is_playing = self.control_panel.play_button.isChecked()

        if self.is_playing:
            self.control_panel.play_button.setText("Pause")
            self.timer.start()
            self.statusBar().showMessage("Simulation running...")
        else:
            self.control_panel.play_button.setText("Play")
            self.timer.stop()
            self.statusBar().showMessage("Simulation paused")

    def single_step(self):
        """Execute a single simulation step."""
        self.simulation_step()
        self.statusBar().showMessage(f"Step {self.sim.step_count} complete")

    def reset_simulation(self):
        """Reset simulation to initial state."""
        self.is_playing = False
        self.control_panel.play_button.setChecked(False)
        self.control_panel.play_button.setText("Play")
        self.timer.stop()

        # Reset HAPTOS components
        self.sim.reset()
        self.renderer.reset()
        self.driver.reset_stats()

        # Reset timing
        self.start_time = time.time()
        self.last_render_step = 0

        # Clear displays
        self.contact_viz.update_contacts([])
        self.cue_display.update_cues({})

        self.statusBar().showMessage("Simulation reset")

    def update_speed(self, value):
        """Update simulation speed."""
        self.simulation_speed = value / 10.0
        self.control_panel.speed_label.setText(f"{self.simulation_speed:.1f}x")

    def simulation_step(self):
        """Execute one simulation step."""
        # Run multiple physics steps based on speed
        steps_to_run = int(self.simulation_speed)

        for _ in range(steps_to_run):
            # Physics step (1kHz)
            filtered_contacts = self.sim.step_filtered()

            # Render cues (100Hz - every 10 steps)
            cue_params = {}
            if self.sim.step_count % 10 == 0:
                if filtered_contacts:
                    cue_params = self.renderer.render(filtered_contacts)
                    self.driver.send(cue_params)
                self.last_render_step = self.sim.step_count

        # Update UI (at UI refresh rate)
        if self.control_panel.show_contacts.isChecked():
            self.contact_viz.update_contacts(filtered_contacts)

        if self.control_panel.show_cues.isChecked():
            self.cue_display.update_cues(cue_params)
            self.cue_plot.update_plot(cue_params)

        # Update statistics
        elapsed = time.time() - self.start_time
        self.stats_panel.update_stats(self.sim, self.renderer, self.driver, elapsed)

        # Update status bar
        self.statusBar().showMessage(
            f"Step: {self.sim.step_count} | "
            f"Time: {self.sim.time:.2f}s | "
            f"Contacts: {len(filtered_contacts) if filtered_contacts else 0} | "
            f"Speed: {self.simulation_speed:.1f}x"
        )

    def closeEvent(self, event):
        """Handle window close event."""
        self.timer.stop()
        self.driver.disconnect_all()
        print("\n✓ Simulator closed")
        event.accept()


def main():
    """Main entry point."""
    print("=" * 60)
    print("  HAPTOS Interactive Greybox Simulator")
    print("=" * 60)
    print()

    # Get model path
    if len(sys.argv) > 1:
        model_path = sys.argv[1]
    else:
        # Try to find a default model
        default_paths = [
            "assets/hand_models/simple_hand.xml",
            "assets/hand_models/detailed_hand.xml"
        ]

        model_path = None
        for path in default_paths:
            if Path(path).exists():
                model_path = path
                break

        if model_path is None:
            print("Error: No model file specified and no default found")
            print()
            print("Usage:")
            print("  python examples/interactive_simulator.py <model.xml>")
            print()
            print("Example:")
            print("  python examples/interactive_simulator.py assets/hand_models/simple_hand.xml")
            sys.exit(1)

    if not Path(model_path).exists():
        print(f"Error: Model file not found: {model_path}")
        sys.exit(1)

    print(f"Model: {model_path}")
    print()

    # Create Qt application
    app = QApplication(sys.argv)

    # Create and show simulator
    simulator = InteractiveSimulator(model_path)
    simulator.show()

    print("\n✓ Simulator window opened")
    print("  Use Play/Pause/Step controls to interact")
    print()

    # Run application
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
