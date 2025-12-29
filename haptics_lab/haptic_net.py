import torch
import torch.nn as nn
import torch.nn.functional as F

class HapticNetV0(nn.Module):
    def __init__(self, input_dim=9, output_dim=5):
        super(HapticNetV0, self).__init__()
        # A simple 3-layer MLP (Multi-Layer Perceptron)
        # Input -> Hidden (32) -> Hidden (32) -> Output
        # We keep it tiny (32 neurons) for <100 microsecond speed.
        
        self.fc1 = nn.Linear(input_dim, 32)
        self.fc2 = nn.Linear(32, 32)
        self.fc3 = nn.Linear(32, output_dim)

    def forward(self, x):
        # Layer 1: ReLU activation (Standard)
        x = F.relu(self.fc1(x))
        
        # Layer 2: ReLU activation
        x = F.relu(self.fc2(x))
        
        # Layer 3: Sigmoid to force output between 0.0 and 1.0
        # (Since haptic amplitude can't be negative or > 100%)
        x = torch.sigmoid(self.fc3(x))
        return x

# Quick Test
if __name__ == "__main__":
    model = HapticNetV0()
    dummy_input = torch.randn(1, 9) # 1 sample, 9 features
    output = model(dummy_input)
    print("Model Output:", output)
    print("Model is ready for training.")