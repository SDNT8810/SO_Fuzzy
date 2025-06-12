# Self-Organizing Deep-Fuzzy Reinforcement Learning for Predictive Path Planning

A novel MATLAB implementation of Self-Organizing Deep-Fuzzy Reinforcement Learning (SARSA DFRL) for autonomous robot navigation and predictive path planning in dynamic environments.

## Overview

This research presents an innovative approach that combines deep fuzzy systems with SARSA reinforcement learning to create an adaptive, interpretable, and computationally efficient navigation framework. The system utilizes a multi-layered deep fuzzy architecture with self-organizing capabilities for optimal path planning in unknown and dynamic environments.

### Key Innovation

- **Deep Fuzzy System (DFS)**: Multi-layered hierarchical fuzzy inference engines for complex interpretable rule-based decision making
- **Self-Organizing Scheme (SOS)**: Automatic fuzzy rule generation and pruning mechanism
- **SARSA Actor-Critic RL**: On-policy reinforcement learning for continuous adaptation
- **Predictive Path Planning**: Forward-looking navigation strategies in dynamic environments
- **TurtleBot3 Integration**: Real-world implementation with differential drive robotics

## Features

### Core Capabilities

- 🧠 **Deep Fuzzy Architecture**: Three-layer hierarchical fuzzy system with convolutional fuzzy processing
- 🔄 **Self-Organizing Rules**: Automatic rule generation, pruning, and adaptation based on experience
- 🎯 **Predictive Planning**: Forward-looking path planning in dynamic environments
- 🤖 **SARSA Actor-Critic**: On-policy reinforcement learning for stable, safe navigation
- 📡 **LiDAR Processing**: Advanced sensor data preprocessing with dimensionality reduction
- ⚡ **Real-time Adaptation**: Dynamic response to changing environmental conditions

### Technical Advantages

- **Low Computational Complexity**: Efficient fuzzy inference with reduced computational cost
- **High Interpretability**: Rule-based decision making with transparent logic
- **Smooth Path Generation**: Continuous, smooth trajectories without oscillations
- **Robust Learning**: Handles uncertainty and incomplete information effectively
- **Scalable Architecture**: Modular design supporting various robot platforms
- **Dynamic Rule Management**: Automatic addition and removal of fuzzy rules

## Deep Fuzzy System Architecture

The system implements a novel three-layer deep fuzzy architecture:

```text
┌──────────────────────────────────────────────────────────────┐
│                    Deep Fuzzy System (DFS)                   │
├──────────────────────────────────────────────────────────────┤
│  Layer 1: Convolutional Fuzzy Layer                          │
│  ┌─────────────────┐    ┌─────────────────┐                  │
│  │   LiDAR 360x1   │───▶│  Feature Extrac.│───▶ 13x1 Output  │
│  │   Raw Data      │    │  Fuzzy Conv.    │                  │
│  └─────────────────┘    └─────────────────┘                  │
├──────────────────────────────────────────────────────────────┤
│  Layer 2: Contextual Feature Integration                     │
│  ┌─────────────────┐    ┌─────────────────┐                  │
│  │   Node 2:       │    │   Node 3:       │                  │
│  │   13x1 + Goal   │    │   13x1 + Robot  │───▶ 14x2 Output  │
│  │   Direction     │    │   Heading       │                  │
│  └─────────────────┘    └─────────────────┘                  │
├──────────────────────────────────────────────────────────────┤
│  Layer 3: Decision Making                                    │
│  ┌────────────────────────────────────────┐                  │
│  │  Fuzzy Inference Engine                │                  │
│  │  ┌──────────────┐    ┌─────────────┐   │                  │
│  │  │  Linear Vel. │    │ Angular Vel.│   │───▶ 2x1 Output   │
│  │  │     (v)      │    │     (ω)     │   │                  │
│  │  └──────────────┘    └─────────────┘   │                  │
│  └────────────────────────────────────────┘                  │
└──────────────────────────────────────────────────────────────┘
            │                                    ▲
            ▼                                    │
   ┌─────────────────┐                  ┌─────────────────┐
   │   TurtleBot3    │                  │   SARSA Actor-  │
   │   Robot Model   │                  │   Critic RL     │
   │                 │                  │                 │
   │  Kinematic      │                  │  Self-Organizing│
   │  Equations      │                  │  Scheme (SOS)   │
   └─────────────────┘                  └─────────────────┘
```

### Self-Organizing Scheme (SOS)

The system features an intelligent rule management mechanism:

- **Rule Addition**: Automatically generates new fuzzy rules when existing rules provide insufficient coverage (activation < γ threshold)
- **Rule Pruning**: Removes rules based on:
  - **Age-based**: Rules unused for >250 iterations
  - **Similarity-based**: Rules with similar input means (Euclidean distance < 0.2)
  - **Cost-based**: Rules with poor performance metrics

## SARSA Actor-Critic Reinforcement Learning

### Mathematical Foundation

The system implements SARSA (State-Action-Reward-State-Action) with fuzzy critic estimator:

#### State-Action Value Function
```
Q^π(s,a) = E[r_{t+1} + γQ^π(s',a') | s,a]
```

#### SARSA Update Rule
```
Q(s,a) ← Q(s,a) + α(r + γQ(s',a') - Q(s,a))
```

#### Fuzzy Critic Estimator (FCE)
```
Q̂(I_critic) = Σ(w_l * z_l) / Σ(z_l)
```

Where:
- `I_critic`: Combined state-action input vector
- `w_l`: Weight of l-th fuzzy rule
- `z_l`: Antecedent firing strength
- `μ_critic`: Gaussian membership functions

### Key Advantages of SARSA

- **On-Policy Learning**: Learns from actual policy execution
- **Safe Exploration**: Reduces risky actions during learning
- **Stability**: More stable convergence in dynamic environments
- **Real-time Adaptability**: Continuous policy refinement

## Installation

### Prerequisites

- MATLAB R2019b or later
- Required MATLAB Toolboxes:
  - Fuzzy Logic Toolbox
  - Deep Learning Toolbox
  - Robotics System Toolbox
  - Reinforcement Learning Toolbox

### Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd SO_Fuzzy
   ```

2. Add the project to MATLAB path:
   ```matlab
   addpath(genpath('SO_Fuzzy'))
   ```

3. Load pre-trained models:
   ```matlab
   load memory.mat
   load FuzzySystemLongMemory.mat
   ```

## Usage

### Basic Simulation

Run the main deep fuzzy system:
```matlab
main
```

### Configuration Parameters

Key parameters in `functions/init_params.m`:

- **Time Parameters**:
  - `T_s`: Time step (default: 0.02s)
  - `T_f`: Final simulation time (default: 100s)
  - `Window_Size`: Learning window size (default: 10)

- **Fuzzy System Parameters**:
  - `gamma`: Minimum activation threshold (default: 0.3)
  - `Nmax`: Maximum number of rules (default: 50)
  - `sigma0`: Initial membership function variance (default: 1.2)
  - `Tage`: Maximum rule age for pruning (default: 250)
  - `Tsim`: Similarity threshold for rule merging (default: 0.2)

- **Robot Parameters**:
  - `V`: Linear velocity (m/s)
  - `Omega`: Angular velocity (rad/s)
  - Robot kinematic model: `[x, y, θ]`

- **SARSA Parameters**:
  - `α`: Learning rate
  - `γ`: Discount factor
  - Experience replay buffer management

## TurtleBot3 Robot Model

The system is designed for differential drive robots following the kinematic model:

```matlab
% State representation
X = [x, y, θ]

% Kinematic equations
ẋ = v * cos(θ)
ẏ = v * sin(θ)
θ̇ = ω

% Wheel speed conversion
[φ̇_R; φ̇_L] = (d/2r) * [1, d/2r; 1, -d/2r] * [v; ω]
```

Where:
- `(x,y)`: Robot position
- `θ`: Robot orientation
- `v`: Linear velocity
- `ω`: Angular velocity
- `r`: Wheel radius
- `d`: Wheelbase distance

## File Structure

```
SO_Fuzzy/
├── main.m                          # Main simulation loop
├── memory.mat                      # Pre-trained system memory
├── FuzzySystemLongMemory.mat       # Saved fuzzy system state
├── functions/                      # Core algorithm implementations
│   ├── init_params.m              # Parameter initialization
│   ├── ManageFuzzySys.m           # Deep fuzzy system management
│   ├── learning_proccess.m        # SARSA reinforcement learning
│   ├── kinodynamics.m             # TurtleBot3 kinematic model
│   ├── update_environment_parameters.m  # Environment sensing
│   ├── update_presentation.m      # Real-time visualization
│   ├── Lidar2Fuzzy.m             # LiDAR preprocessing
│   ├── Antc.m                     # Antecedent calculation
│   └── Q.m                        # Q-function estimation
├── maps/                          # Environment maps and scenarios
├── plotters/                      # Visualization and analysis scripts
│   ├── plot_fuzzy_node_*.m       # Fuzzy layer visualizations
│   ├── Layer_*.m                 # Architecture diagrams
│   └── animation.m               # Path animation
└── SDNT_JP1.md                   # Research paper
```

## Algorithm Details

### Deep Fuzzy System Layers

#### Layer 1: Convolutional Fuzzy Layer
- **Input**: 360×1 LiDAR readings
- **Processing**: Fuzzy convolutional operations for feature extraction
- **Output**: 13×1 compressed environmental representation
- **Function**: Spatial feature extraction and dimensionality reduction

#### Layer 2: Contextual Feature Integration
- **Node 2**: Combines 13×1 vector with goal direction
- **Node 3**: Combines 13×1 vector with robot heading
- **Processing**: Contextual information fusion
- **Output**: Enhanced state representation for decision making

#### Layer 3: Decision Making
- **Input**: Combined outputs from Layer 2 nodes
- **Processing**: Fuzzy inference engine
- **Output**: Control commands [v, ω]
- **Function**: Generate smooth, optimal control actions

### Self-Organizing Learning Process

1. **Experience Collection**: Gather state-action-reward trajectories
2. **Rule Evaluation**: Assess existing fuzzy rules coverage
3. **Rule Addition**: Create new rules for novel situations
4. **Rule Pruning**: Remove outdated, similar, or low-performing rules
5. **Parameter Update**: Adjust membership functions and weights
6. **Policy Refinement**: Update SARSA actor-critic parameters

## Performance Advantages

### Compared to Traditional Methods

- **vs. Q-Learning**: Better stability and safer exploration
- **vs. Deep Q-Learning**: Lower computational complexity, higher interpretability
- **vs. A*/Dijkstra**: Dynamic adaptability and real-time responsiveness
- **vs. Potential Fields**: No local minima, smoother paths

### Key Performance Metrics

- **Path Efficiency**: Optimal distance-to-goal ratio
- **Obstacle Avoidance**: Zero collision rate in testing
- **Learning Speed**: Fast convergence with stable performance
- **Computational Cost**: Real-time processing capability
- **Adaptability**: Dynamic response to environmental changes
- **Interpretability**: Clear fuzzy rule-based decision logic

## Applications

### Primary Applications

- **Service Robot Navigation**: Indoor autonomous navigation
- **Autonomous Transportation**: Self-driving vehicle path planning
- **Warehouse Automation**: Mobile robot logistics
- **Search and Rescue**: Dynamic environment navigation
- **Agricultural Robotics**: Field navigation and obstacle avoidance

### Environmental Scenarios

- **Dynamic Obstacles**: Moving pedestrians, vehicles, objects
- **Unknown Environments**: Unmapped or partially mapped spaces
- **Complex Geometries**: Narrow corridors, cluttered spaces
- **Multi-objective Navigation**: Goal-reaching with constraints

## Research Contributions

### Novel Methodological Contributions

1. **Deep Fuzzy System Framework**: Hierarchical fuzzy inference with interpretable deep structure
2. **Self-Organizing Scheme**: Automatic rule generation and management
3. **SARSA-Fuzzy Integration**: On-policy RL with fuzzy state-action value estimation
4. **Predictive Path Planning**: Forward-looking navigation in dynamic environments

### Technical Innovations

- **Convolutional Fuzzy Processing**: Novel approach to LiDAR data processing
- **Contextual Feature Integration**: Multi-modal sensor fusion architecture
- **Dynamic Rule Management**: Intelligent pruning and addition mechanisms
- **Smooth Control Generation**: Continuous, oscillation-free path execution

## Future Enhancements

### Planned Developments

- **Multi-Agent Systems**: Coordination between multiple robots
- **Real-World Deployment**: Hardware implementation and testing
- **Advanced Sensor Integration**: Vision, IMU, GPS sensor fusion
- **Cloud-Based Learning**: Distributed training and knowledge sharing

### Research Directions

- **Transfer Learning**: Knowledge transfer between different environments
- **Explainable AI**: Enhanced interpretability of fuzzy decisions
- **Adaptive Architecture**: Dynamic adjustment of network structure
- **Safety Verification**: Formal verification of navigation safety

## Citation

If you use this work in your research, please cite:

```bibtex
@article{jalaeian2025deep,
  title={Self-Organizing Deep-Fuzzy Reinforcement Learning for Predictive Path Planning and Navigation in Dynamic Environment},
  author={Jalaeian-F, Mohsen and Nikkhouy, Davoud and Alirezaee, Shahpour and Mozaffari, Saeed and Rastegarmoghaddam, Mahshad and Samadzadeh, Shima},
  journal={[Journal Name]},
  year={2025},
  publisher={[Publisher]}
}
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contact

For questions, collaboration, or technical support:

- **Lead Author**: Mohsen Jalaeian-F (mohsen.jalaeian@polimi.it)
- **Corresponding Author**: Davoud Nikkhouy (davoudnikkhouy@gmail.com)
- **Institution**: Politecnico di Milano, University of Windsor

## Acknowledgments

This research was conducted at:
- Department of Electronic, Information and Bioengineering (DEIB), Politecnico di Milano, Italy
- Mechanical, Automotive and Materials Engineering (MAME), University of Windsor, Canada

---

**Note**: This implementation represents cutting-edge research in autonomous navigation. The system has been extensively tested in simulation environments and shows superior performance compared to traditional methods. For real-world deployment, appropriate safety measures and calibration procedures should be implemented.
