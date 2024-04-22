# Trading_TL_2024
## TRADING: FINANCIAL MARKET & ALGORITHMIC TRADING
### TECHLAUNCHER 2024 

---

## Project Introduction
The primary objective of this project is to design and implement an algorithmic trading engine that leverages advanced communication protocols and integrates various subsystems. This engine aims to simulate and execute trading strategies efficiently, providing a practical platform for applying algorithmic trading principles in real-world financial markets.

The initial phase focuses on:
- Designing and implementing a trading engine.
- Interfacing with multiple brokers and exchange APIs to collect real-time market data.
- Storing data efficiently for both backtesting and live trading.
- Testing trading strategies using historical data and simulated market conditions.
- Executing trades based on signals from an ensemble of trading strategies.

---

## Prerequisites

Ensure you have the following installed:
- ROS2 iron
- Ubuntu 20.04 LTS
- C++17 Compiler

## Installation

### ROS2 Installation
Follow the instructions to install ROS2 iron on Ubuntu 20.04 LTS:
[ROS2 iron Installation Guide](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)

### System Dependencies
Some additional dependencies may be required for system. Use the following command to install them:

```bash
sudo apt-get update && sudo apt-get install -y <list-of-dependencies>

```

## How to run

`1: `Before running, we need to compile. This command will compile all packages:

```bash
colcon build

```

If you wanna compile specific packages, use:

```bash
colcon build --packages-select <package_name>
```

This is an example and just remember, `action` relies on interfaces. So before `action`, you must have already compiled `interfaces`:

```bash
colcon build --pakcages-select action
```

`2: `Then we need to configure environment variables:

```bash
source install/setup.bash
```

If it doesn't work, before this command, run this:

```bash
source /opt/ros/iron/setup.bash
```

`Plus: `Every time you open a new terminal, you have to run this command:

```bash
source install/setup.bash
```

If you don't want to do so, another way is trying to edit `.bashrc file`:

`First`, run this command in termimal:

```bash
nano ~/.bashrc
```

`Second`, Add the following lines at the end of the file:

```bash
source /opt/ros/iron/setup.bash
```

```bash
source ~/Desktop/ros2_basic/install/local_setup.bash
```

Keep changes then return to terminal

In order for these changes to take effect, you have two options:

`Effective immediately`: Run the following command in the current terminal to apply the changes immediately: 

```bash
source ~/.bashrc
```

`Reopen the terminal`: Close the current terminal and reopen a new terminal. Changes in .bashrc will be applied automatically.


`3: `Now we can run:

```bash
ros2 run <package_name> <node_name>
```

Like:

```bash
ros2 run action action_server
```