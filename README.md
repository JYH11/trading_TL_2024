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
- Ubuntu 22.04 LTS
- C++17 Compiler

## Installation

### ROS2 Installation
Follow the instructions to install ROS2 iron on Ubuntu 20.04 LTS:
[ROS2 iron Installation Guide](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)

### Configuring IntelliSense in VS Code

Ensure your have adequate path, then the configurations may look like this:

```bash
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "/opt/ros/iron/include",
                "${workspaceFolder}/**",
                "${workspaceFolder}/install/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-x64",
            "compilerArgs": []
        }
    ],
    "version": 4
}
```

### System Dependencies
Some additional dependencies may be required for system. Use the following command to install them:

1. For ccapi:
Suggeted clone it on Desktop or you may modify the cmakelisy

```bash
git clone https://github.com/crypto-chassis/ccapi.git
```

Prepare the Build Directory:

```bash
cd /home/harrison/Desktop/ccapi/app
mkdir build
cd build
```

Run CMake and Make:

```bash
cmake ..
make
```
Notice, sometimes the ccapi may change their URL. If you meet problems for this library, please refer to:

`https://github.com/crypto-chassis/ccapi`

2. For Arrow:

Install Required Dependencies:

`https://arrow.apache.org/install/`



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
source install/local_setup.bash
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

`Note: ` Some packages may depend on `interfaces` and sometimes we might need to manually add the installation path to CMAKE_PREFIX_PATH:
```bash
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:Desktop/trading_TL_2024/install
```

If still not work, please `Debugging Further`:
```bash
colcon build --packages-select action --event-handlers console_direct+
```