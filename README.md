# KelRot 5655 | 2026 Season Rebuilt Template

This repository serves as the official **2026 Season Rebuilt Template** for **FRC Team 5655 KelRot**. It is designed to provide a robust, competitive foundation for the upcoming season, integrating modern FRC software libraries and best practices.

Built upon **AdvantageKit** and **Maple-Sim**, this template supports advanced logging, simulation, and reliable swerve drive control.

## Key Features

- **AdvantageKit Integration**: Advanced log-replay functionality for faster debugging and tuning.
- **Physics Simulation**: Integrated **Maple-Sim** for realistic swerve drive physics in simulation.
- **Advanced Control**:
  - **Motion Profiling**: Custom `SCurveProfile` and `ExponentialPID` classes for smooth motion control.
  - **Swerve Drive**: Pre-configured support for REV SparkMax/Flex swerve modules.
- **Utilities**:
  - **LED System**: Comprehensive `Led` subsystem for robot state visualization.
  - **Battery Monitoring**: `BatteryUtils` for tracking voltage sag and power usage.
  - **Tunable Numbers**: `LoggedTunableNumber` for real-time dashboard tuning without code deploys.
- **Code Quality**: Pre-configured `Spotless` formatting and structured Command-Based architecture.

---

## 🚀 Getting Started

### Prerequisites

- Operating System: Windows 10/11, macOS, or Linux.
- **WPILib 2025/2026**: The latest FRC Game Tools and Software.

### WPILib Installation Guide

To develop code for this project, you must interpret the official WPILib development environment. Follow these steps:

#### 1. Download the Installer
- Visit the official [WPILib Releases Page on GitHub](https://github.com/wpilibsuite/allwpilib/releases).
- Download the latest release installer for your operating system (e.g., `WPILib_Windows-2026.x.x.iso`).

#### 2. Run the Installer
- **Windows**: Right-click the downloaded `.iso` file and select **Mount**. Open the mounted drive and run `WPILibInstaller.exe`.
- **macOS/Linux**: Extract the downloaded archive and run the installer script.

#### 3. Installation Steps
1.  **Welcome Screen**: Click **Start**.
2.  **Select Install Type**: Choose **"Install for this User"** (Recommended).
3.  **VS Code Options**:
    - If you don't have VS Code installed, let the installer download and install it.
    - If you strictly want to use an existing install, select it, but the "Included VS Code" is safer for compatibility.
4.  **Confirm**: Click **Install**. The installer will set up the JDK, C++ Compiler, VS Code, and WPILib extensions.
5.  **Finish**: Once complete, you will see a new shortcut on your desktop labeled **"FRC VS Code"**.

**Important**: Always open this project using the **"FRC VS Code"** shortcut, not the standard VS Code shortcut.

---

### Project Setup

1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/KelRot/lib.git
    cd 2026-library-lastversion
    ```

2.  **Open in FRC VS Code**:
    - Launch **FRC VS Code**.
    - Go to `File > Open Folder...` and select the repository folder.

3.  **Build the Code**:
    - Press `Ctrl+Shift+P` (Windows) or `Cmd+Shift+P` (macOS) to open the Command Palette.
    - Type and select **`WPILib: Build Robot Code`**.
    - The first build may take a few minutes as it downloads dependencies (vendor libraries, etc.).

---

## 🎮 Simulation

This template supports high-fidelity simulation using Maple-Sim.

1.  Ensure you have [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) installed.
2.  In FRC VS Code, press `F5` or verify the target is set to **Simulation**.
3.  When the simulation launches, open AdvantageScope.
4.  Connect to the simulator within AdvantageScope to visualize the robot's motion and logs.

---

## 🛠 Maintenance

- **Formatting**: This project uses Spotless to enforce code style. Run the following command to fix formatting issues automatically:
  ```bash
  ./gradlew spotlessApply
  ```

---

**Maintained by FRC Team 5655 KelRot**
