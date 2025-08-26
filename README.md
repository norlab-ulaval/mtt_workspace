# mtt_project

## Installation

### Prerequistes

- Vscode
- Docker

### Steps

1. Clone the repo

```bash
git clone git@github.com:AlexCampanozzi/mtt_project.git
```

2. Open in vscode

```bash
cd mtt_project
code .
```

3. Make sure you have the "Dev Containers" and "Remote - SSH" extension installed in vscode

![](documentations/img/vscode-extensions.png)

4. Build the container with Ctrl+Shift+P and select "Dev Containers: Rebuild and Reopen in Container". Vscode should build the container and open your vscode frontend inside of it

5. Now you should be inside the container, on the bottom left of the screen, make sure you see this

![](documentations/img/vscode-opened-in-devcontainer.png)

6. You can now try to launch the simulation in a vscode terminal
```bash
ros2 launch mtt_bringup mtt_bringup.launch.py
```

7. If you want to rebuild the workspace, do Ctrl+Shift+B and select "colcon: Colcon Build with compile_commands"
