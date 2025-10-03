# mtt_project

## Installation

Tested on:
- Ubuntu 24
- Ubuntu 22

### Prerequistes

- Vscode
- Docker

#### Docker
Once docker in installed, verify that it is well configured

1. Check if Docker works as root
sudo docker ps

2. Add your user to the docker group

Run:
```
sudo groupadd docker  # (only if the group doesn't exist yet)
sudo usermod -aG docker $USER
```
3. Apply the group membership

Either log out and log back in, or run:
```
newgrp docker
```
This updates your current session with the new group.

4. Test again by running:
```
docker ps
```

#### Git
The docker requires a .gitconfig in the home directory. The file can be left empty or with the git user informations. If the github has not been configured on the current system, run:
```
touch ~/.gitconfig
```
or 
```
git config --global user.email "youremail"
```

#### ROS
By default the docker does not set a ROS_DOMAIN_ID value to prevent collision on multiple system on the same network, in the computer's .bashrc add the following line where X is the chosen domain id (ROS is not required on the system):
```
export ROS_DOMAIN_ID=X
```
### Repo installation

1. Clone the repo

```bash
git clone git@github.com:AlexCampanozzi/mtt_project.git
```

2. Enable xhost to properly forward GUI
```bash
xhost +local:docker
```

3. Open in vscode

```bash
cd mtt_project
code .
```

4. Make sure you have the "Dev Containers" and "Remote - SSH" extension installed in vscode

![](documentations/img/vscode-extensions.png)

5. Build the container with Ctrl+Shift+P and select "Dev Containers: Rebuild and Reopen in Container". Vscode should build the container and open your vscode frontend inside of it

6. Now you should be inside the container, on the bottom left of the screen, make sure you see this

![](documentations/img/vscode-opened-in-devcontainer.png)

7. You can now try to launch the simulation in a vscode terminal
```bash
ros2 launch mtt_bringup mtt_simulation.launch.py
```

8. If you want to rebuild the workspace, do Ctrl+Shift+B and select "colcon: Colcon Build with compile_commands"
