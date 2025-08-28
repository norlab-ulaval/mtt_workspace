#!/bin/bash

sudo chown -R $(whoami) /home/ws/
sudo apt-get update -y
sudo rosdep update
sudo rosdep install --from-paths mtt_ws/src --ignore-src -y

# Need to find a way to install specifically version 3.1 of joy to see if it works in docker
# sudo rosdep install --from-paths mtt_ws/src --ignore-src --skip-keys="joy" -y

# Install lazygit
LAZYGIT_VERSION=$(curl -s "https://api.github.com/repos/jesseduffield/lazygit/releases/latest" | \grep -Po '"tag_name": *"v\K[^"]*')
curl -Lo lazygit.tar.gz "https://github.com/jesseduffield/lazygit/releases/download/v${LAZYGIT_VERSION}/lazygit_${LAZYGIT_VERSION}_Linux_x86_64.tar.gz"
tar xf lazygit.tar.gz lazygit
sudo install lazygit -D -t /usr/local/bin/
rm lazygit*
