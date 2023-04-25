
#!/bin/bash
sudo apt install tmuxinator -y
DIR=~/.config/tmuxinator
FILES=~/workspace/tiny_quadruped_ws/src/tiny_quadruped/startup_scripts/tmuxinator/*

if [ -d "$DIR" ];
then
    sudo rm ~/.config/tmuxinator/tq*
    echo "$DIR directory exists."
else
    echo "$DIR directory does not exist, creating new directory"
    mkdir "$DIR"
fi
for f in "$FILES"
do
    ln -s $f ~/.config/tmuxinator/
done