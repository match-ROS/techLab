# techLab
Code für das Techlab


# PC einrichten
1. Ros ONE installieren

    # Configure ROS One apt repository
    ```
    sudo apt install curl
    sudo curl -sSL https://ros.packages.techfak.net/gpg.key -o /etc/apt/keyrings/ros-one-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros1.list
    echo "# deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs) main-dbg" | sudo tee -a /etc/apt/sources.list.d/ros1.list
    ```

    # Install and setup rosdep
    # Do not install python3-rosdep2, which is an outdated version of rosdep shipped via the Ubuntu repositories (instead of ROS)!
    ```
    sudo apt update
    sudo apt install python3-rosdep
    sudo rosdep init
    ```

    # Define custom rosdep package mapping
    ```
    echo "yaml https://ros.packages.techfak.net/ros-one.yaml one" | sudo tee /etc/ros/rosdep/sources.list.d/1-ros-one.list
    rosdep update
    ```

    # Install packages, e.g. ROS desktop
    ```
    sudo apt install ros-one-desktop
    ```

2. PC Einstellungen für Dobot

    ```
    sudoedit /etc/udev/rules.d/50-myusb.rules
    ```

    ```
    KERNEL=="ttyUSB[0-9]*",MODE="0666"
    KERNEL=="ttyACM[0-9]*",MODE="0666"
    ```

    ```
    sudo nano .bashrc
    ```
    add this to .bashrc 
    ```
    source /opt/ros/one/setup.bash
    source /home/rosmatch/catkin_ws/devel/setup.bash
    ```


3. Repo clonen
    ```
    mkdir catkin_ws
    cd catkin_ws/
    mkdir src
    cd src/
    sudo apt install git
    sudo apt-get install python3-catkin-tools
    sudo apt install ros-one-joy
    sudo apt update
    sudo apt install libqt5serialport5
    git clone https://github.com/match-ROS/techLab
    cd ..
    catkin build
    ```

4. Autostart einrichten
    ```
    cd 
    mkdir bin
    cd bin
    nano start_dobot_teleop.sh
    ```
    code aus start_dobot_teleop.sh (oder ganze Datei) kopieren. Datei speichern und
    ```
    chmod +x start_dobot_teleop.sh 
    ```

    dann 
    ```
    nano ~/.config/systemd/user/dobot-teleop.service
    ```
    code auf dobot-teleop.service kopieren.
    speichern und ausführen:.
     ```
    systemctl --user daemon-reload
    systemctl --user enable --now dobot-teleop.service
    ```