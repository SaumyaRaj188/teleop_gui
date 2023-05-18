Git clone inside src folder of the workspace

# To run twist message publisher
1. Install dependensies
    ```
    rosdep install -i --from-paths src --rosdistro foxy -y
    ```
  
2. Build and source using
    ```
    colcon build --symlink-install
    source install/setup.bash
    ```
  
3. run using
    ```
    ros2 run teleop_gui teleop
    ```

# To run PyQt5 controller gui:
1. Create a virtual environment and activate it. [Optional]
    ```
    python -m venv .venv
    .venv\Scripts\Activate.ps1
    ```
    
2. Install the required packages.
    ```
    pip install -r src/teleop_gui/requirements.txt
    ```
    
3. Run the main.py file.
    ```
    python3 src/teleop_gui/gui/main.py
    ```
