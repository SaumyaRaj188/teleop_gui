# teleop_gui (Only given Linear velocity for now)
install dependensies - rosdep install -i --from-paths src --rosdistro foxy -y
  
build in src folder of workspace using - colcon build --symlink-install
  
run using - ros2 run teleop_gui teleop

# To run PyQt5 controller gui:
1. Create a virtual environment and activate it. [Optional]
    ```
    python -m venv .venv
    .venv\Scripts\Activate.ps1
    ```
2. Install the required packages.
    ```
    pip install -r requirements.txt
    ```
3. Run the main.py file.
    ```
    python teleop_gui/main.py
    ```