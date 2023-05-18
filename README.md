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
    - Create a virtual environment
        ```
        pip install virtualenv
        virtualenv .venv
        ```
    - Source the environment (Windows) or
        ```
        .venv\Scripts\Activate.ps1
        ```
    - Source the environment (Linux)
        ```
        source .venv\bin\activate
        ```
    
2. Install the required packages.
    ```
    pip install -r requirements.txt
    ```
    
3. Run the application.
    ```
    python3 teleop_gui/main.py
    ```

4. Run the test_listener.py file. [Optional]
    ```
    python3 teleop_gui/test_listener.py
    ```
