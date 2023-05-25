Git clone inside src folder of the workspace

# To build the pakage
1. Install dependensies
    ```
    rosdep install -i --from-paths src --rosdistro foxy -y
    ```
  
2. Build and source using
    ```
    colcon build --symlink-install
    source install/setup.bash
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
    ros2 launch teleop_gui controller.launch.py
    ```

   Run with the test_listener.py file. [Optional]
    ```
    ros2 launch teleop_gui controller.launch.py test:=True
    ```
   
# Controls:
```
        W                   ↑
    S       D               ↓
     (Angle)             (Speed)

     To hold mouse press : Shift Key
```
