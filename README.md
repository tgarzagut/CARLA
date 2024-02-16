# CARLA
Working on manual control with logitech G920

### So far
- No longer using Logidrivepy for Logitech controllers
    - Logidrivepy is a better approach for mirroring autonomous driving in the controls
- Connection controls have passed
- Have connectivity when running CARLA simulation
- February:
    - Fully working pedals + wheel (yay!)
    - Needs debugging


### To-Do
- Fix glitches in wheel steering.
    - Pygame can only register one event.axis at a time, attempt to use .get_axis() to correct that
- Inside controller_test.py
    - (untested) inside our event loop we obtain the values of each axis all at the same time targeting different axis with .get_axis()
- Inside control_test_2.py
    - (working) inside our event loop we have if statements that determine what axis is being used
    - PROBLEMS: only registers one axis at a time aka cant steer and throttle at the same time