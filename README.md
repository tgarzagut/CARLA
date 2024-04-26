# CARLA
Collection of tests and files for working with CARLA Simulator using logitech G920 controller

### Comments
- No longer using Logidrivepy for Logitech controllers
    - Logidrivepy is a better approach for mirroring autonomous driving in the controls
- Used pygame to integrate the controls
- Fully working controls for manual (single and triple screen)


### To-Do
- See if you can switch screen size for middle triple screen.
- Implement controls for the semiautonomous

### File Descriptions
#### Examples
- miniScenario_1
    - Implemented correct spawn point, pedestrian spawns too early. Need to change either spawn location or pedestrian spwan point
    - Need to implement manual control, switch from autonomous to manual already implemented
- final_run.py (Working attempt)
    - We use .get_axis() from the pygame library to distinguish between the values of the different axis
    - Received feedback from a handful of testers and it looks + feels good!
- triple_hc_trial_run.py (working attempt)
    - Manual control with triple camera using pygame joystick.
- semiauto_test.py (In progress...)
    - Semi-autonomous attempt
    - Switch from autonomous to manual (vice-versa?)

- controller_test.py (Failed attempt)
    - (works but yikes) inside our event loop we obtain the values of each axis all at the same time
- control_test_2.py (Failed atttempt)
    - (working) inside our event loop we have if statements that determine what axis is being used
    - PROBLEMS: only registers one axis at a time aka cant steer and throttle at the same time

#### Tests
- wheel_test.py
    - LOGIDRIVEPY: tests wheel spin and limits
- controller_outside_test.py
    - PYGAME: tests each axis and button in the Logitech setup
-  testing_controls.py
    - LOGIDRIVEPY: 
