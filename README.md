# CARLA
Collection of tests and files for working on manual control with logitech G920

### So far
- No longer using Logidrivepy for Logitech controllers
    - Logidrivepy is a better approach for mirroring autonomous driving in the controls
- Fully working steer and throttle, break and gear work but could use work on the realism
- Used pygame to integrate the controls

### To-Do
- In triple monitor, see why throttle starts without axis value ??
- in triple monitor, add brakes and gear change.
- Fix up brakes to be less sensitive
- Update gear change to a button instead (previously using a pedal)
    - using our third pedal is not very intuitive for the driver, testers strugle using three pedals
- Include manual brake and additional controls.
- Integrate these controls onto our autonomous vehicle so a driver can take over
    - I'm thinking of using a button to switch from manual to autonomous

### File Descriptions
#### Examples
- final_run.py (Working attempt)
    - We use .get_axis() from the pygame library to distinguish between the values of the different axis
    - Received feedback from a handful of testers and it looks + feels good!
- controller_test.py (Failed attempt)
    - (works but yikes) inside our event loop we obtain the values of each axis all at the same time
- control_test_2.py (Failed atttempt)
    - (working) inside our event loop we have if statements that determine what axis is being used
    - PROBLEMS: only registers one axis at a time aka cant steer and throttle at the same time
- triple_hc_trial_run.py (kinda works so far!)
    - manual control with triple camera using pygame joystick

#### Tests
- wheel_test.py
    - LOGIDRIVEPY: tests wheel spin and limits
- controller_outside_test.py
    - PYGAME: tests each axis and button in the Logitech setup
    - 
