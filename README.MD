
This code runs an obstacle avoidance path planning method. 
It assumes the primitive for the path is given (from demonstration, by imitation, or hard-coded)
as a sequence of xy points in a matrix of size [n_steps, 2].
Once the primitive is loaded as a trajectory, it deforms it online so that the path countours the obstacle/hole on the surface. There are two options when an obstacle crosses a path: to move to the right or to the left of the obstacle. The solver will return the side which has the shortest path.

Uses python 3.

Install requirements

    pip install -r requirements.txt

## How to use

To run the code do

    python3 run_main.py

Make sure that the terminal is the active window as it is used to capture the keyboard commands.

## Stopping the code with the keyboard
With the terminal as the active windows, press ESC.

## Moving the obstacles with the keyboard
With the terminal as the active windows, use 'adsw' for obstacle 1, and 'jlki' for obstacle 2.

## Using and IDE (tested with pycharm)
In the case you want to use an IDE to debug (only tested with PyCharm), make sure
 that the argument of the main function in run_main.py is set to 'ide' as below

        main(parameters_wipe,
         interface_type = 'ide' # 'terminal'
         )

This will open a figure using OpenCV that must be the active window when you are using the keyboard to move the obstacles.






