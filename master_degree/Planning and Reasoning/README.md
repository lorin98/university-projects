# Heuristic RRT-based approaches to improve planning in obstacle-populated environments

This is the repository of the project proposed for the course of *Planning and Reasoning*.

The project is based on the [**Open Motion Planning Library (OMPL)**](https://ompl.kavrakilab.org/).

## Compile and run the project

### Prerequisites

Please, [download the OMPL installation script](https://ompl.kavrakilab.org/install-ompl-ubuntu.sh) and follow the [instructions](https://ompl.kavrakilab.org/installation.html) from the OMPL documentation to install your desired version of the library:
```
chmod u+x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh [--python]/[--app]/[--github]
```
The script downloads and installs OMPL and all dependencies via *apt-get* & *pip* and from source.

For one experiment, [OpenCV](https://opencv.org/) is also required. You can install it in Ubuntu by running:
```
sudo apt update
sudo apt install libopencv-dev python3-opencv
```
According to the latest release, this will install OpenCV v4. With future versions, you should slightly modify the [Makefile](/Makefile) to correcly link the library.

### Compilation and execution
Continue downloading this repository as well. You can then compile and run the project with:
```
cd <your_path>/pr_project/
make
./HeuristicPlanning [--outfile <your_file.txt>, (default: "solution.txt")]
                    [--graphfile <your_file.graphml>, (default: "solution.graphml")]
                    [--exp <"obstacles", "maze", "real">, (default: "obstacles")]
                    [--planner <"RRT", "RRTstar", "InfRRTstar">, (default: "RRT")]
                    [--obj <"length", "clearance", "multi">, (default: "")]
                    [--runs <int>, (default: 1)]
```
This will save the solution into the specified text file.

Optionally, you are free to save also the search tree computed during the execution of the program by passing to the command line a non-empty argument for ```--graphfile``` (ex. tree.graphml). This GraphML file can be parsed to plot the tree as explained in the next section.

### Create video from solution

We provide a procedure to convert the solution path to a video simulation. You are free to use the Python script as follows:
```
cd solution_to_video/
pip install -r requirements.txt
python solution_to_video [--input_path, (default: "solution.txt")]
                         [--video_path, (default: "solution_to_video.mp4")]
                         [--tree_path, (default: "")]
                         [--exp, (default: "")]
```
This will create a video animation of the realized path.

Optionally, you can plot as a background image for the video the search tree saved before as follows:
```
cd solution_to_video/
python draw_tree [--graphml, (default: "solution.graphml")]
                 [--outfile, (default: "tree.png")]
                 [--exp, (default: "")]
```
This will create an image of the search tree graph that can be used as the ```--tree_path``` argument of the ```solution_to_video.py``` script above.

## Experiments

Each experiment has been run for *N* times and the related data cointaining number of states, path lengths, clearances and execution times are collected in the correspondent ```logs/solution_<environment>_<objective>.log``` files. These data have been parsed to produce the images of quantitative results grouped in the ```statistics/``` folder of this repository.

Please, refer to the [report](./report.pdf) for further details on implementantion, experimental set-ups and results.

### 1. Obstacles avoidance

It is an environment with two big areas and a narrow passage in the middle. The areas are populated with circular-shaped obstacles that have been placed in the scenario in order to show how results can differ accordingly to the objective the planner has to optimize. 

|         State      |     <img src="https://render.githubusercontent.com/render/math?math=\x">     | <img src="https://render.githubusercontent.com/render/math?math=\y">  | <img src="https://render.githubusercontent.com/render/math?math=\theta"> |
| ------------- |:-------------:| -----:| -----:|
| Start      | -2  |  0 | 0 |
| Goal     | 2 | 0 | <img src="https://render.githubusercontent.com/render/math?math=-\pi"> |

| RRT | RRT* | InformedRRT* |
|:----------:|:----------:|:----------:|
| <img src="/solution_to_video/obstacles/RRT/solution_obstacles_rrt.gif" width="300" height="290"/> | **Path length minimization:**<br><img src="/solution_to_video/obstacles/RRTstar/length/solution_obstacles_rrtstar.gif" width="300" height="290"/><br>**Minimum clearance maximization:**<br><img src="/solution_to_video/obstacles/RRTstar/clearance/solution_obstacles_rrtstar.gif" width="300" height="290"/> | **Path length minimization:**<br><img src="/solution_to_video/obstacles/InfRRTstar/length/solution_obstacles_infrrtstar.gif" width="300" height="290"/><br>**Minimum clearance maximization:**<br><img src="/solution_to_video/obstacles/InfRRTstar/clearance/solution_clearance_infrrtstar.gif" width="300" height="290"/>

### 2. Maze

It is an enviroment created as a tricky labyrinth with lots of dead-end corridors to test the ability of the planner of backtracking when useless tree branches are found in the search. The maze exit is indicated by the green arrow at the bottom left corner.

|         State      |     <img src="https://render.githubusercontent.com/render/math?math=\x">     | <img src="https://render.githubusercontent.com/render/math?math=\y">  | <img src="https://render.githubusercontent.com/render/math?math=\theta"> |
| ------------- |:-------------:| -----:| -----:|
| Start      | 2.5  |  2.5 | 0 |
| Goal     | 0.25 | 0.25 | <img src="https://render.githubusercontent.com/render/math?math=-\frac{\pi}{2}"> |

| RRT | RRT* | InformedRRT* |
|:----------:|:----------:|:----------:|
| <img src="/solution_to_video/maze/RRT/solution_maze_rrt.gif" width="300" height="290"/> | <img src="/solution_to_video/maze/RRTstar/length/solution_maze_rrtstar.gif" width="300" height="290"/> | <img src="/solution_to_video/maze/InfRRTstar/length/solution_maze_infrrtstar.gif" width="300" height="290"/>

### 3. Real-world test

It is a realistic environment representing a house with several rooms and obstacles contained therein. The user is free to select his/her desired goal state by specifying it through the OpenCV GUI. The following table contains two sampled runs with different goals and planning algorithms. The main goal of this final scenario is to study the behaviour of the robot in a close-to-truth world.

|         State      |     <img src="https://render.githubusercontent.com/render/math?math=\x">     | <img src="https://render.githubusercontent.com/render/math?math=\y">  | <img src="https://render.githubusercontent.com/render/math?math=\theta"> |
| ------------- |:-------------:| -----:| -----:|
| Start      | 2.675  |  6.5 | <img src="https://render.githubusercontent.com/render/math?math=\frac{\pi}{2}"> |
| Goal     | user choice | user choice | user choice |

| Goal | Planner |
|:----------:|:----------:|
| Go to the **Bedroom 3 (ID 6)** | **RRT***:<br><img src="/solution_to_video/real/RRTstar/solution_real_rrtstar_6.gif" width="500" height="360"/><br>**InformedRRT***:<br><img src="/solution_to_video/real/InfRRTstar/solution_real_infrrtstar_6.gif" width="500" height="360"/>
| Go to the **WC1 (ID 3)** | **RRT***:<br><img src="/solution_to_video/real/RRTstar/solution_real_rrtstar_3.gif" width="500" height="360"/><br>**InformedRRT***:<br><img src="/solution_to_video/real/InfRRTstar/solution_real_infrrtstar_3.gif" width="500" height="360"/>

## Author

Lorenzo Nicoletti - 1797464
