# Unmanned Aerial Vehicles (UAVs) 3D-Motion Planning
![Quad Image](./misc/enroute.png)

This project is a continuation of the Backyard Flyer project, in which the drone executed a simple square shaped flight path. In this project, I have implemented a motion planning algorithm, which enables the drone to fly through the simulated city of San Francisco, CA, from a start location to a specified goal (latitude-longitude) location on the map.

## Project Implementation Write-up

The project implementation summary can be found in this [project write-up](./writeup.md).

## Cloning and executing this project on your local machine

The instructions for executing the drone flight on you local machine can be found under the *<b>Executing The Flight</b>* section in the [project write-up](./writeup.md).

Please follow the project execution setup below, before your flight!

### Project Execution Setup
#### Step 1: Download the Simulator

This is a Udacity Simulator environment!  

Download the Motion-Planning simulator for this project that's appropriate for your operating system from the [simulator releases respository](https://github.com/udacity/FCND-Simulator-Releases/releases).

#### Step 2: Set up your Python Environment
If you haven't already, set up your Python environment and get all the relevant packages installed using Anaconda following instructions in [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit)

#### Step 3: Clone this Repository
```sh
git clone https://github.com/ashishsnaik/UAVs-3D-Motion-Planning.git
```
#### Step 4: Test the setup
The first task in this project is to test the [solution code](./backyard_flyer_solution.py) for the Backyard Flyer project in the simulator. Verify that your Backyard Flyer solution code works as expected and your drone can perform the square flight path in the new simulator. To do this, start the simulator and run the [`backyard_flyer_solution.py`](./backyard_flyer_solution.py) script.

```sh
source activate fcnd # if you haven't already sourced your Python environment, do so now.
python backyard_flyer_solution.py
```
The quad should take off, fly a square pattern and land, just as in the previous project. If everything functions as expected then you are ready to start work on this project. 
