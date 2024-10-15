# Simulator Mapping

## Requirements:

- Ubuntu 22.04
- At least 4 cores(If you're working from virtual machine please allocate at least 4 cores to the VM)
- Prefered: 6 cores

## Refrence
the simulator class and code is a project by rbdhaifa labs - `https://github.com/rbdlabhaifa/simulatorMapping `

## I. Installation:

### 0. Install the simulator
- `./install.sh`*
- `cd Thirdparty/slam/Vocabulary`
- `tar -xf ORBvoc.txt.tar.gz`
- `cd -`

* There is a chance that you will encouter some make errors - they happen because you don't give enough resources to the Virtual Machine. Just give more resources(You can find how easily in the internet). When you run again don't run `./install.sh` instead `cd build` and then `make`.

### 1. Use existing data or build new simulator:
This part is only for users of existing simulator and not meant for users who want to create new simulator model.
If you want to use existing simulator(like the one of the TLV University Lab) download this zip file:
https://drive.google.com/file/d/1McpMjSu7-ziM0-tqMumX0LM7evFS9Irs/view?usp=sharing

After you downloaded the zip file go to `generalSettings.json` and change the paths described in part 1 as follows:
if the path is in the `simulatorMapping` directory - just change the path to your `simulatorMapping` path.
if the path isn't in the `simulatorMapping` directory - change it to the equivalent file in the all-data file you downloaded

### 2. Configure the parameters to your machine:
Open generalSettings.json and change:
- `slam_configuration/vocabulary_path` to the path of your vocabulary on the project
- `slam_configuration/drone_yaml_path` to the path of your drone config file
- `offline_video_test_path` to the path of the video you want to run of offline orb slam(described later)
- `load_map_path`to the path of the map you created with orb slam
- `simulator_output_dir` to the directory you want the output maps will go to
- `simulator_configuration/model_path` to the path of your `.obj` model


## II. Usage:
this project specificaly is an implementation of a fire detection model using the simulator
- run the fire detection exacutable by going into directory  build
- from this directory  run `/FireDetectionApp/fireDetection` 
