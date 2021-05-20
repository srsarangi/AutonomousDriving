# AutonomousDriving

## How to get started?

Follow the procedure mentioned below on Windows

1. Clone the repository
   1. The enitre commit history is quite large, so to shallow clone use  `git clone --depth 1 https://github.com/srsarangi/AutonomousDriving.git`
2. Open folder named city as a project in unity 2018.4 (Last tested with 2018.4.19f1)
3. Follow the Lane detection section (mandatory) and 3D Object detection section (optional, follow it only if you want to use PV-RCNN)

## Lane detection
This project uses PINet for lane detection, see Key Points Estimation and Point Instance Segmentation Approach for Lane Detection- {Yeongmin Ko, Younkwan Lee}.
Parts of lane detection code was taken from https://github.com/koyeongmin/PINet.
Requirements for lane detection
   1. Install Python 3.6.5 and set installation directory as path (Option in installer itself)
   2. NVIDIA Graphics card is required for running the lane detection code. On CPU it runs at a very low frame rate.
      1. pytorch (<=1.5.1) required with a suitable version of CUDA toolkits installed. The code was last tested on pytorch version 1.5.1 with CUDA version 9.2 
      2. Other depdendencies required are
		- opencv-python
		- numpy
		- visdom
		- sklearn
		- ujon
		- csaps
5. To install all python dependencies use 'conda create --name <env> --file requirements.txt'. If you don't want to use conda then individual dependencies need to be installed using pip. This installs python dependencies for all modules which includes
      1. Object detection and tracking
      2. Lane detection

## 3D Object detection
In this project we had also set up PV-RCNN- a deep neural network for 3D object detection on point cloud. We are not using it currently. Instead we are using our own algorithm for 3D object detection.
To set up PV-RCNN follow the instructions on https://github.com/open-mmlab/OpenPCDet
Dependecies for our algortihm is included in requirements.txt.


## Running Level 3 automated car
1. Go to the Unity project. Open Assets/Scenes/Lane Detection_4
2. Click on play button.
3. Go to Inference_Server folder inside the root directory of project, then run inference_server3.py.
4. If you are using our system for running the simulation, then all dependencies have been installed in the conda virtual environment. Run the command following command before running the inference_server3.py
	conda activate spconv
5. Once the python program starts running click on the Car object from left pane. On the inspector plane (on the right side) find ACC2 script and tick start moving button.
6. The car should start self-driving.
7. The path of car can be changed by setting a different next node and nodeToPath node.