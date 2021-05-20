# Folder Structure

1. `Scripts` contains all the C# and Python code written by us. It is also the recommended directory to have user written scripts
2. `Scenes`
   1. Contains all scenes of different levels and functionality implemented in the simulator 
3. `SocketIO`
   1. Unity's implementation to use socket by adding a game object
4. `Traffic System`
   1. Everything related to Roads, Traffic Lights and Traffic Vehicles
   2. Was imported as an asset, bought from Unity Store
5. `Pedestrian System`
   1. Assets for pedestrians on the walkway 
   2.`Pedestrian system incorporated from Urban Traffic System (UTS pro)   
6. `Streaming Assets`
    1. OpenCvforUnity sub folder contains build requirements for object detection ie weights cfg files etc 
    2. Deep Neural Network weights
    3. Dnn folder contains necessary weights and CFG for yolov3-tiny used as a classifier for object detection
7. `Unity Capture`
    1. High performance camera that converts Texture2D to webcamtexure to be used by OpenCvforUnity
    2. Capable of giving output up to 212 fps which is very fast
    3. Used in Proj for capturing the simulator screen and rendering it as a webcamtexure to opencvforunity for further processing
8. `Standard Assets`
   1. Unity's standard assets - commonly required game objects and scripts 
   2. Essential for building project