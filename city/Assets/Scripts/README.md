# Folder structure

1. `carDrivingAlgo`
   1. All algorithms for each version of self driving car
      1. Prabhleen's car
      2. Node and Lane detection based level 2 car 
2. `carPhysics`
   1. Mechanics of car like wheel rotation
   2. Scripts for Sensors
   3. Code for Lidar using depth camera
3. `environment`
   1. Environment aspects of simulator like 
      1. making nodes invisible in run mode
      2. Getting the image captured by a camera
      3. Lighting rotation script to simulate day and night
4. `externalScripts`
   1. General implementation which would run as a separate process than unity
      1. Lane detection
         1. Canny Edges Implementation
         2. Sliding Window polynomial fiting
         3. Lanenet (requires further cloning of the repo mentioned in file)
5. `interProcessCommunication`
   1. For sending info from unity to other programs IPC is required
   2. Contains following implementation of IPC - 
      1. Socket `socket`
      2. Shared Memory - Memory Mapped Files (Windows Specific) `MMF`
      3. Pipes `pipe`
   3. With parallelization synchronization is also required for which `PetersonLock` is implemented
6. `NeuralNetworkScripts`
   1. CNN algorithm for car control for later stages of automation 
7. `projectManagement`
   1. Welcome screen that controls the navigation to different scenes of projects  
   2. Car physics and behaviour management scripts
   3. Car movement scripts
   4. Circuit follow for node based movement of car in a circuit 
8. HuracanCar folder
   1. Contains all scripts related to manuvering and handeling of New Car (Lamborghini Huracán)    
   
   
