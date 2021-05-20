using System;
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;
using System.Collections.Generic;
using CarWorld;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController2))]
    public class Brain : MonoBehaviour
    {   
        // the car controller we want to use
        private CarController2 carController;

        // rigid body of object of the car
        private Rigidbody rigidCar;
        
        public float speedAtTurn;

        // speed of car in kmph is stored here for output
        public float speed;

        // Boolean that decides whether car should move forward or not
        public bool startMoving = false;

        // Speed of car in units/seconds
        private float currentSpeed;

        // If this is true then brakes are applied. different blocks change this to apply brakes to the car 
        public bool isbreak = false;

        // Map sets this true if there is a turn ahead 
        private bool takeTurn = false;

        [Header("Sensors")]
        // Points to the current gps node
        public int nodeIndex = 0;

        //straight sensor position in front of car
        public Vector3 frontSensorPos = new Vector3(0f, 0f, 3f);

        //sensor position for side straight and side-angled sensors
        public Vector3 sidesensorpos = new Vector3(1f, 0f, 3f);

        //sensor position for sensors on walls
        public Vector3 sideWallSensorPos;

        //utility variable for calculating safe braking distance
        public float longSensorLength = 200f;

        //utility variable for calculating safe braking distance
        public float shortSensorLength = 20f;

        //stores the lidar readings of previous frame for all 5 sensors at the front 
        private readonly float[] prevLidar = new float[5];

        //denotes the number of nodes in the path to follow
        public int ListCount = 42;

        //m_NextNode is the next node that car needs to go, m_nodeToPathTo is the final destination node
        public TrafficSystemNode m_nextNode = null, m_nodeToPathTo = null;

        //vmax for the car
        public float TopSpeed = 20f;

        //boolean that denotes lane needs to be changed
        private bool changingLane = false;

        //angle at which side sensor is inclined thetal_l
        public float sideSensorAngle = 25f;

        //threshold distance for lane change. Currently not in use as we need to modify the map inorder to incorporate lane change at all nodes. Currently lane changing is allowed only at nodes where the map allows to do so. 
        public float laneChangeThresh = 4f;

        // this is the nodes that it will traverse to get to the "m_nodeToPathTo" destination
        private List<TrafficSystemNode> m_nodeToPathToRoute = new List<TrafficSystemNode>();

        //Safe distance that the side sensors need to maintain while changing lane
        public float sideLaneThresh = 5f;

        //denotes the start and end nodes for lane change
        private int turn_start, turn_end;

        private bool m_doneCalculatingPath = false;

        // Utility variable for path planning
        private bool m_calculatingPathToTake = false;

        // True when car is taking turn
        public bool checkTurn=false;

        //Called once at the start of script
        private void Start()
        {
            carController.m_Topspeed = TopSpeed;
            for (int i = 0; i < prevLidar.Length; i++)
            {
                prevLidar[i] = 0; //initialize the array
            }
        }

        private void Awake()
        {
            // get the car controller
            carController = GetComponent<CarController2>();
            rigidCar = GetComponent<Rigidbody>();
        }

        // called once per each update frame
        private void FixedUpdate()
        {
            WorldState currentWorldState = WorldState.Instance;

            if (!m_doneCalculatingPath && !m_calculatingPathToTake) // Wait until path planning is completed
            {
                CalculatePathToTake(m_nodeToPathTo);
                ListCount = 24;
                return;
            }
            if (!startMoving) // If start moving is false dont do anything
            {
                return;
            }
            ListCount = m_nodeToPathToRoute.Count; // Count of nodes in the path

#if !MOBILE_INPUT
            float handbrake = CrossPlatformInputManager.GetAxis("Jump"); // Handbrake can be applied using space bar

            if (!checkTurn && takeTurn && (currentWorldState.TotalNumberOfPoints < 40 || currentWorldState.LeftLanes == 0 || currentWorldState.RightLanes == 0)) // if map says to take turn and there are no lane lines
            {
                //db = 0;
                checkTurn = true;
            }

            speed = 3.6f * currentSpeed; //update the speed in the output
            if (checkTurn) // this section is run when car is navigating on intersection, roundabouts
            {
                if (nodeIndex > turn_end)
                {
                    checkTurn = false;
                    takeTurn = false;
                    changingLane = false;
                }
                else //in real scenarios these information will come from the inertial sensors. here we simply use coordinate system in unity
                {
                    Vector3 direction = m_nodeToPathToRoute[nodeIndex].transform.position - transform.position;  // estimates the vector from the car to the next node in the path 
                    WorldState.Instance.SteerAngle = -Vector3.SignedAngle(transform.forward, direction, transform.up);  // calculates steering angle
                }
            }

            /*if (!checkTurn)
            {
                if (CanLaneChange())
                {
                    
                }
            }*/
            if (nodeIndex < m_nodeToPathToRoute.Count) // This section simulates GPS
            {
                Vector3 dir = m_nodeToPathToRoute[nodeIndex].transform.position - transform.position;
                if (dir.magnitude < 3)
                {
                    nodeIndex++;
                    if (!takeTurn && nodeIndex < m_nodeToPathToRoute.Count)
                    {
                        if (m_nodeToPathToRoute[nodeIndex].isTurn)
                        {

                            takeTurn = true;
                            turn_start = nodeIndex;
                            turn_end = turn_start;
                            while (m_nodeToPathToRoute[turn_end].isTurn)
                            {
                                turn_end++;
                            }
                        }
                        else if (m_nodeToPathToRoute[nodeIndex - 1].m_lane != m_nodeToPathToRoute[nodeIndex].m_lane)
                        {
                            checkTurn = true;
                            takeTurn = true;
                            turn_end = nodeIndex + 2;
                            changingLane = true;
                        }

                    }
                }

            }
            // if car is at turn or the steering angle is greater than the threshold steering angle then limit the velocity at turn
            if (WorldState.Instance.SteerAngle >= 10 || WorldState.Instance.SteerAngle <= -10)
            {
                carController.m_Topspeed = speedAtTurn; // v_max_turn=10
            }
            else
            {
                carController.m_Topspeed = TopSpeed; // normal speed limits of the car
            }

            Sensor(-(float)WorldState.Instance.SteerAngle / 25, 0.5f, handbrake); //Steering angle sent to the sensors which control the velocity, 25 in first argument is actually maximum steer angle
#else
            m_Car.Move(h, v, v, 0f);
#endif
        }
        
        // Returns true if, it is possible to change lane
        /*private bool CanLaneChange()
        {
            
        }*/
        
       // private float Project


        // This function was taken from the old path planning script. There was a bug here which was corrected by Depansh. this doesnot incroporates traffic
        private void PathPlanning()
        {
            m_nodeToPathToRoute.Clear();

            if (m_nextNode == m_nodeToPathTo) return;

            // Implements A star algorithm
            // f, g, h function are standard notations for A star 
            List<Tuple<TrafficSystemNode, float, float>> candidateList = new List<Tuple<TrafficSystemNode, float, float>>();
            Dictionary<int, TrafficSystemNode> idToNodeMap = new Dictionary<int, TrafficSystemNode>();
            Dictionary<int, int> bestParent = new Dictionary<int, int>();
            Dictionary<int, float> distanceTillNode = new Dictionary<int, float>();
            float fBestYet = float.MaxValue;

            candidateList.Add(new Tuple<TrafficSystemNode, float, float>(m_nextNode, 0.0f, 0.0f));
            int startNodeId = m_nextNode.GetInstanceID();
            idToNodeMap.Add(startNodeId, m_nextNode);
            distanceTillNode.Add(startNodeId, 0.0f);

            // BFS like loop
            while (candidateList.Count > 0)
            {
                Tuple<TrafficSystemNode, float, float> candidate = popMin(ref candidateList);
                int candidateId = candidate.Item1.GetInstanceID();

                if (!idToNodeMap.ContainsKey(candidateId))
                {
                    idToNodeMap.Add(candidateId, candidate.Item1);
                }

                List<TrafficSystemNode> children = candidate.Item1.GetChildren();

                foreach (TrafficSystemNode child in children)
                {
                    int childId = child.GetInstanceID();
                    float g = candidate.Item3 + DisplacementMagnitude(child, candidate.Item1);
                    // f = g + h
                    float f = g + DisplacementMagnitude(child, m_nodeToPathTo);

                    if (distanceTillNode.ContainsKey(childId))
                    {
                        if (distanceTillNode[childId] > f)
                        {
                            distanceTillNode[childId] = f;
                            bestParent[childId] = candidateId;
                            candidateList.Add(new Tuple<TrafficSystemNode, float, float>(child, f, g));
                        }
                    }
                    else
                    {
                        distanceTillNode.Add(childId, f);
                        bestParent.Add(childId, candidateId);
                        candidateList.Add(new Tuple<TrafficSystemNode, float, float>(child, f, g));
                    }

                    // Reched destination, but is it the best path?
                    if (child == m_nodeToPathTo)
                    {
                        if (f < fBestYet)
                        {
                            fBestYet = f;
                        }
                    }
                }
                // Condition to ensure best path is selected, break only when g is more than best f achieved till yet
                if (fBestYet < candidate.Item2) break;
            }

            // Backtrack to get path
            int finishNodeId = m_nodeToPathTo.GetInstanceID();
            int currentNodeId = finishNodeId;

            // UnityEngine.Debug.Log("End Point " + finishNodeId);
            // UnityEngine.Debug.Log("Start Point " + startNodeId);

            while (true)
            {
                m_nodeToPathToRoute.Add(idToNodeMap[currentNodeId]);
                // UnityEngine.Debug.Log(currentNodeId);

                if (currentNodeId == startNodeId) break;

                currentNodeId = bestParent[currentNodeId];
            }
            // backtracking givves the reverse path
            m_nodeToPathToRoute.Reverse();
            // TestCalculatedPath();
            return;
        }

        // Utility function for path planning taken from older code
        private Tuple<TrafficSystemNode, float, float> popMin(ref List<Tuple<TrafficSystemNode, float, float>> candidateList)
        {
            float minval = float.MaxValue;
            int index = -1;
            for (int i = 0; i < candidateList.Count; i++)
            {
                if (candidateList[i].Item2 < minval)
                {
                    minval = candidateList[i].Item2;
                    index = i;
                }
            }
            Tuple<TrafficSystemNode, float, float> ret = candidateList[index];
            candidateList.RemoveAt(index);
            return ret;
        }

        // Utility vehicle for path planning taken from older code
        public TrafficSystemVehicle VehicleExistsOnNode(TrafficSystemNode a_node)
        {
            if (!a_node)
                return null;

            RaycastHit[] hitInfo;
            hitInfo = Physics.SphereCastAll(a_node.transform.position, 2.0f, transform.forward, 0.0f);

            for (int hIndex = 0; hIndex < hitInfo.Length; hIndex++)
            {
                TrafficSystemVehicle vehicle = hitInfo[hIndex].transform.GetComponent<TrafficSystemVehicle>();
                if (vehicle && vehicle != this && vehicle.m_velocity <= 0.0f)
                    return vehicle;
            }

            return null;
        }

        //Utility vehicle for path planning taken from older code. Wrapper for path planning
        public void CalculatePathToTake(TrafficSystemNode a_nodeToPathTo)
        {
            ListCount = -223;
            if (!m_nextNode)
                return; // we are not connected to the road network

            if (m_calculatingPathToTake) // we are already calculating a path to find
                return;

            m_nodeToPathTo = a_nodeToPathTo;

            if (m_nodeToPathTo && !m_doneCalculatingPath)
            {
                m_calculatingPathToTake = true;
                PathPlanning();
                m_doneCalculatingPath = true;
                m_calculatingPathToTake = false;
            }
        }

        private float DisplacementMagnitude(TrafficSystemNode a, TrafficSystemNode b)
        {
            Vector3 disp = a.transform.position - b.transform.position;
            return disp.magnitude;
        }

        // TODO Implement Logic
        private bool AtEndOfTheRoad()
        {
            return true;
        }

        //v is fractional motor torque, h is fractional steering angle
        void Sensor(float steeringAngle, float motorTorque, float handbrake)
        {
            
            isbreak = false; //set isBrakefalse
            float frontBrake = 0; //does same work as isBreak

            if (Vector3.Angle(rigidCar.velocity, transform.forward) < 50f && Vector3.Magnitude(rigidCar.velocity) > 0.01f) //applicable if car is moving forward
            {
                RaycastHit hit_mid, hit_left, hit_right, sideleft, sideright, lane_change, left_wall, right_wall; //different lidar sensors
                Vector3 sensorPos_mid = transform.TransformPoint(frontSensorPos); //calculating sensor position on car
                Vector3 sensorPos_right = transform.TransformPoint(sidesensorpos);
                sidesensorpos.x = -sidesensorpos.x;
                Vector3 sensorPos_left = transform.TransformPoint(sidesensorpos);
                Vector3 wallsense = transform.TransformPoint(sideWallSensorPos);
                sidesensorpos.x = -sidesensorpos.x;
                sideWallSensorPos.x = -sideWallSensorPos.x;
                Vector3 wallsenseR = transform.TransformPoint(sideWallSensorPos);
                sideWallSensorPos.x = -sideWallSensorPos.x;
                Vector3 rc = rigidCar.transform.position; //position vector of the car, in reality this is not needed. This is needed only for simulation purpose 
                currentSpeed = Vector3.Magnitude(rigidCar.velocity); //speed of car in units/second
                //shortSensorLength = (float)(2500 * vc * vc / (m_Car.m_BrakeTorque) + 0.5);
                float temp = Vector3.Magnitude(Vector3.Project(rigidCar.velocity, Quaternion.AngleAxis(sideSensorAngle, transform.up) * transform.forward));
                //float shortSideSensorLength = (float)(2500 * temp * temp / (m_Car.m_BrakeTorque) + 0.5);
                float shortSideSensorLength = shortSensorLength = 12f;
                bool b_mid = Physics.Raycast(sensorPos_mid, transform.forward, out hit_mid, shortSensorLength); //lidar simulation, first arg is pos, then direction, then the outut variable, and th range of sensor
                bool isLaneChange = Physics.Raycast(sensorPos_mid, transform.forward, out lane_change, laneChangeThresh);
                bool b_left = Physics.Raycast(sensorPos_left, transform.forward, out hit_left, shortSensorLength);
                bool b_right = Physics.Raycast(sensorPos_right, transform.forward, out hit_right, shortSensorLength);
                bool b_sideRight = Physics.Raycast(sensorPos_right, Quaternion.AngleAxis(sideSensorAngle, transform.up) * transform.forward, out sideright, shortSideSensorLength);
                bool b_sideLeft = Physics.Raycast(sensorPos_left, Quaternion.AngleAxis(-sideSensorAngle, transform.up) * transform.forward, out sideleft, shortSideSensorLength);
                bool b_leftwall= Physics.Raycast(wallsense, -transform.right, out left_wall, sideLaneThresh);
                bool b_rightwall = Physics.Raycast(wallsenseR, transform.right, out right_wall, sideLaneThresh); 
                //bool isLaneChange = Physics.Raycast(sensorPos_mid, transform.forward, out lane_change, laneChangeThresh);
                if (b_sideRight && !sideright.collider.CompareTag("Terrain")) //checking where did the sensor hit
                {
                    float d = Vector3.Magnitude(sideright.point - sensorPos_right);
                    if (Math.Abs(steeringAngle) > 0.5) //steering angle should be large enough to get the angled sensors working
                    {
                        float rd = (d - prevLidar[0]) / Time.deltaTime; //rate of change od lidar reading
                        float vother = rd + temp; //calculation of vother for angled sensor
                        if (vother >= 0) //if other vehicle has a positive velocity then calcculate ssafe
                        {
                            shortSensorLength = (float)(1250* currentSpeed * currentSpeed / (carController.m_BrakeTorque) + 0.5);
                        }
                        else
                        {
                            shortSensorLength = (float)(1250 * rd * rd / (carController.m_BrakeTorque) + 0.5);
                        }
                    }
                    else //if car is not turning take ssafe=so
                    {
                        shortSensorLength = 0.5f;
                    }
                    if (d < shortSensorLength) //if lidar reading is less thatn ssafe then brake
                    {
                         frontBrake = -2f;
                         isbreak = true;
                    }
                    prevLidar[0] = d;
                    
                    Debug.DrawLine(sensorPos_right, sideright.point, Color.red); //visualization sensor in scene mode
                }
                if (b_sideLeft && !sideleft.collider.CompareTag("Terrain"))
                {
                    float d = Vector3.Magnitude(sideleft.point - sensorPos_left);
                    if (Math.Abs(steeringAngle) > 0.5)
                    {
                        float rd = (d - prevLidar[1]) / Time.deltaTime;
                        float vother = rd + temp;
                        if (vother >= 0)
                        {
                            shortSensorLength = (float)(1250 * currentSpeed * currentSpeed / (carController.m_BrakeTorque) + 0.5);
                        }
                        else
                        {
                            shortSensorLength = (float)(1250 * rd * rd / (carController.m_BrakeTorque) + 0.5);
                        }
                    }
                    else
                    {
                        shortSensorLength = 0.5f;
                    }
                    if (d < shortSensorLength)
                    {
                        frontBrake = -2f;
                        isbreak = true;
                    }
                    prevLidar[1] = d;
                    
                   
                    Debug.DrawLine(sensorPos_left, sideleft.point, Color.black);
                }
                if (b_mid && !hit_mid.collider.CompareTag("Terrain"))
                {
                    float d = Vector3.Magnitude(hit_mid.point - sensorPos_mid);
                    float rd = (d - prevLidar[2]) / Time.deltaTime;
                    float vother = rd + currentSpeed;
                    if (vother >= 0)
                    {
                        shortSensorLength = (float)(2500 * currentSpeed * currentSpeed / (carController.m_BrakeTorque) + 0.5);
                    }
                    else
                    {
                        shortSensorLength = (float)(2500 * rd * rd / (carController.m_BrakeTorque) + 0.5);
                    }
                    if (d < shortSensorLength)
                    {
                        frontBrake = -2f;
                        isbreak = true;
                    }
       
                    prevLidar[2] = d;

                    Debug.DrawLine(sensorPos_mid, hit_mid.point);
                }
                if (b_left && !hit_left.collider.CompareTag("Terrain"))
                {
                    float d = Vector3.Magnitude(hit_left.point - sensorPos_left);
                    float rd = (d - prevLidar[3]) / Time.deltaTime;
                    float vother = rd + currentSpeed;
                    if (vother >= 0)
                    {
                        shortSensorLength = (float)(2500 * currentSpeed * currentSpeed / (carController.m_BrakeTorque) + 0.5);
                    }
                    else
                    {
                        shortSensorLength = (float)(2500 * rd * rd / (carController.m_BrakeTorque) + 0.5);
                    }
                    if (d < shortSensorLength)
                    {
                        frontBrake = -2f;
                        isbreak = true;
                    }

                    prevLidar[3] = d;


                     //handbrake = 2f;


                    Debug.DrawLine(sensorPos_left, hit_left.point, Color.black);
                }
                if (b_right && !hit_right.collider.CompareTag("Terrain"))
                {
                    float d = Vector3.Magnitude(hit_right.point - sensorPos_right);
                    float rd = (d - prevLidar[4]) / Time.deltaTime;
                    float vother = rd + currentSpeed;
                    if (vother >= 0)
                    {
                        shortSensorLength = (float)(2500 * currentSpeed * currentSpeed / (carController.m_BrakeTorque) + 0.5);
                    }
                    else
                    {
                        shortSensorLength = (float)(2500 * rd * rd / (carController.m_BrakeTorque) + 0.5);
                    }
                    if (d < shortSensorLength)
                    {
                        frontBrake = -2f;
                        isbreak = true;
                    }

                    prevLidar[4] = d;

                    //handbrake = 2f;

                    //handbrake = 2f;
                    Debug.DrawLine(sensorPos_right, hit_right.point, Color.red);
                }
                if(changingLane && b_leftwall && !left_wall.collider.CompareTag("Terrain"))  //brakig while lane changing usig side sensor
                {
                    frontBrake = -2f;
                    isbreak = true;
                    Debug.DrawLine(wallsense, left_wall.point, Color.red);
                }
                if (changingLane &&  b_rightwall && !right_wall.collider.CompareTag("Terrain")) //brakig while lane changing usig side sensor
                {
                    frontBrake = -2f;
                    isbreak = true;
                    Debug.DrawLine(wallsenseR, right_wall.point, Color.red);
                }
            }
            else if (Vector3.Magnitude(rigidCar.velocity) > 0.01f) //apply same kind of logic for moving back. I did not do much here but one can adjust this for back sensor. Since the car in the scene is not moving back (current map structure does not allow that) so not much emphasis was not paid here.
            {
                RaycastHit hit_mid, hit_left, hit_right, sideleft, sideright;
                frontSensorPos.z = -frontSensorPos.z;
                sidesensorpos.z = -sidesensorpos.z;
                Vector3 sensorPos_mid = transform.TransformPoint(frontSensorPos);
                Vector3 sensorPos_right = transform.TransformPoint(sidesensorpos);
                sidesensorpos.x = -sidesensorpos.x;
                Vector3 sensorPos_left = transform.TransformPoint(sidesensorpos);
                sidesensorpos.x = -sidesensorpos.x;
                shortSensorLength = (float)(5000 * Vector3.Magnitude(rigidCar.velocity) * Vector3.Magnitude(rigidCar.velocity) / (carController.m_BrakeTorque) + 1);
                frontSensorPos.z = -frontSensorPos.z;
                sidesensorpos.z = -sidesensorpos.z;
                float temp = Vector3.Magnitude(Vector3.Project(rigidCar.velocity, Quaternion.AngleAxis(sideSensorAngle, transform.up) * transform.forward));
                float shortSideSensorLength = (float)(5000 * temp * temp / (carController.m_BrakeTorque) + 1);
                bool b_mid = Physics.Raycast(sensorPos_mid, -transform.forward, out hit_mid, shortSensorLength);
                bool b_left = Physics.Raycast(sensorPos_left, -transform.forward, out hit_left, shortSensorLength);
                bool b_right = Physics.Raycast(sensorPos_right, -transform.forward, out hit_right, shortSensorLength);
                bool b_sideRight = Physics.Raycast(sensorPos_right, Quaternion.AngleAxis(sideSensorAngle, -transform.up) * -transform.forward, out sideright, shortSideSensorLength);
                bool b_sideLeft = Physics.Raycast(sensorPos_left, Quaternion.AngleAxis(-sideSensorAngle, -transform.up) * -transform.forward, out sideleft, shortSideSensorLength);
                if (b_left && !hit_left.collider.CompareTag("Terrain"))
                {
                    frontBrake = -2f;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_left, hit_left.point, Color.red);
                }
                if (b_mid && !hit_mid.collider.CompareTag("Terrain"))
                {
                    frontBrake = -2f;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_mid, hit_mid.point, Color.white);
                }
                if (b_right && !hit_right.collider.CompareTag("Terrain"))
                {
                    frontBrake = -2f;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_right, hit_right.point, Color.black);
                }
                if (b_sideRight && !sideright.collider.CompareTag("Terrain"))
                {
                    frontBrake = -2f;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_right, sideright.point);
                }
                if (b_sideLeft && !sideleft.collider.CompareTag("Terrain"))
                {
                    frontBrake = -2f;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_left, sideleft.point);
                }
            }

            if (WorldState.Instance.TrafficLightColor == 'r')
            {
                isbreak = true;
            }
            carController.Move(steeringAngle, motorTorque, frontBrake, handbrake, isbreak); //Giving the inputs to the car,fractional steering angle=h, v=fractional motor torque, handbrake and isbrake
        }
    }
}

