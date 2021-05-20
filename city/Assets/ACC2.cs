using System;
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;
using System.IO.MemoryMappedFiles;
using System.Threading;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;
using System.Text;
using static TrafficSystemNode;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController2))]
    public class ACC2 : MonoBehaviour
    {

        [DllImport("sem", EntryPoint="unlinkSHM", CharSet = CharSet.Ansi)]
        public static extern void  unlinkSHM(string str);
        [DllImport("sem", EntryPoint="reset", CharSet = CharSet.Ansi)]
        public static extern void  reset();
        [DllImport("sem", EntryPoint="ReadInt", CharSet = CharSet.Ansi)]
        public static extern int ReadInt(int mmap, int size);
        [DllImport("sem", EntryPoint="WriteInt", CharSet = CharSet.Ansi)]
        public static extern int WriteInt(int val, int mmap);
        [DllImport("sem", EntryPoint="semaphore_open", CharSet = CharSet.Ansi)]
        public static extern int semaphore_open(string semname, int oflag, int val);
        [DllImport("sem", EntryPoint="getO_Creat", CharSet = CharSet.Ansi)]
        public static extern int getO_Creat();
        [DllImport("sem", EntryPoint="wait", CharSet = CharSet.Ansi)]
        public static extern void wait(int ind);
        [DllImport("sem", EntryPoint="post", CharSet = CharSet.Ansi)]
        public static extern void post(int ind);
        [DllImport("sem", EntryPoint="shared_mem_open", CharSet = CharSet.Ansi)]
        public static extern int shared_mem_open(string name, int shm_flag);
        [DllImport("sem", EntryPoint="ftrunc", CharSet = CharSet.Ansi)]
        public static extern void ftrunc(int shm_fd, int size);
        [DllImport("sem", EntryPoint="mmap_obj", CharSet = CharSet.Ansi)]
        public static extern int mmap_obj(int size, int shm_fd);
        [DllImport("sem", EntryPoint="writeMMF", CharSet = CharSet.Ansi)]
        public static extern void writeMMF(string msg, int mmap);
        [DllImport("sem", EntryPoint="test", CharSet = CharSet.Ansi)]
        public static extern string test();
        [DllImport("sem", EntryPoint="readMMF", CharSet = CharSet.Ansi)]
        public static extern string readMMF(int mmap, int size);
        [DllImport("sem", EntryPoint="getO_CREAT_ORDWR", CharSet = CharSet.Ansi)]
        public static extern int getO_CREAT_ORDWR();
        private CarController2 m_Car; // the car controller we want to use
        private Rigidbody rigid_car; //rigid body of object of the car
        public float speed; //speed of car in kmph is stored here for output
        public bool startMoving = false; //Boolean that decides whether car should move forward or not
        public double SteerAngle; //steering angle of the car
        private float vc; //speed of car in units/seconds
        public bool isbreak = false; //if this is true then brakes are applied. different blocks change this to apply brakes to the car
        private int mmf, mmf2, mmf3; //MMF of objects for exchanging steering angle, number of lanes detected from another mmfProcessClient.py
        //private MemoryMappedViewAccessor accessor, accessor2, accessor3; //accessor for above MMF objects
        private bool takeTurn = false; //Map sets this true if there is a turn ahead 
        private int lockformmf; //lockformmf2;
        private int curr_lane_lock, curr_lane_mmf;
        private float acc_val=0.5f; //The value to accelrate forward: range is from 0 to 1
        [Header("Sensors")]
        public int node_index = 0; //points to the current gps node
        public Vector3 frontSensorPos = new Vector3(0f, 0f, 3f); //straight sensor position in front of car
        public Vector3 sidesensorpos = new Vector3(1f, 0f, 3f); //sensor position for side straight and side-angled sensors
        public Vector3 sideWallSensorPos; //sensor position for sensors on walls
        public float longSensorLength = 200f; //utility variable for calculating safe braking distance
        public float shortSensorLength = 20f; //utility variable for calculating safe braking distance
        public bool isWait = true; //used for synchronization at the start of the program
        private float[] prevLidar = new float[5]; //stores the lidar readings of previous frame for all 5 sensors at the front 
        public int ListCount = 42; //denotes the number of nodes in the path to follow
        // private Vector3 prevPos=new Vector3(0f,0f, 0f);
        public TrafficSystemNode m_nextNode = null, m_nodeToPathTo = null; //m_NextNode is the next node that car needs to go, m_nodeToPathTo is the final destination node  
        public float TopSpeed = 20f; //vmax for the car
        private bool changingLane = false; //boolean that denotes lane needs to be changed
        public float sideSensorAngle = 25f; //angle at which side sensor is inclined thetal_l
        public float laneChangeThresh = 4f; //threshold distance for lane change. Currently not in use as we need to modify the map inorder to incorporate lane change at all nodes. Currently lane changing is allowed only at nodes where the map allows to do so. 
        private List<TrafficSystemNode> m_nodeToPathToRoute = new List<TrafficSystemNode>(); // this is the nodes that it will traverse to get to the "m_nodeToPathTo" destination
        public float sideLaneThresh = 5f; //Safe distance that the side sensors need to maintain while changing lane
        
        private int turn_start, turn_end; //denotes the start and end nodes for lane change
        // private static float time = 0f;
        private bool m_doneCalculatingPath = false, m_calculatingPathToTake = false; //utility variable for path planning
        public bool checkTurn=false; //true when car is taking turn
        //this function was taken from the old path planning script. There was a bug here which was corrected by Depansh. this doesnot incroporates traffic
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
                    float g = candidate.Item3 + displacementMagnitude(child, candidate.Item1);
                    // f = g + h
                    float f = g + displacementMagnitude(child, m_nodeToPathTo);

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

            m_doneCalculatingPath = true;
            m_calculatingPathToTake = false;

            // TestCalculatedPath();
            return;
        }
        /*private void PathPlanning()
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

                    float timeweight = calculatetime(child);

                    float g = candidate.Item3 + displacementMagnitude(child, candidate.Item1) + displacementMagnitude(child, candidate.Item1) * timeweight;
                    // f = g + h
                    float bigweight = longtimewait(child, m_nodeToPathTo);
                    float f = g + displacementMagnitude(child, m_nodeToPathTo) + displacementMagnitude(child, m_nodeToPathTo) * bigweight;

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

            m_doneCalculatingPath = true;
            m_calculatingPathToTake = false;

            // TestCalculatedPath();
            return;
        }*/
        //Utility function for path planning taken from older code
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
        private float minimum(List<float> l)
        {
            float z = float.MaxValue;
            foreach (float f in l)
            {
                if (z > f)
                {
                    z = f;
                }

            }
            return z;

        }
        //Utility vehicel for path planning taken from older code
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
        //Utility vehicel for path planning taken from older code. Wrapper for path planning
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
            }
        }
        //Written by Deepans, this is of no use. There is error in this function
        /*private bool IsRightAheadTurn(List<TrafficSystemNode> nodelist, int m)
        {
            for (int i = 0; i < m; i++)
            {
                if ((nodelist[i].m_directionListing != DirectionListing.RIGHT_STRAIGHT) && (nodelist[i].m_directionListing != DirectionListing.LEFT_RIGHT))
                {
                    continue;
                }
                List<TrafficSystemNode> temp = nodelist[i].m_connectedChangeLaneNodes;
                if (nodelist[i].m_directionListing == DirectionListing.LEFT_RIGHT)
                {
                    if (i == m - 1)
                    {
                        return false;

                    }
                    for (int k = 0; k < temp.Count; k++)
                    {
                        if (nodelist[i + 1] == temp[k])
                        {
                            if (temp[k].m_driveSide == TrafficSystem.DriveSide.RIGHT)
                            {
                                return true;
                            }
                            else
                            {
                                continue;
                            }

                        }
                    }


                }
                else if (nodelist[i].m_directionListing == DirectionListing.RIGHT_STRAIGHT)
                {
                    if (i == m - 1)
                    {
                        return false;

                    }


                    if (temp.Count == 1)
                    {
                        if (nodelist[i + 1] == temp[0])
                        {
                            return true;
                        }
                        else if (nodelist[i + 1] != temp[0])
                        {
                            continue;
                        }
                    }
                    else if (temp.Count != 1)
                    {
                        for (int j = 0; j < temp.Count; j++)
                        {
                            if (nodelist[i + 1] == temp[j])
                            {
                                if (temp[j].m_roundaboutExit == RoundaboutExit.EXIT_2)
                                {
                                    break;
                                }
                                else if (temp[j].m_roundaboutExit == RoundaboutExit.EXIT_1)
                                {
                                    return true;
                                }

                            }
                        }
                    }

                }
                {


                }


            }

            return false;

        }*/
        //Written by deepansh, there is error in this function it is of no use
        /*private bool IsLeftAheadTurn(List<TrafficSystemNode> nodelist, int m)
        {
            for (int i = 0; i < m; i++)
            {
                if ((nodelist[i].m_directionListing != DirectionListing.LEFT_RIGHT) && (nodelist[i].m_directionListing != DirectionListing.LEFT_STRAIGHT_UTURN) && (nodelist[i].m_directionListing != DirectionListing.LEFT_STRAIGHT))
                {
                    continue;
                }
                List<TrafficSystemNode> temp = nodelist[i].m_connectedChangeLaneNodes;
                if (nodelist[i].m_directionListing == DirectionListing.LEFT_STRAIGHT)
                {
                    if (i == m - 1)
                    {
                        return false;
                    }
                    else if (nodelist[i + 1] == temp[0])
                    {
                        return true;
                    }
                    else if (nodelist[i + 1] != temp[0])
                    {
                        continue;
                    }


                }

                else if (nodelist[i].m_directionListing == DirectionListing.LEFT_RIGHT)
                {
                    if (i == m - 1)
                    {
                        return false;
                    }
                    for (int k = 0; k < temp.Count; k++)
                    {
                        if (nodelist[i + 1] == temp[k])
                        {
                            if (temp[k].m_driveSide == TrafficSystem.DriveSide.LEFT)
                            {
                                return true;
                            }
                            else
                            {
                                continue;
                            }
                        }
                    }
                }

                else if (nodelist[i].m_directionListing == DirectionListing.LEFT_STRAIGHT_UTURN)
                {
                    if (i == m - 1)
                    {
                        return false;
                    }
                    for (int k = 0; k < temp.Count; k++)
                    {
                        if (nodelist[i + 1] == temp[k])
                        {
                            if (temp[k].m_roundaboutExit == RoundaboutExit.EXIT_3 || temp[k].m_roundaboutExit == RoundaboutExit.EXIT_4)
                            {
                                return true;
                            }
                            else
                            {
                                break;
                            }


                        }

                    }
                }


            }


            return false;


        }*/
        //Utility vehicel for path planning taken from older code
        private List<TrafficSystemNode> pathlist(TrafficSystemNode a, TrafficSystemNode b, TrafficSystemNode c)
        {
            List<TrafficSystemNode> list = new List<TrafficSystemNode>();
            list.Clear();

            if (b == c)
            {
                list.Add(c);
                return list;
            }

            // Implements A star algorithm
            // f, g, h function are standard notations for A star 
            List<Tuple<TrafficSystemNode, float, float>> candidateList = new List<Tuple<TrafficSystemNode, float, float>>();
            Dictionary<int, TrafficSystemNode> idToNodeMap = new Dictionary<int, TrafficSystemNode>();
            Dictionary<int, int> bestParent = new Dictionary<int, int>();
            Dictionary<int, float> distanceTillNode = new Dictionary<int, float>();
            float fBestYet = float.MaxValue;

            candidateList.Add(new Tuple<TrafficSystemNode, float, float>(b, 0.0f, 0.0f));
            int startNodeId = b.GetInstanceID();
            idToNodeMap.Add(startNodeId, b);
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
                    float g = candidate.Item3 + displacementMagnitude(child, candidate.Item1);
                    // f = g + h
                    float f = g + displacementMagnitude(child, c);

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
                    if (child == c)
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
            int finishNodeId = c.GetInstanceID();
            int currentNodeId = finishNodeId;

            // UnityEngine.Debug.Log("End Point " + finishNodeId);
            // UnityEngine.Debug.Log("Start Point " + startNodeId);

            while (true)
            {
                list.Add(idToNodeMap[currentNodeId]);
                // UnityEngine.Debug.Log(currentNodeId);

                if (currentNodeId == startNodeId) break;

                currentNodeId = bestParent[currentNodeId];
            }
            // backtracking givves the reverse path
            list.Reverse();



            // TestCalculatedPath();
            return list;
        }
        //Written by Deepansh, does not work well
        /*private float longtimewait(TrafficSystemNode a, TrafficSystemNode b)
        {
            List<TrafficSystemNode> childrenlist = a.GetChildren();
            List<float> timedelay = new List<float>();
            foreach (TrafficSystemNode child in childrenlist)
            {
                List<TrafficSystemNode> path = pathlist(a, child, b);
                float trafficweight = 0;
                foreach (TrafficSystemNode nextnode in path)
                {
                    trafficweight = trafficweight + calculatetime(nextnode);
                }
                timedelay.Add(trafficweight);

            }

            float x = minimum(timedelay);
            return x;

        }*/
        /*
        private float calculatetime(TrafficSystemNode node)
        {
            //int cars = 0;
            //int count = 0;
            float vsum = 0;
            List<TrafficSystemNode> list = node.GetChildren();
            foreach (TrafficSystemNode child in list)
            {

                if (VehicleExistsOnNode(child) != null)
                {
                    float x = VehicleExistsOnNode(child).m_velocity;
                    vsum = vsum + (1 / (1 + x));
                }
                else
                {
                    continue;
                }

            }



            return vsum;



        }
        */
        private float displacementMagnitude(TrafficSystemNode a, TrafficSystemNode b)
        {
            Vector3 disp = a.transform.position - b.transform.position;
            return disp.magnitude;
        }
        //Called once at the start of script
        private void Start()
        {
            for(int i=0; i<prevLidar.Length; i++)
            {
                prevLidar[i] = 0; //initialize the array
            }
            
            /*m_nodeToPathToRoute.Add(nd1);
            m_nodeToPathToRoute.Add(nd2);
            m_nodeToPathToRoute.Add(nd3);
            m_nodeToPathToRoute.Add(nd4);
            m_nodeToPathToRoute.Add(nd5);
            m_nodeToPathToRoute.Add(nd6);*/
            int shm_fd=shared_mem_open("steerAngle", getO_CREAT_ORDWR());
            ftrunc(shm_fd, 20);
            mmf=mmap_obj(20, shm_fd);
            //mmf = MemoryMappedFile.OpenExisting("steerAngle");
            shm_fd=shared_mem_open("total_pts", getO_CREAT_ORDWR());
            ftrunc(shm_fd, 20);
            mmf2=mmap_obj(20, shm_fd);
            //mmf2 = MemoryMappedFile.OpenExisting("numLeft");
            shm_fd=shared_mem_open("numLane", getO_CREAT_ORDWR());
            ftrunc(shm_fd, 20);
            mmf3=mmap_obj(20, shm_fd);
            
            shm_fd=shared_mem_open("curr_lane", getO_CREAT_ORDWR());
            ftrunc(shm_fd, 20);
            curr_lane_mmf=mmap_obj(20, shm_fd);
            //mmf3 = MemoryMappedFile.OpenExisting("numRight");
            lockformmf=semaphore_open("lockSteer", getO_Creat(), 1);
            curr_lane_lock=semaphore_open("curr_lane_lock", getO_Creat(), 1);
            wait(curr_lane_lock);
            WriteInt(m_nextNode.m_lane, curr_lane_mmf);
            post(curr_lane_lock);
            //lockformmf = Mutex.OpenExisting("lockSteer");
            //accessor = mmf.CreateViewAccessor();
            //accessor2 = mmf2.CreateViewAccessor();
            //accessor3 = mmf3.CreateViewAccessor();
            //lockformmf.ReleaseMutex();
            //post(lockformmf);
            //lockformmf2 = Mutex.OpenExisting("lockNum");
            //accessor2 = mmf2.CreateViewAccessor();
            //accessor3 = mmf3.CreateViewAccessor();
            //lockformmf2.ReleaseMutex();
            /*lockformmf2.WaitOne();
            accessor2 = mmf2.CreateViewAccessor();
            lockformmf2.ReleaseMutex();*/
            //CalculatePathToTake(m_nodeToPathTo);
            //node_index = -42;
        }
        private void Awake()
        {
            // get the car controller
            m_Car = GetComponent<CarController2>();
            rigid_car = GetComponent<Rigidbody>();
        }
        //called once per each update frame
        private void FixedUpdate()
        {
            if (!m_doneCalculatingPath && !m_calculatingPathToTake) //wait untill path planning is completed
            {
                CalculatePathToTake(m_nodeToPathTo);
                ListCount = 24;
                return;
            }
            if (!startMoving) //if start moving is false dont do anything
            {
                return;
            }
            ListCount = m_nodeToPathToRoute.Count; //count of nodes in the path
            //ListCount = m_nodeToPathToRoute.Count;
            // pass the input to the car!
            //float h = CrossPlatformInputManager.GetAxis("Horizontal");
            //float v = CrossPlatformInputManager.GetAxis("Vertical");
//#if !MOBILE_INPUT
            float handbrake = CrossPlatformInputManager.GetAxis("Jump"); //handbrake can be applied using space bar
            /*lockformmf.WaitOne(); //reading steering angle, total number of key points and number of lane detected
            //h = (float)accessor.ReadDouble(0);
            //accessor.WriteArray(0, ba, 0, ba.Length);
            SteerAngle = accessor.ReadDouble(0);
            int total_pts = accessor2.ReadInt32(0);
            int num_lane = accessor3.ReadInt32(0);
            lockformmf.ReleaseMutex();*/
            //Debug.Log(db);
            //ChangeLane("2");
            int total_pts = 0, num_lane = 0;
            SteerAngle = 0;
            try
            {
                //Read values from memory mapped files
                wait(lockformmf);
                SteerAngle = ReadInt(mmf, 20);
                total_pts = ReadInt(mmf2, 20);
                num_lane = ReadInt(mmf3, 20);
                /*SteerAngle = BitConverter.ToDouble(Convert.FromBase64String(readMMF(mmf, 12)), 0);
                total_pts = BitConverter.ToInt32(Convert.FromBase64String(readMMF(mmf2, 8)), 0);//Int32.Parse(readMMF(mmf2, 4));
                num_lane = BitConverter.ToInt32(Convert.FromBase64String(readMMF(mmf3, 8)), 0);//Int32.Parse(readMMF(mmf3, 4));*/
                post(lockformmf);
            }
            catch(Exception e)
            {
                return;
            }

            if(!checkTurn && takeTurn && (total_pts < 40 || num_lane%10==0 || num_lane/10==0)) //if map says to take turn and there are no lane lines
            {
                //db = 0;
                checkTurn = true;
            }
            /*if (checkTurn)
            {
           
                if(total_pts >= 40 && num_lane % 10> 0 && num_lane/10>0)
                {
                    checkTurn = false;

                }
                else
                {
                    db = 0;
                }
            }*/
            /*if (checkTurn)
            {
                
                if(node_index== m_nodeToPathToRoute.Count)
                {
                    checkTurn = false;
                    node_index = 0;
                }
                Vector3 dir = m_nodeToPathToRoute[node_index].transform.position - transform.position;
                float angle = -Vector3.SignedAngle(transform.forward, dir, transform.up);
                db = angle;
                if (dir.magnitude < 2)
                {
                    node_index++;
                }

            }*/

            /*if(!takeTurn && node_index+1 < m_nodeToPathToRoute.Count && m_nodeToPathToRoute[node_index + 1].isTurn)
            {
                
                    takeTurn = true;
                    turn_start = node_index + 1;
                    turn_end = turn_start;
                    while (m_nodeToPathToRoute[turn_end+1].isTurn)
                    {
                        turn_end++;
                    }
                
            }*/
            speed = 3.6f * vc; //update the speed in the output
            if (checkTurn) // this section si run when car is navigating on intersection, roundabouts
            {
                if (node_index > turn_end)
                {
                    checkTurn = false;
                    takeTurn = false;
                    changingLane = false;
                }
                else //in real scenarios these information will come from the inertial sensors. here we simply use coordinate systme in unity
                {
                    Vector3 dir = m_nodeToPathToRoute[node_index].transform.position - transform.position; //estimates the vector from the car to the next node in the path 
                    SteerAngle = -Vector3.SignedAngle(transform.forward, dir, transform.up); //calculates steering angle
                }
            }
            /*if (node_index < ListCount)
            {
                Vector3 dir = m_nodeToPathToRoute[node_index].transform.position - transform.position;
                float angle = -Vector3.SignedAngle(transform.forward, dir, transform.up);
                db = angle;
                if (dir.magnitude < 2)
                {
                    node_index++;
                }
            }*/
            if (node_index < m_nodeToPathToRoute.Count) //This section simulates GPS
            {
                acc_val=0.5f;
                Vector3 dir = m_nodeToPathToRoute[node_index].transform.position - transform.position;
                if (!checkTurn && dir.magnitude < 7)
                {
                    node_index++;
                    if (node_index < m_nodeToPathToRoute.Count)
                    {
                        if (m_nodeToPathToRoute[node_index].isTurn)
                        {

                            takeTurn = true;
                            turn_start = node_index;
                            turn_end = turn_start;
                            while (m_nodeToPathToRoute[turn_end].isTurn)
                            {
                                turn_end++;
                                if(turn_end>=m_nodeToPathToRoute.Count){
                                  break;
                                }
                            }
                            wait(curr_lane_lock);
                            WriteInt(m_nodeToPathToRoute[turn_end-1].m_lane, curr_lane_mmf);
                            post(curr_lane_lock);
                            
                        }
                        else if(m_nodeToPathToRoute[node_index-1].m_lane != m_nodeToPathToRoute[node_index].m_lane)
                        {
                            checkTurn = true;
                            takeTurn = true;
                            turn_end = node_index+2;
                            changingLane = true;
                        }

                    }
                }
                

                else if (dir.magnitude < 3)
                {
                    node_index++;
                }

            }
            else{
                  acc_val=0; //set acceleration to zero if reached at destination
            }
            if (SteerAngle >= 10 || SteerAngle <= -10) //if car is at turn or the steering angle is greater than the threshold steering angle then limit the velocity at turn
            {
                m_Car.m_Topspeed = 10; //v_max_turn=10
            }
            else
            {
                m_Car.m_Topspeed = TopSpeed; //normal speed limits of the car
            }
            
            Sensor(-(float)SteerAngle / 25, acc_val, handbrake); //Steering angle sent to the sensors which control the velocity, 25 in first argument is actually maximum steer angle
//#else       
         //   m_Car.Move(h, v, v, 0f);
//##endif
        }
       
        private void OnApplicationQuit()
        {
            //post(lockformmf);
            reset();
            //unlinkSHM("steerAngle");
            //unlinkSHM("total_pts");
            //unlinkSHM("numLane");
            //accessor.Dispose();
            // stream.Dispose();
            //mmf.Dispose();
            //mmf2.Dispose();
            //lockformmf.Dispose();
            //lockformmf2.Dispose();
        }
        /*private void Update()
        {
            lockformmf.WaitOne();
            //h = (float)accessor.ReadDouble(0);
            //accessor.WriteArray(0, ba, 0, ba.Length);
            double db=accessor.ReadDouble(0);
            lockformmf.ReleaseMutex();
            Debug.Log(db);
        }*/
        //v is fractional motor torque, h is fractional steering angle
       
        void Sensor(float h, float v, float handbrake)
        {
            
            isbreak = false; //set isBrakefalse
            float fb = 0; //does same work as isBreak

            if (Vector3.Angle(rigid_car.velocity, transform.forward) < 50f && Vector3.Magnitude(rigid_car.velocity) > 0.01f) //applicable if car is moving forward
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
                Vector3 rc = rigid_car.transform.position; //position vector of the car, in reality this is not needed. This is needed only for simulation purpose 
                vc = Vector3.Magnitude(rigid_car.velocity); //speed of car in units/second
                //shortSensorLength = (float)(2500 * vc * vc / (m_Car.m_BrakeTorque) + 0.5);
                float temp = Vector3.Magnitude(Vector3.Project(rigid_car.velocity, Quaternion.AngleAxis(sideSensorAngle, transform.up) * transform.forward));
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
                    if (Math.Abs(h) > 0.5) //steering angle should be large enough to get the angled sensors working
                    {
                        float rd = (d - prevLidar[0]) / Time.deltaTime; //rate of change od lidar reading
                        float vother = rd + temp; //calculation of vother for angled sensor
                        if (vother >= 0) //if other vehicle has a positive velocity then calcculate ssafe
                        {
                            shortSensorLength = (float)(1250* vc * vc / (m_Car.m_BrakeTorque) + 0.5);
                        }
                        else
                        {
                            shortSensorLength = (float)(1250 * rd * rd / (m_Car.m_BrakeTorque) + 0.5);
                        }
                    }
                    else //if car is not turning take ssafe=so
                    {
                        shortSensorLength = 0.5f;
                    }
                    if (d < shortSensorLength) //if lidar reading is less thatn ssafe then brake
                    {
                         fb = -2f;
                         isbreak = true;
                         v=0;
                    }
                    prevLidar[0] = d;
                    
                   
                    Debug.DrawLine(sensorPos_right, sideright.point, Color.red); //visualization sensor in scene mode
                }
                if (b_sideLeft && !sideleft.collider.CompareTag("Terrain"))
                {
                    float d = Vector3.Magnitude(sideleft.point - sensorPos_left);
                    if (Math.Abs(h) > 0.5)
                    {
                        float rd = (d - prevLidar[1]) / Time.deltaTime;
                        float vother = rd + temp;
                        if (vother >= 0)
                        {
                            shortSensorLength = (float)(1250 * vc * vc / (m_Car.m_BrakeTorque) + 0.5);
                        }
                        else
                        {
                            shortSensorLength = (float)(1250 * rd * rd / (m_Car.m_BrakeTorque) + 0.5);
                        }
                    }
                    else
                    {
                        shortSensorLength = 0.5f;
                    }
                    if (d < shortSensorLength)
                    {
                        fb = -2f;
                        v=0;
                        isbreak = true;
                    }
                    prevLidar[1] = d;
                    
                   
                    Debug.DrawLine(sensorPos_left, sideleft.point, Color.black);
                }
                if (b_mid && !hit_mid.collider.CompareTag("Terrain"))
                {
                    float d = Vector3.Magnitude(hit_mid.point - sensorPos_mid);
                    float rd = (d - prevLidar[2]) / Time.deltaTime;
                    float vother = rd + vc;
                    if (vother >= 0)
                    {
                        shortSensorLength = (float)(2500 * vc * vc / (m_Car.m_BrakeTorque) + 0.5);
                    }
                    else
                    {
                        shortSensorLength = (float)(2500 * rd * rd / (m_Car.m_BrakeTorque) + 0.5);
                    }
                    if (d < shortSensorLength)
                    {
                        fb = -2f;
                        v=0;
                        isbreak = true;
                    }
       
                    prevLidar[2] = d;
                    //v = -(float)Math.Pow((Vector3.Magnitude(rigid_car.velocity)/m_Car.m_Topspeed), 2);
                    
                    //handbrake = 2f;

                    Debug.DrawLine(sensorPos_mid, hit_mid.point);
                }
                if (b_left && !hit_left.collider.CompareTag("Terrain"))
                {
                    float d = Vector3.Magnitude(hit_left.point - sensorPos_left);
                    float rd = (d - prevLidar[3]) / Time.deltaTime;
                    float vother = rd + vc;
                    if (vother >= 0)
                    {
                        shortSensorLength = (float)(2500 * vc * vc / (m_Car.m_BrakeTorque) + 0.5);
                    }
                    else
                    {
                        shortSensorLength = (float)(2500 * rd * rd / (m_Car.m_BrakeTorque) + 0.5);
                    }
                    if (d < shortSensorLength)
                    {
                        fb = -2f;
                        v=0;
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
                    float vother = rd + vc;
                    if (vother >= 0)
                    {
                        shortSensorLength = (float)(2500 * vc * vc / (m_Car.m_BrakeTorque) + 0.5);
                    }
                    else
                    {
                        shortSensorLength = (float)(2500 * rd * rd / (m_Car.m_BrakeTorque) + 0.5);
                    }
                    if (d < shortSensorLength)
                    {
                        fb = -2f;
                        v=0;
                        isbreak = true;
                    }

                    prevLidar[4] = d;

                    //handbrake = 2f;

                    //handbrake = 2f;
                    Debug.DrawLine(sensorPos_right, hit_right.point, Color.red);
                }
                /*if(isLaneChange && !lane_change.collider.CompareTag("Terrain"))
                {
                    lockformmf2.WaitOne();
                    int num_left = accessor2.ReadInt32(0);
                    int num_right = accessor3.ReadInt32(0);
                    lockformmf2.ReleaseMutex();
                    if (num_right > 1)
                    {
                        shortSideSensorLength =(float)( 1.5 * sideLaneThresh);
                        b_sideRight = Physics.Raycast(sensorPos_right, Quaternion.AngleAxis(sideSensorAngle, transform.up) * transform.forward, out sideright, shortSideSensorLength);
                        if (b_rightwall && !right_wall.collider.CompareTag("Terrain") && b_sideRight && !sideright.collider.CompareTag("Terrain"))
                        {
                            h = 0.4f;
                        }
                    }

                    Debug.DrawLine(sensorPos_mid, lane_change.point, Color.blue);
                }*/
                if(changingLane && b_leftwall && !left_wall.collider.CompareTag("Terrain"))  //brakig while lane changing usig side sensor
                {
                    fb = -2f;
                    v=0;
                    isbreak = true;
                    Debug.DrawLine(wallsense, left_wall.point, Color.red);
                }
                if (changingLane &&  b_rightwall && !right_wall.collider.CompareTag("Terrain")) //brakig while lane changing usig side sensor
                {
                    fb = -2f;
                    v=0;
                    isbreak = true;
                    Debug.DrawLine(wallsenseR, right_wall.point, Color.red);
                }
            }
            else if (Vector3.Magnitude(rigid_car.velocity) > 0.01f) //apply same kind of logic for moving back. I did not do much here but one can adjust this for back sensor. Since the car in the scene is not moving back (current map structure does not allow that) so not much emphasis was not paid here.
            {
                RaycastHit hit_mid, hit_left, hit_right, sideleft, sideright;
                frontSensorPos.z = -frontSensorPos.z;
                sidesensorpos.z = -sidesensorpos.z;
                Vector3 sensorPos_mid = transform.TransformPoint(frontSensorPos);
                Vector3 sensorPos_right = transform.TransformPoint(sidesensorpos);
                sidesensorpos.x = -sidesensorpos.x;
                Vector3 sensorPos_left = transform.TransformPoint(sidesensorpos);
                sidesensorpos.x = -sidesensorpos.x;
                shortSensorLength = (float)(5000 * Vector3.Magnitude(rigid_car.velocity) * Vector3.Magnitude(rigid_car.velocity) / (m_Car.m_BrakeTorque) + 1);
                Debug.Log(shortSensorLength);
                frontSensorPos.z = -frontSensorPos.z;
                sidesensorpos.z = -sidesensorpos.z;
                float temp = Vector3.Magnitude(Vector3.Project(rigid_car.velocity, Quaternion.AngleAxis(sideSensorAngle, transform.up) * transform.forward));
                float shortSideSensorLength = (float)(5000 * temp * temp / (m_Car.m_BrakeTorque) + 1);
                bool b_mid = Physics.Raycast(sensorPos_mid, -transform.forward, out hit_mid, shortSensorLength);
                bool b_left = Physics.Raycast(sensorPos_left, -transform.forward, out hit_left, shortSensorLength);
                bool b_right = Physics.Raycast(sensorPos_right, -transform.forward, out hit_right, shortSensorLength);
                bool b_sideRight = Physics.Raycast(sensorPos_right, Quaternion.AngleAxis(sideSensorAngle, -transform.up) * -transform.forward, out sideright, shortSideSensorLength);
                bool b_sideLeft = Physics.Raycast(sensorPos_left, Quaternion.AngleAxis(-sideSensorAngle, -transform.up) * -transform.forward, out sideleft, shortSideSensorLength);
                if (b_left && !hit_left.collider.CompareTag("Terrain"))
                {
                    fb = -2f;
                    v=0;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_left, hit_left.point, Color.red);
                }
                if (b_mid && !hit_mid.collider.CompareTag("Terrain"))
                {
                    fb = -2f;
                    v=0;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_mid, hit_mid.point, Color.white);
                }
                if (b_right && !hit_right.collider.CompareTag("Terrain"))
                {
                    fb = -2f;
                    v=0;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_right, hit_right.point, Color.black);
                }
                if (b_sideRight && !sideright.collider.CompareTag("Terrain"))
                {
                    fb = -2f;
                    v=0;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_right, sideright.point);
                }
                if (b_sideLeft && !sideleft.collider.CompareTag("Terrain"))
                {
                    fb = -2f;
                    v=0;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_left, sideleft.point);
                }
                

            }
            
            
            m_Car.Move(h, v, fb, handbrake, isbreak); //Giving the inputs to the car,fractional steering angle=h, v=fractional motor torque, handbrake and isbrake
        }
    }
}

/*using System;
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController2))]
    public class ACC : MonoBehaviour
    {
        private CarController2 m_Car; // the car controller we want to use


        private void Awake()
        {
            // get the car controller
            m_Car = GetComponent<CarController2>();
        }


        private void FixedUpdate()
        {
            // pass the input to the car!
            float h = CrossPlatformInputManager.GetAxis("Horizontal");
            float v = CrossPlatformInputManager.GetAxis("Vertical");
#if !MOBILE_INPUT
            float handbrake = CrossPlatformInputManager.GetAxis("Jump");
            m_Car.Move(h, v, v, handbrake, false);
#else
            m_Car.Move(h, v, v, 0f);
#endif
        }
    }
}
*/
