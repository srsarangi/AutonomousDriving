

//Importing libraries
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;
using UnityEngine.UI;

public class my_car : MonoBehaviour
{
    // Marker is used to visualize the next point to be followed
    public GameObject Marker;
	
	// Text to be displayed on screen for information
    public Text cmd;
    public Text speed;
	
	//Path 
	[Header("Path")]
    public Transform path_left;
    private List<Transform> left_nodes;
    public Transform path_right;
    private List<Transform> right_nodes;
    public int currentNode = 0;
	
	//Wheel Attributes
    public float maxSteerAngle = 45f;
    public float turnSpeed = 500f;
    public WheelCollider wheelFL;
    public WheelCollider wheelFR;
    public WheelCollider wheelRL;
    public WheelCollider wheelRR;
    public float maxMotorTorque = 0f;
    public float maxBrakeTorque = 2000f;
    public float currentSpeed = 0f;
    public float maxSpeed;
    public bool isBraking = false;
    public float targetSteerAngle = 0;
	
	//Car center of mass
	public Vector3 centerOfMass;
	
	//Sensor Attributes
    [Header("Sensors")]
    public float sensorLength = 30f;
    public Vector3 frontSensorPosition = new Vector3(0f,0.5f,1f);
    public float frontSideSensorPosition = 1f;
    public float frontSensorAngle = 25f;
    
	//Lane Related Information
    public bool laneChanged = false;
    public bool lane = false;
    public bool dont_accelerate = false;
	
	//File read
    public FileInfo theSourceFile = null;
    public StreamReader reader = null;
    public string text = " "; 

    //Initialise acceleration and preceeding car distance
    public float acceleration = 0.0f;
    public float preceeding_car_distance = 0.0f;

    private Rigidbody carRigidBody;
    
    private void Start()
    {
       /* theSourceFile = new FileInfo("command.txt");
        reader = theSourceFile.OpenText();*/
		
		//Assigning center of mass to the rigid body
        GetComponent<Rigidbody>().centerOfMass = centerOfMass;
		
		//Adding nodes to the path -left and right
        Transform[] pathTransforms_left = path_left.GetComponentsInChildren<Transform>();
        left_nodes = new List<Transform>();

        for (int i = 0; i < pathTransforms_left.Length; i++)
        {

            if (pathTransforms_left[i] != path_left.transform)
            {
                left_nodes.Add(pathTransforms_left[i]);
            }
        }
        Transform[] pathTransforms_right = path_right.GetComponentsInChildren<Transform>();
        right_nodes = new List<Transform>();

        for (int i = 0; i < pathTransforms_right.Length; i++)
        {

            if (pathTransforms_right[i] != path_right.transform)
            {
                right_nodes.Add(pathTransforms_right[i]);
            }
        }

        carRigidBody = GetComponent<Rigidbody>();
    }
    
    private void FixedUpdate()
    {
	    //lane equal to false for left lane and true for right lane
		//Following if-else block assigns the position to the marker
		//Left_path will be used for left lane and right path path will be used for right lane
        
        // removing showed marker for now
        // if(lane==false)
        // {
        //     Marker.transform.position = left_nodes[currentNode].position;
        // }
        // else
        // {
        //     Marker.transform.position = right_nodes[currentNode].position;
        // }
        
		
		//Display Current Speed on the screen
        speed.text = "Current Speed: " + currentSpeed + "m/s";

        /*if (text != null)
        {
            text = reader.ReadLine();
            //Console.WriteLine(text);
            //Debug.Log(text);
            cmd.text = text;
        }
        else
        {
            cmd.text = "No Command ";
            isBraking = true;
            Braking();
        }
       */
	   
	    //Following functions are called with each updation of the frames
        Sensors();
        Braking();
        ApplySteer();
        Drive();
        CheckWayPointDistance();
        LerpToSteerAngle();
       
    }
    void Braking()
    {
	
	    //Calculate the maximum brake torque
        maxBrakeTorque=1000*currentSpeed*wheelFL.radius+Time.fixedDeltaTime;
		
		//Apply brake torque if isBraking is true
		//Otherwise make brake torque equal to zero
		//Brake Torque has been applied to the rear wheels
        if(isBraking)
        {
            // Debug.Log("Braking ");
            wheelRL.brakeTorque = maxBrakeTorque;
            wheelRR.brakeTorque = maxBrakeTorque;
        }
        else
        {
            wheelRL.brakeTorque = 0;
            wheelRR.brakeTorque = 0;
        }
       
    }
   
    private void OnDrawGizmos()
    {
	
	    //Display sensors in scene view only
        if (transform.tag == "Player")
        {
            //forward sensors
            Gizmos.DrawRay(transform.position+new Vector3(0.0f, 0.5f, 0.0f), transform.forward * (sensorLength + transform.localScale.z));
            Gizmos.DrawRay(transform.position + new Vector3(0.0f, 0.5f, 0.0f), Quaternion.AngleAxis(-20, transform.up) * transform.forward * (sensorLength + transform.localScale.z));
            Gizmos.DrawRay(transform.position + new Vector3(0.0f, 0.5f, 0.0f), Quaternion.AngleAxis(20, transform.up) * transform.forward * (sensorLength + transform.localScale.z));
            Gizmos.DrawRay(transform.position + new Vector3(0.0f, 0.5f, 0.0f), Quaternion.AngleAxis(-10, transform.up) * transform.forward * (sensorLength + transform.localScale.z));
            Gizmos.DrawRay(transform.position + new Vector3(0.0f, 0.5f, 0.0f), Quaternion.AngleAxis(10, transform.up) * transform.forward * (sensorLength + transform.localScale.z));
            Gizmos.DrawRay(transform.position + new Vector3(0.0f, 0.5f, 0.0f), Quaternion.AngleAxis(-30, transform.up) * transform.forward * (sensorLength + transform.localScale.z));
            Gizmos.DrawRay(transform.position + new Vector3(0.0f, 0.5f, 0.0f), Quaternion.AngleAxis(30, transform.up) * transform.forward * (sensorLength + transform.localScale.z));
            //back sensors
            Gizmos.DrawRay(transform.position + new Vector3(0.0f, 0.5f, 0.0f), -transform.forward * (sensorLength + transform.localScale.z));
            Gizmos.DrawRay(transform.position + new Vector3(0.0f, 0.5f, 0.0f), Quaternion.AngleAxis(-frontSensorAngle, transform.up) * -transform.forward * (sensorLength + transform.localScale.z));
            Gizmos.DrawRay(transform.position + new Vector3(0.0f, 0.5f, 0.0f), Quaternion.AngleAxis(frontSensorAngle, transform.up) * -transform.forward * (sensorLength + transform.localScale.z));
           //right sensors
            Gizmos.DrawRay(transform.position + new Vector3(0.0f, 0.5f, 0.0f), transform.right * (sensorLength + transform.localScale.x));
            Gizmos.DrawRay(transform.position + new Vector3(0f, 0.5f, 1f), transform.right * (sensorLength + transform.localScale.x));
            Gizmos.DrawRay(transform.position + new Vector3(0f, 0.5f, -1f), transform.right * (sensorLength + transform.localScale.x));
            //left sensors
            Gizmos.DrawRay(transform.position + new Vector3(0.0f, 0.5f, 0.0f), -transform.right * (sensorLength + transform.localScale.x));
            Gizmos.DrawRay(transform.position+ new Vector3(0f,0.5f,1f), -transform.right * (sensorLength + transform.localScale.x));
            Gizmos.DrawRay(transform.position+ new Vector3(0f, 0.5f, -1f) , -transform.right * (sensorLength + transform.localScale.x));
        }
            
    }
    private void Sensors()
    {
	    //initialising the sensor output values with a maximum value
        float left = 10.0f;
        float right = 10.0f;
        float front = 10.0f;
        float front_left10 = 10.0f;
        float front_left20 = 10.0f;
        float front_left30 = 10.0f;
        float front_right10 = 10.0f;
        float front_right20 = 10.0f;
        float front_right30 = 10.0f;
        float back = 10.0f;

		//initialising safe distance
        double forward_safe_distance = 1f;
        float sides_safe_distance = 0.5f;

        //defining the sensors
        RaycastHit hit;
        Vector3 sensorStartPos = transform.position + new Vector3(0.0f, 0.3f, 0.0f);
        //Vector3 sensorStartPos = transform.position + new Vector3(0.0f, 0.5f, 0.0f);
        #region sensors

        #region right_sensors
        //right sensor- three right sensors are used
        //right is initialised to the minimum of the three right sensor values (for simplification only)

        if (Physics.Raycast(sensorStartPos,transform.right,out hit,(sensorLength + transform.localScale.x)))
        {
		    //Terrain tag is used to avoid terrain objects 
            if(hit.transform.tag!="Terrain")
            {
                right = hit.distance;
            }
        }
        Vector3 rightSideSensorPos = sensorStartPos + new Vector3(0f, 0f, 1f);
        if (Physics.Raycast(rightSideSensorPos, transform.right, out hit, (sensorLength + transform.localScale.x)))
        {
            if (hit.transform.tag != "Terrain")
            {
                if(right>hit.distance)
                {
                    right = hit.distance;
                }
            }
        }
        rightSideSensorPos = sensorStartPos + new Vector3(0f, 0f, -1f);
        if (Physics.Raycast(rightSideSensorPos, transform.right, out hit, (sensorLength + transform.localScale.x)))
        {
            if (hit.transform.tag != "Terrain")
            {
                if (right > hit.distance)
                {
                    right = hit.distance;
                }
            }
        }
		#endregion
        
		#region left_sensors
		//left sensor- three left sensors are used
		//left is initialised to the minimum of the three left sensor values (for simplification only)
        if (Physics.Raycast(sensorStartPos, -transform.right, out hit, (sensorLength + transform.localScale.x)))
        {
            if (hit.transform.tag != "Terrain")
            {
                left = hit.distance;
            }
        }
        Vector3 leftSideSensorPos = sensorStartPos + new Vector3(0f, 0f, 1f);
        if (Physics.Raycast(leftSideSensorPos, -transform.right, out hit, (sensorLength + transform.localScale.x)))
        {
            if (hit.transform.tag != "Terrain")
            {
                if (left > hit.distance)
                {
                    left = hit.distance;
                }
            }
        }
        leftSideSensorPos = sensorStartPos + new Vector3(0f, 0f, -1f);
        if (Physics.Raycast(leftSideSensorPos, -transform.right, out hit, (sensorLength + transform.localScale.x)))
        {
            if (hit.transform.tag != "Terrain")
            {
                if (left > hit.distance)
                {
                    left = hit.distance;
                }
            }
        }
		#endregion
        
		#region front_sensors
		
		//front sensor- seven front sensors 
		//(middle, 
		//at 10 degree on left and right side of the middle 
		//at 20 degree on left and right side of the middle 
		//at 30 degree on left and right side of the middle )
        if (Physics.Raycast(sensorStartPos, transform.forward, out hit, (sensorLength + transform.localScale.z)))
        {
            if (hit.transform.tag != "Terrain")
            {
                front = hit.distance;

            }
        }
        if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(-10, transform.up) * transform.forward, out hit, (sensorLength + transform.localScale.z)))
        {
            if (hit.transform.tag != "Terrain")
            {
                front_left10 = hit.distance;
            }
        }
        if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(10, transform.up) * transform.forward, out hit, (sensorLength + transform.localScale.z)))
        {

            if (hit.transform.tag != "Terrain")
            {
                front_right10 = hit.distance;
            }

        }
        if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(-20, transform.up) * transform.forward, out hit, (sensorLength + transform.localScale.z)))
        {
            if (hit.transform.tag != "Terrain")
            {
                front_left20 = hit.distance;
            }
        }
        if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(20, transform.up) * transform.forward, out hit, (sensorLength + transform.localScale.z)))
        {

            if (hit.transform.tag != "Terrain")
            {
                front_right20 = hit.distance;
            }

        }
        if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(-30, transform.up) * transform.forward, out hit, (sensorLength + transform.localScale.z)))
        {
            if (hit.transform.tag != "Terrain")
            {
                front_left30 = hit.distance;
            }
        }
        if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(30, transform.up) * transform.forward, out hit, (sensorLength + transform.localScale.z)))
        {

            if (hit.transform.tag != "Terrain")
            {
                front_right30 = hit.distance;
            }

        }
		#endregion
        
		#region back_sensors
		//back sensor -three back sensors ,two for the blind-spots
        if (Physics.Raycast(sensorStartPos, -transform.forward, out hit, (sensorLength + transform.localScale.z)))
        {
            if (hit.transform.tag != "Terrain")
            {
                back = hit.distance;
            }
        }
        if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(-frontSensorAngle, transform.up) * -transform.forward, out hit, (sensorLength + transform.localScale.z)))
        {
            if (hit.transform.tag != "Terrain")
            {
                if (hit.distance < back)
                {
                    back = hit.distance;
                }
            }
        }
        if (Physics.Raycast(sensorStartPos, Quaternion.AngleAxis(frontSensorAngle, transform.up) * -transform.forward, out hit, (sensorLength + transform.localScale.z)))
        {
            if (hit.transform.tag != "Terrain")
            {
                if (hit.distance < back)
                {
                    back = hit.distance;
                }
            }

        }
        #endregion

        #endregion

        //Calculate forward safe distance using total braking distance equation

        forward_safe_distance = (currentSpeed * currentSpeed) / (2 * 0.1 * 9.8) + currentSpeed * Time.fixedDeltaTime + Time.fixedDeltaTime;
        //forward_safe_distance = (currentSpeed * currentSpeed) / (2 * 0.1 * 9.8);


        //Let vehicles pass from the sides
        if (right<sides_safe_distance)
        {
            targetSteerAngle = -maxSteerAngle/2;
            LerpToSteerAngle();
            dont_accelerate = true;
        }
        if (left < sides_safe_distance)
        {
            targetSteerAngle = maxSteerAngle / 2;
            LerpToSteerAngle();
            dont_accelerate = true;
        }

        #region braking_check

        //Apply brakes if front is less than the forward safe distance + side safe distance 

        //Debug.Log("front: " + front.ToString("R"));
        //Debug.Log("forward: " + forward_safe_distance.ToString("R"));
        //Debug.Log("side_safe: " + sides_safe_distance.ToString("R"));

        //if (front <= forward_safe_distance + sides_safe_distance && front != 10.0f)
        if (front <= forward_safe_distance && front != 10.0f)
        {
            dont_accelerate = true;
            isBraking = true;
        }
        else
        {
            dont_accelerate = false;
            isBraking = false;
        }

        Braking();
        #endregion

        #region turn_check
        //Slowing down at the turns 
        //if (currentNode == 2 || currentNode == 4 || currentNode == 5 || currentNode == 6 )
        //      {
        //          if ((lane == true && Vector3.Distance(transform.position, right_nodes[currentNode].position) < 5f) ||
        //              (lane == false && Vector3.Distance(transform.position, left_nodes[currentNode].position) < 5f))
        //          {
        //              dont_accelerate = true;
        //          }
        //      }
        #endregion

        #region lane_change
        if (lane == false)
            {
                if (front_right10 > front && front_right20>front && front_right30>front  && right>sides_safe_distance)
                {
                    lane = true;
                    laneChanged = true;
                }
            }
            else
            {
                
                if (front_left10 > front && front_left20 > front && front_left30 > front && left>sides_safe_distance)
                {
                    lane = false;
                    laneChanged = true;   
                }
        }
		#endregion
        
		#region accelerate
		//Calculate acceleration using GM Model
		if (!dont_accelerate && !isBraking && currentSpeed < maxSpeed)
        {
            float preceeding_car_velocity = (preceeding_car_distance - front) / Time.deltaTime;
            acceleration = Mathf.Abs(13 * (currentSpeed - preceeding_car_velocity) / front);
        }
        else
        {
            acceleration = 0;
        }
		#endregion
		
        preceeding_car_distance = front;
    }
    private void ApplySteer()
    {
        //Find the steer angle on the basis of current position of the vehicle and current node of the path being followed
		
		//for left lane
        if(lane==false)
        {
            Vector3 relativeVector = transform.InverseTransformPoint(left_nodes[currentNode].position);
            float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;
            targetSteerAngle = newSteer;

        }
		//for right lane
        else
        {
            Vector3 relativeVector = transform.InverseTransformPoint(right_nodes[currentNode].position);
            float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;
            targetSteerAngle = newSteer;

        }
    }
    private void Drive()
    {
        //Find the motor torque needed if current speed is less than maximum speed limit
        currentSpeed = carRigidBody.velocity.magnitude ;
		maxMotorTorque = 1000 * acceleration * wheelFL.radius;

        //motor torque is increased by 1.5 times if travelling on a bridge

        //bool isBridge=true;
        /*if(isBridge)
		{
		    maxMotorTorque=1.5f*maxMotorTorque;
		}*/

        //Motor torque is applied on the front wheels
        if (currentSpeed < maxSpeed && !isBraking && !dont_accelerate)
        {
            wheelFL.motorTorque = maxMotorTorque;
            wheelFR.motorTorque = maxMotorTorque;
        }
        else
        {
            wheelFL.motorTorque = 0;
            wheelFR.motorTorque = 0;
        }

        Debug.Log("currentSpeed: " + currentSpeed.ToString("R"));
        Debug.Log("maxSpeed: " + maxSpeed.ToString("R"));
        Debug.Log("Accel: " + acceleration.ToString("R"));
        Debug.Log("FL: " + wheelFL.motorTorque.ToString("R"));
        Debug.Log("FR: " + wheelFR.motorTorque.ToString("R"));

    }
    private void CheckWayPointDistance()
    {
        //Calculate next way-point, Update current node 
        //when the distance of the vehicle to the current node is less than the maximum distance to goal
        float maxDistanceToGoal = 0f;
        if (laneChanged)
        {
            //This is to compensate the difference in distances 
            //for same lane and different lane
            maxDistanceToGoal = 6f;
        }
        else
        {
            maxDistanceToGoal = 4f;
        }
        //for left lane
        if (lane==false)
        {
            if (Vector3.Distance(transform.position, left_nodes[currentNode].position) < maxDistanceToGoal)
            {

                if (currentNode == left_nodes.Count - 1)
                {
                    currentNode = 0;
                }
                else
                {
                    currentNode++;
                }
            }
        }
		//for right lane
        else
        {
            if (Vector3.Distance(transform.position, right_nodes[currentNode].position) < maxDistanceToGoal)
            {

                if (currentNode == right_nodes.Count - 1)
                {
                    currentNode = 0;
                }
                else
                {
                    currentNode++;
                }
            }
        }
		//To ensure slowing down at the turns
		//set dont_accelerate flag to true;
        if(currentNode==2 || currentNode==4 || currentNode ==5 || currentNode==6 )
        {
            if((lane ==true && Vector3.Distance(transform.position, right_nodes[currentNode].position)<5f)|| 
                (lane == false && Vector3.Distance(transform.position, left_nodes[currentNode].position) < 5f))
                {
                dont_accelerate = true;
                }
           
        }
      

    }
    private void LerpToSteerAngle()
    {
	    //To ensure smooth steering
        wheelFL.steerAngle = Mathf.Lerp(wheelFL.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
        wheelFR.steerAngle = Mathf.Lerp(wheelFR.steerAngle, targetSteerAngle, Time.deltaTime * turnSpeed);
    }
    private void OnCollisionEnter(Collision collision)
    {
	//Display collision on console
        Debug.Log("Collision");
    }

}
