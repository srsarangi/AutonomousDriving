using System;
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class ACC : MonoBehaviour
    {
        private CarController2 m_Car; // the car controller we want to use
        private Rigidbody rigid_car;
        [Header("Sensors")]
        public Vector3 frontSensorPos = new Vector3(0f, 0f, 3f);
        public Vector3 sidesensorpos = new Vector3(1f, 0f, 3f);
        public float longSensorLength = 200f;
        public float shortSensorLength = 20f;
        // private Vector3 prevPos=new Vector3(0f,0f, 0f);
        private Vector3[] prevpos = new Vector3[5];
        float[] back_sensorlength = new float[5];
        private bool isShortSensorActive = false;
        public float TopSpeed=20f;
        public float sideSensorAngle=25f;
       // private static float time = 0f;
        private void Awake()
        {
            // get the car controller
            m_Car = GetComponent<CarController2>();
            rigid_car = GetComponent<Rigidbody>();
        }


        private void FixedUpdate()
        {
            // pass the input to the car!
            float h = CrossPlatformInputManager.GetAxis("Horizontal");
            float v = CrossPlatformInputManager.GetAxis("Vertical");
#if !MOBILE_INPUT
            float handbrake = CrossPlatformInputManager.GetAxis("Jump");
            Sensor(h, v, handbrake);
#else
            m_Car.Move(h, v, v, 0f);
#endif
        }
        void Sensor(float h, float v, float handbrake)
        {
            isShortSensorActive = false;
            bool isbreak=false;
            float fb = 0;
            Debug.Log(h);

            if (Vector3.Angle(rigid_car.velocity, transform.forward) < 50f && Vector3.Magnitude(rigid_car.velocity) > 0.01f)
            {
                RaycastHit hit_mid, hit_left, hit_right, sideleft, sideright;
                Vector3 sensorPos_mid = transform.TransformPoint(frontSensorPos);
                Vector3 sensorPos_right = transform.TransformPoint(sidesensorpos);
                sidesensorpos.x = -sidesensorpos.x;
                Vector3 sensorPos_left = transform.TransformPoint(sidesensorpos);
                sidesensorpos.x = -sidesensorpos.x;
                shortSensorLength = (float)(2500 * Vector3.Magnitude(rigid_car.velocity) * Vector3.Magnitude(rigid_car.velocity) / (m_Car.m_BrakeTorque) + 0.5);
                Debug.Log(shortSensorLength);
                Quaternion quat = Quaternion.AngleAxis(sideSensorAngle, transform.up);
                float temp = Vector3.Magnitude(Vector3.Project(rigid_car.velocity, quat * transform.forward));

                float shortSideSensorLength = (float)(2500 * temp * temp / (m_Car.m_BrakeTorque) + 1);
                //shortSideSensorLength = 20f;
                bool b_mid = Physics.Raycast(sensorPos_mid, transform.forward, out hit_mid, shortSensorLength);
                bool b_left = Physics.Raycast(sensorPos_left, transform.forward, out hit_left, shortSensorLength);
                bool b_right = Physics.Raycast(sensorPos_right, transform.forward, out hit_right, shortSensorLength);
                bool b_sideRight = Physics.Raycast(sensorPos_right, quat * transform.forward, out sideright, shortSideSensorLength);
                bool b_sideLeft = Physics.Raycast(sensorPos_left, Quaternion.AngleAxis(-sideSensorAngle, transform.up) * transform.forward, out sideleft, shortSideSensorLength);
                if (b_sideRight && !sideright.collider.CompareTag("Terrain"))
                {

                        fb = -2f;
                        isbreak = true;
                    
                    Debug.DrawLine(sensorPos_right, sideright.point);
                }
                if (b_sideLeft && !sideleft.collider.CompareTag("Terrain"))
                {
                    
                        fb = -2f;
                        isbreak = true;
                    
                    Debug.DrawLine(sensorPos_left, sideleft.point);
                }
                if (b_mid && !hit_mid.collider.CompareTag("Terrain"))
                {
                    //isShortSensorActive = true;
                   
                        //v = -(float)Math.Pow((Vector3.Magnitude(rigid_car.velocity)/m_Car.m_Topspeed), 2);
                        fb = -2f;
                        isbreak = true;
                        //handbrake = 2f;
                    
                    Debug.DrawLine(sensorPos_mid, hit_mid.point);
                }
                if (b_left && !hit_left.collider.CompareTag("Terrain"))
                {
                    isShortSensorActive = true;
                   
                        isbreak = true;
                        fb = -2f;
                        //handbrake = 2f;
                    

                    Debug.DrawLine(sensorPos_left, hit_left.point, Color.black);
                }
                if (b_right && !hit_right.collider.CompareTag("Terrain"))
                {
                    isShortSensorActive = true;
                    
                        fb = -2f;
                        isbreak = true;
                        //handbrake = 2f;
                    
                    //handbrake = 2f;
                    Debug.DrawLine(sensorPos_right, hit_right.point, Color.red);
                }
            }
            else if(Vector3.Magnitude(rigid_car.velocity) > 0.01f)
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
                    isbreak = true;
                    Debug.DrawLine(sensorPos_left, hit_left.point, Color.red);
                }
                if (b_mid && !hit_mid.collider.CompareTag("Terrain"))
                {
                    fb = -2f;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_mid, hit_mid.point, Color.white);
                }
                if (b_right && !hit_right.collider.CompareTag("Terrain"))
                {
                    fb = -2f;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_right, hit_right.point, Color.black);
                }
                if(b_sideRight && !sideright.collider.CompareTag("Terrain"))
                {
                    fb = -2f;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_right, sideright.point);
                }
                if(b_sideLeft && !sideleft.collider.CompareTag("Terrain"))
                {
                    fb = -2f;
                    isbreak = true;
                    Debug.DrawLine(sensorPos_left, sideleft.point);
                }

            }
            Debug.Log(v);
           
            m_Car.Move(h, v, fb, handbrake, isbreak);
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