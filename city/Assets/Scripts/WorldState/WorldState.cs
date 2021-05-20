using System.Collections.Generic;
using UnityEngine;


namespace CarWorld
{
    public class WorldState: MonoBehaviour
    {
        #region  Singelton
        public static WorldState _instance;
        public static WorldState Instance { get { return _instance; } }
        private void SetupSingelton()
        {
            if (_instance != null)
            {
                Debug.LogError("Error in settings. Multiple singeltons exists: ");
            }
            else
            {
                _instance = this;
            }
        }
        #endregion

        [field: SerializeField]
        public float SteerAngle { get; set; }

        [field: SerializeField]
        public int TotalNumberOfPoints { get; set; }

        [field: SerializeField]
        public int LeftLanes { get; set; }

        [field: SerializeField]
        public int RightLanes { get; set; }
        
        public int MyLane { get; set; }

        [field: SerializeField]
        public char TrafficLightColor { get; set; }

        public BBox[] CarBBoxes { get; set; }
        
        public BBox[] BikeBBoxes { get; set; }
        
        public BBox[] PedestrianBBoxes { get; set; }
        
        private void Awake()
        {
            SetupSingelton();
            WorldState.Instance.SteerAngle = 0;
            WorldState.Instance.TotalNumberOfPoints = 0;
            WorldState.Instance.LeftLanes = 0;
            WorldState.Instance.RightLanes = 0;
            WorldState.Instance.TrafficLightColor = 'u';
            WorldState.Instance.CarBBoxes = new BBox[] { };
            WorldState.Instance.BikeBBoxes = new BBox[] { };
            WorldState.Instance.PedestrianBBoxes = new BBox[] { };
        }

        // private static void updateObjects(Dictionary<string, List<List<float>>> json)
        // {
        //     BBox[] carBoxes = new BBox[json["c"].Count] { };
        //     foreach (var l in json["c"])
        //     {
        //         float x = l[0], y = l[1], z = l[2];
        //         float dx = l[3], dy = l[4], dz = l[5];
        //         float direction = l[7];
        //         BBox b = new BBox(x, y, z, dx, dy, dz, direction);
        //         
        //     }
        // }

        public void ChangeLane(char direction)
        {
            // Move left
            if (direction == 'l')
            {
                
            }
            // Move right
            else
            {
                
            }
        }
        
        public class BBox
        {
            public BBox(float x, float y, float z, float dx, float dy, float dz, float direction)
            {
                this.Center = new Vector3(x, y, z);
                this.Length = dx * 2;
                this.Breadth = dy * 2;
                this.Height = dz * 2;
                this.Direction = direction;
            }
            private Vector3 Center { get; set; }
            private float Length { get; set; }
            private float Breadth { get; set; }
            private float Height { get; set; }

            private float Direction { get; set; }

            public Vector3[] GETCornerCoordinates()
            {
                Vector3[] cords =
                {
                    new Vector3(Center.x + Length/2,Center.y + Breadth/2, Center.z + Height/2),
                    new Vector3(Center.x + Length/2,Center.y + Breadth/2, Center.z - Height/2),
                    new Vector3(Center.x + Length/2,Center.y - Breadth/2, Center.z + Height/2),
                    new Vector3(Center.x + Length/2,Center.y - Breadth/2, Center.z - Height/2),
                    new Vector3(Center.x - Length/2,Center.y + Breadth/2, Center.z + Height/2),
                    new Vector3(Center.x - Length/2,Center.y + Breadth/2, Center.z - Height/2),
                    new Vector3(Center.x - Length/2,Center.y - Breadth/2, Center.z + Height/2),
                    new Vector3(Center.x - Length/2,Center.y - Breadth/2, Center.z - Height/2),
                };
                return cords;
            }
        }
        
    }
}