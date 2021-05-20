using UnityEngine;
using InferenceCommunication;
using System.Net.Sockets;
using System;
using Newtonsoft.Json;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using CarPhysics;
using CarWorld;
using UnityEditor.PackageManager;


public class InferenceClient : MonoBehaviour
{

    [SerializeField] private string hostName = "localhost";
    [SerializeField] private int port = 11111;
    [SerializeField] private int minConnection = 5;
    [SerializeField] private int maxConnection = 20;
    
    [SerializeField] private Camera roadCamera;
    private Byte[] _pngImage;
    private bool _isImageFresh = false;
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
    //private int point_sem, mmf;
    // private StreamWriter outputfile;
    // private string path;

    void Awake()
    {
        ConnectionPool.InitializeConnectionPool(hostName, port, minConnection, maxConnection);
    }
    void Start()
    {
        // path = @"./";
        // outputfile = new StreamWriter(Path.Combine(path, "outputfile.txt"), true);
        /*point_sem=semaphore_open("point_sem", getO_Creat(), 1);
        int shm_fd=shared_mem_open("test", getO_CREAT_ORDWR());
        ftrunc(shm_fd, 1000000);
        mmf=mmap_obj(1000000, shm_fd);*/
        new Thread(TcpRequesterForImage).Start();
        new Thread(TcpRequesterForPointCloud).Start();
    }

    private void Update()
    {
        if (!_isImageFresh)
        {
            RenderTexture renderTexture = RenderTexture.active;

            RenderTexture.active = roadCamera.targetTexture;
            roadCamera.Render();
            Texture2D imageToBeAnalyzed = new Texture2D(roadCamera.targetTexture.width, roadCamera.targetTexture.height,
                TextureFormat.RGB24, false);
            imageToBeAnalyzed.ReadPixels(
                new Rect(0, 0, roadCamera.targetTexture.width, roadCamera.targetTexture.height), 0, 0);
            imageToBeAnalyzed.Apply();
            _pngImage = imageToBeAnalyzed.EncodeToPNG();
            //string str_pts = Convert.ToBase64String(_pngImage);
            //voxelGrid=new Texture2D(grid_size[0], grid_size[1], TextureFormat.RGB24, false);
            //wait(point_sem);
            //writeMMF(str_pts, mmf);
            //post(point_sem);
            RenderTexture.active = renderTexture;

            //_isImageFresh = true;
        }
    }

    void TcpRequesterForImage()
    {
        while (true)
        {
            if (_isImageFresh)
            {
                using (CustomSocket client = ConnectionPool.GetSocket())
                {
                    Debug.Log("Image Socket Take");
                    // Translate the render texture to texture2d and store it as a Byte array.
                    Int32 imageSize = _pngImage.Length;
                    
                    byte[] imageSizeInBytes = BitConverter.GetBytes(imageSize);
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(imageSizeInBytes);

                    // Get a client stream for reading and writing.
                    NetworkStream stream = client.GetStream();

                    // Send the message to the connected TcpServer.
                    
                    // First Byte Indicates type of Data I is for image and P is for PointCloud
                    stream.Write(Encoding.ASCII.GetBytes("I"), 0, 1);
                    stream.Write(imageSizeInBytes, 0, 4);
                    stream.Write(_pngImage, 0, imageSize);

                    // Receive the TcpServer.response.
                    // Buffer to store the response bytes.
                    Byte[] responseBuffer = new Byte[256];

                    // Read the first batch of the TcpServer response bytes.
                    Int32 bytes = stream.Read(responseBuffer, 0, responseBuffer.Length);
                    String responseData = Encoding.ASCII.GetString(responseBuffer, 0, bytes);

                    // String to JSON
                    Dictionary<string, string> imageAnalysis = JsonConvert.DeserializeObject<Dictionary<string, string>>(responseData);

                    // Update World State
                    if (imageAnalysis.ContainsKey("SteeringAngle")) { WorldState.Instance.SteerAngle = float.Parse(imageAnalysis["SteeringAngle"]); }
                    if (imageAnalysis.ContainsKey("TotalNumberOfPoints")) { WorldState.Instance.TotalNumberOfPoints = int.Parse(imageAnalysis["TotalNumberOfPoints"]); }
                    if (imageAnalysis.ContainsKey("LeftLanes")) { WorldState.Instance.LeftLanes = int.Parse(imageAnalysis["LeftLanes"]); }
                    if (imageAnalysis.ContainsKey("RightLanes")) { WorldState.Instance.RightLanes = int.Parse(imageAnalysis["RightLanes"]); }
                    // if (imageAnalysis.ContainsKey("TrafficLightColor")) { WorldState.Instance.TrafficLightColor = imageAnalysis["TrafficLightColor"][0]; }
                    // // if (imageAnalysis.ContainsKey("TrafficLightColor")) { WorldState.Instance.TrafficLightColor = 'r';}
                    
                    // if (imageAnalysis.ContainsKey("LeftLanes")) { Debug.Log("Left: "+ int.Parse(imageAnalysis["LeftLanes"])); }
                    // if (imageAnalysis.ContainsKey("RightLanes")) { Debug.Log("Right: "+int.Parse(imageAnalysis["RightLanes"])); }
                    
                    ConnectionPool.PutSocket(client);
                    Debug.Log("Image Socket put");
                    _isImageFresh = false;
                }
            }
        }
    }
    
    void TcpRequesterForPointCloud()
    {
        while (true)
        {
            Vector3[] pointCloud = LIDARSIM.GETLatestPointCloud();
            if (pointCloud == null)
            {
                continue;
            }
            using (CustomSocket client = ConnectionPool.GetSocket())
            {
                Debug.Log("Point Cloud Socket Take");
                // Get the point point cloud from Simulator to send
                // Vector3[] pointCloud = LIDARSIM.GETLatestPointCloud();
                // if (pointCloud == null)
                // {
                //     Debug.Log("Point Cloud Socket Put");
                //     ConnectionPool.PutSocket(client);
                //     continue;
                // }

                /*foreach (var t in pointCloud)
                {
                    outputfile.WriteLine(pointCloud[0] + " " + pointCloud[1] + " " + pointCloud[2]);
                }*/

                // Serialize the vector3 to specific string format
                string pointCloudString = vectorArrayToString(pointCloud);

                byte[] pointCloudBytes = Encoding.ASCII.GetBytes(pointCloudString);

                Int32 pointCloudBytesSize = pointCloudBytes.Length;

                byte[] pointCloudSizeInBytes = BitConverter.GetBytes(pointCloudBytesSize);
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(pointCloudSizeInBytes);

                // Get a client stream for reading and writing.
                NetworkStream stream = client.GetStream();

                // Send the message to the connected TcpServer.
                // First Byte Indicates type of Data I is for image and P is for PointCloud
                stream.Write(Encoding.ASCII.GetBytes("P"), 0, 1);
                stream.Write(pointCloudSizeInBytes, 0, 4);
                stream.Write(pointCloudBytes, 0, pointCloudBytesSize);

                // Receive the TcpServer.response.
                // Buffer to store the response bytes.
                // Byte[] responseBuffer = new Byte[1024];
                // //
                // // Read the first batch of the TcpServer response bytes.
                // Int32 bytes = stream.Read(responseBuffer, 0, responseBuffer.Length);
                // String responseData = System.Text.Encoding.ASCII.GetString(responseBuffer, 0, bytes);
                //
                // // String to JSON
                // Dictionary<string, List<float>> pointCloudInfo =
                //      JsonConvert.DeserializeObject<Dictionary<string, List<float>>>(responseData);
                
                ConnectionPool.PutSocket(client);
                Debug.Log("Point Cloud Socket Put");

            }
        }
    }

    private string vectorArrayToString(Vector3[] vector3Arrays)
    {
        StringBuilder serializedPointCloud = new StringBuilder();
        foreach (Vector3 vector3 in vector3Arrays)
        {
            string line = String.Format($"{(-vector3.z).ToString("0.000")},{vector3.x.ToString("0.000")},{vector3.y.ToString("0.000")}\n");
            serializedPointCloud.Append(line);
        }

        return serializedPointCloud.ToString();
    }

}


