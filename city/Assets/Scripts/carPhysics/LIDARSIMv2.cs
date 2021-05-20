using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;
using Emgu;
using System.Drawing;
using System.Runtime.InteropServices;
using Emgu.CV;
using Emgu.CV.Structure;
using Object = System.Object;

namespace CarPhysics
{
    public class LIDARSIMv2 : MonoBehaviour
    {
        public float upperFOV = 5f;
        public float lowerFOV = -20f;
        public int numChannels = 20;
        //public Rigidbody car_rigid_body;
        // string path;
        //StreamWriter outputfile;
        //public float speed;
        public int numRays = 36;
        private Vector3 origin;
        private RaycastHit outhit;
        private bool flag;
        Vector3[,] dirs;
        private string path;
        private Quaternion horQuat;
        private static Vector3[] points;
        private volatile int points_idx;
        private volatile Vector3[] pointCloud;
        private static bool isPointCloudReady = false;
        //private StreamWriter outputfile;
        private float res=0.5f;
        private float floor=-1.2f;
        private Rigidbody rigid_car;
        //private string dll_path = Application.dataPath+"/Scripts/carPhysics/sem.so";
        [DllImport("sem", EntryPoint="semaphore_open", CharSet = CharSet.Ansi)]
	      public static extern int semaphore_open(string semname, int oflag, int val);
          [DllImport("sem", EntryPoint="reset", CharSet = CharSet.Ansi)]
          public static extern void  reset();
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
      	[DllImport("sem", EntryPoint="WriteInt", CharSet = CharSet.Ansi)]
      	public static extern void WriteInt(int val, int mmap);
      	[DllImport("sem", EntryPoint="readMMF", CharSet = CharSet.Ansi)]
      	public static extern string readMMF(int mmap, int size);
      	[DllImport("sem", EntryPoint="getO_CREAT_ORDWR", CharSet = CharSet.Ansi)]
        public static extern int getO_CREAT_ORDWR();
        private readonly int[] grid_size = new int[2];
        private int offset_x, offset_y;
        //private int voxelGrid[,];
        Texture2D voxelGrid;
        private int point_sem, mmf, mmf2;
        // Start is called before the first frame update
        void Start()
        {
            grid_size[0] = grid_size[1] = 80;
            //outputfile = new StreamWriter(Path.Combine("./", "thesisRes.txt"), true);
            point_sem=semaphore_open("point_sem", getO_Creat(), 1);
            int shm_fd=shared_mem_open("objects", getO_CREAT_ORDWR());
            ftrunc(shm_fd, 1000000);
            mmf=mmap_obj(1000000, shm_fd);
            
            shm_fd=shared_mem_open("speed", getO_CREAT_ORDWR());
            ftrunc(shm_fd, 20);
            mmf2=mmap_obj(20, shm_fd);
            
            points_idx = 0;
//            path = @"./Assets/Scripts/carPhysics";
            
            //points = new Vector3[14400];
            voxelGrid=new Texture2D(grid_size[0], grid_size[1], TextureFormat.RGB24, false);
            offset_x=grid_size[0]/2;
            offset_y=grid_size[1]/2;
            Quaternion vert_angle_up = Quaternion.AngleAxis(1f, transform.right);
            Quaternion vert_angle_down = Quaternion.AngleAxis(-1f, transform.right);
            dirs = new Vector3[numRays, numChannels+3];
            horQuat = Quaternion.AngleAxis(1, transform.up);
            dirs[0, 0] = vert_angle_down * transform.forward;
            for (int i = 1; i < 3; i++)
            {
                dirs[0, i] = vert_angle_down * dirs[0, i - 1];
            }

            dirs[0, 3] = vert_angle_up * transform.forward;
            for (int i = 4; i < numChannels+3; i++)
            {
                dirs[0, i] = vert_angle_up * dirs[0, i - 1];
            }

            float tempangle = 360 / numRays;
            for (int i = 1; i < numRays; i++)
            {
                Quaternion temp = Quaternion.AngleAxis(tempangle, transform.up);
                for (int j = 0; j < numChannels+3; j++)
                {
                    dirs[i, j] = temp * dirs[i - 1, j];
                }
            }

            rigid_car = (Rigidbody) GetComponentInParent(typeof(Rigidbody));
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            //if (!isPointCloudReady)
            //{
                //points_idx = 0;
                origin = transform.position + transform.up;
                // var results = new NativeArray<RaycastHit>(numRays * 2 * numChannels, Allocator.Temp);
                //outputfile.WriteLine("hi");
                // var commands = new NativeArray<RaycastCommand>(numRays * 2 * numChannels, Allocator.Temp);
                // outputFile.WriteLine("Fourth Line");
                for (int j = 0; j < numRays; j++)
                {
                    for (int i = 0; i < numChannels+3; i++)
                    {
                        // Debug.Log(points_idx.ToString());
                        flag = Physics.Raycast(origin, dirs[j, i], out outhit, 20f);
                        if (flag && !outhit.collider.CompareTag("Terrain"))
                        {
                            //Debug.DrawLine(origin, outhit.point, Color.red);
                            //Vector3 tmp = outhit.point - origin;
                            Vector3 tmp=transform.InverseTransformPoint(outhit.point);
                            //outputfile.WriteLine(tmp[2] + " " + tmp[0] + " " + tmp[1]);
                            if(tmp[1]>floor)
                              voxelGrid.SetPixel((int)(tmp[0]/res)+offset_y, (int)(tmp[2]/res)+offset_x, Color.white);
                            //else
                              //voxelGrid.SetPixel((int)(tmp[0]/res)+offset_y, (int)(-tmp[2]/res)+offset_x, Color.black);
                            //points[points_idx] = tmp;
                            
                        }

                        // commands[j*numRays+i] = new RaycastCommand(origin, dirs[j, i]);
                        dirs[j, i] = horQuat * dirs[j, i];
                        // cnout = j;
                    }
                }
                points_idx++;
                if(points_idx>=5){ 
                  voxelGrid.Apply();
                  byte[] image = voxelGrid.EncodeToPNG();
                  //Object.DestroyImmediate(voxelGrid);
                  string strPts = Convert.ToBase64String(image);
                  voxelGrid = new Texture2D(grid_size[0], grid_size[1], TextureFormat.RGB24, false);
                  int speed = (int) (rigid_car.velocity.magnitude * 100);
                  wait(point_sem);
                  writeMMF(strPts, mmf);
                  WriteInt(speed, mmf2);
                  post(point_sem);
                  points_idx=0;
                }
              
                //isPointCloudReady = true;

                /*JobHandle handle = RaycastCommand.ScheduleBatch(commands, results,1 , default(JobHandle));
                handle.Complete();
                int n = numRays * 2 * numChannels;
                for(int i=0; i<n; i++)
                {
                    if (results[i].collider != null)
                    {
                        Debug.DrawLine(origin, results[i].point, Color.red);
                    }
                    else
                    {
                        break;
                    }
                }*/
            //}
        }
        

        

        private void OnApplicationQuit()
        {
            reset();
        }
       
        
        
        
        
        
    }
}