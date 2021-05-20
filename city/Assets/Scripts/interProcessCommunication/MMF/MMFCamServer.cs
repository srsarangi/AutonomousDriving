using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Threading;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;
namespace UnityStandardAssets.Vehicles.Car
{
    public class MMFCamServer : MonoBehaviour
    {
        private int mmf;
        private int lockForMMF;
        //public double outme=0;
        //private Process mmfProcessClient;
        private int mmf2, mmf3, mmf4;
        //private int lockForMMF2, lockForMMF3;
        public Camera cam;
        private ACC2 acc;
        [DllImport("sem", EntryPoint="reset", CharSet = CharSet.Ansi)]
        public static extern void  reset();
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
        // MemoryMappedViewStream stream;
        //private MemoryMappedViewAccessor accessor, accessor2;
        void Start()
        {
            // create new mmf, takes filename and max size in bytes 
            // FIXME: fix size to max resolution of image, pass as argument maybe
            // creates file structure in memory only unless loaded from file
            int shm_fd=shared_mem_open("imageTransfer", getO_CREAT_ORDWR());
            ftrunc(shm_fd, 1000000);
            mmf=mmap_obj(1000000, shm_fd);
            //mmf = MemoryMappedFile.CreateNew("imageTransfer", 1000000);
            //shm_fd=shared_mem_open("steerAngle", getO_CREAT_ORDWR());
            //ftrunc(shm_fd, 12);
            //mmf2=mmap_obj(12, shm_fd);
            //mmf2 = MemoryMappedFile.CreateNew("steerAngle", 8);
            //shm_fd=shared_mem_open("total_pts", getO_CREAT_ORDWR());
            //ftrunc(shm_fd, 8);
            //mmf3=mmap_obj(8, shm_fd);
            ///mmf3= MemoryMappedFile.CreateNew("numLeft", 4);
            //shm_fd=shared_mem_open("numLane", getO_CREAT_ORDWR());
            //trunc(shm_fd, 8);
            //mmf4=mmap_obj(8, shm_fd);
            //mmf4 = MemoryMappedFile.CreateNew("numRight", 4);
            acc = GetComponent<ACC2>(); // get the car control script
            // creating lock
            bool mutexCreated; 
            // secoond argument of Mutex is lock name
            // defualt security for mutex is SYNCHRONIZE
            //lockForMMF2=semaphore_open("lockSteer", getO_Creat(), 1);
            //lockForMMF2 = new Mutex(true, "lockSteer", out mutexCreated);
            //lockForMMF2.ReleaseMutex();
            //post(lockForMMF2);
            lockForMMF=semaphore_open("lockForMMF_img", getO_Creat(), 1);
            //post(lockForMMF);
            //lockForMMF = new Mutex(true, "lockForMMF", out mutexCreated);
            //accessor = mmf.CreateViewAccessor();
            //lockForMMF.ReleaseMutex();
            //lockForMMF3=semaphore_open("lockNum", getO_Creat(), 1);
            //post(lockForMMF3);
            //lockForMMF3 = new Mutex(true, "lockNum", out mutexCreated);
            //accessor2= mmf4.CreateViewAccessor();
            //lockForMMF3.ReleaseMutex();
            // stream = mmf.CreateViewStream();

            // start python scripts
            //ProcessStartInfo processInfo = new ProcessStartInfo("cmd.exe", "/c " + "python Scripts/interProcessCommunication/MMF/mmfProcessClient.py");
            //processInfo.WorkingDirectory = Application.dataPath;
            //processInfo.CreateNoWindow = false;
            //processInfo.UseShellExecute = true;
            //processInfo.RedirectStandardError = false;
            //processInfo.RedirectStandardOutput = false;

            //mmfProcessClient = Process.Start(processInfo);
            
        }

        void LateUpdate()
        {
            string s = Convert.ToBase64String(ImageCapture.CameraCapture(cam));
            //byte[] Buffer = ASCIIEncoding.ASCII.GetBytes(s);
            // Acquire lock
            wait(lockForMMF);
            writeMMF(s, mmf);
            post(lockForMMF);
//            lockForMMF.WaitOne();
  //          accessor.WriteArray(0, Buffer, 0, Buffer.Length);
            // Release lock
    //        lockForMMF.ReleaseMutex();
      //      lockForMMF3.WaitOne();
        //    outme = accessor2.ReadInt32(0);
          //  lockForMMF3.ReleaseMutex();
            
        }
        

        void OnApplicationQuit()
        {
	        //post(lockForMMF);
	        reset();
            // Kill child process, close opened files
            /*if (!mmfProcessClient.HasExited)
            {
                mmfProcessClient.Kill();
                mmfProcessClient.WaitForExit();
                UnityEngine.Debug.Log("killed");
            }*/
            
            // ReadOnlyCollectionBase all mmf resources
            //accessor.Dispose();
            // stream.Dispose();
            //mmf.Dispose();
            //lockForMMF.Dispose();
            //accessor2.Dispose();
            // stream.Dispose();
            //mmf2.Dispose();
            //lockForMMF2.Dispose();
            //accessor.Dispose();
            // stream.Dispose();
            //mmf3.Dispose();
            //lockForMMF3.Dispose();
            //mmf4.Dispose();
            //UnityEngine.Debug.Log("closed");
        }
    }
}
