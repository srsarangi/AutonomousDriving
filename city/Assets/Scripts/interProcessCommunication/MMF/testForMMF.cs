using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.CrossPlatformInput;
using System.IO.MemoryMappedFiles;
using System.Threading;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Runtime.InteropServices;
using System.Text;
using static TrafficSystemNode;

public class testForMMF : MonoBehaviour
{
    [DllImport("sem", EntryPoint="semaphore_open", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
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
    [DllImport("sem", EntryPoint="test", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
    public static extern string test();
    [DllImport("sem", EntryPoint="readMMF", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
    public static extern string readMMF(int mmap, int size);
    [DllImport("sem", EntryPoint="getO_CREAT_ORDWR", CharSet = CharSet.Ansi)]
    public static extern int getO_CREAT_ORDWR(); 
    [DllImport("sem", EntryPoint="memfree", CharSet = CharSet.Ansi)]
    public static extern void memfree(int mmap);
   
    [DllImport("sem", EntryPoint="buffinit", CharSet = CharSet.Ansi)]
    public static extern void buffinit(int mmap, int size);
    [DllImport("sem", EntryPoint="ReadInt", CharSet = CharSet.Ansi)]
    public static extern int ReadInt(int mmap, int size);
    [DllImport("sem", EntryPoint="WriteInt", CharSet = CharSet.Ansi)]
    public static extern int WriteInt(int val, int mmap);
    

    private int mmf, mmf2, mmf3, lockformmf;
    public string s1, s2;
    public int  v1, v2;
    public double angle;
    // Start is called before the first frame update
    void Start()
    {
        int shm_fd=shared_mem_open("steerAngle", getO_CREAT_ORDWR());
        ftrunc(shm_fd, 20);
        mmf=mmap_obj(20, shm_fd);
        //buffinit(mmf, 20);
        //mmf = MemoryMappedFile.OpenExisting("steerAngle");
        shm_fd=shared_mem_open("total_pts", getO_CREAT_ORDWR());
        ftrunc(shm_fd, 20);
        mmf2=mmap_obj(20, shm_fd);
        //buffinit(mmf2, 20);
        //mmf2 = MemoryMappedFile.OpenExisting("numLeft");
        shm_fd=shared_mem_open("numLane", getO_CREAT_ORDWR());
        ftrunc(shm_fd, 20);
        mmf3=mmap_obj(20, shm_fd);
        //buffinit(mmf3, 20);
        //mmf3 = MemoryMappedFile.OpenExisting("numRight");
        lockformmf=semaphore_open("lockSteer", getO_Creat(), 1);
        post(lockformmf);
    }

    // Update is called once per frame
    void Update()
    {
        wait(lockformmf);
        //memfree(mmf);
        //string s3=readMMF(mmf,20);
        //angle=BitConverter.ToDouble(Convert.FromBase64String(s3), 0);
        //memfree(mmf);
        angle = ReadInt(mmf, 20);
        v1 = ReadInt(mmf2, 20);
        v2 = ReadInt(mmf3, 20);
        post(lockformmf);
        
        /*try{
          s2=readMMF(mmf,12);
          angle=BitConverter.ToDouble(Convert.FromBase64String(s3), 0);
        }
        catch (Exception e){
          post(lockformmf);
        }*/
        //angle=BitConverter.ToDouble(Convert.FromBase64String(readMMF2(mmf, 12)), 0);
        
        //s1 = test();
    }

    private void OnApplicationQuit()
    {
        post(lockformmf);
    }
}
