using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Threading;
using System.Diagnostics;
using System.Text;

public class MMFCamServerPeterson : MonoBehaviour
{
	private MemoryMappedFile mmf;
	private Process mmfClientProcess;
	// Lock
	private PetersonLock mutex;
	public Camera cam;
	private MemoryMappedViewAccessor accessor;

	// Ex: "python Scripts/interProcessCommunication/MMF/mmfProcessClientPeterson.py"
	public string clientProcessCommand;

	// Ex: "imageTransferPeterson"
	public string imageTranferFileName;

	// Ex: "MMFPeterson"
	public string petersonLockFileName;

	void Start() {
		// create new mmf, takes filename and max size in bytes 
		// FIXME: fix size to max resolution of image, pass as argument maybe
		// creates file structure in memory only unless loaded from file
		mmf = MemoryMappedFile.CreateNew(imageTranferFileName, 1000000);

		// creating lock
		mutex = new PetersonLock(petersonLockFileName, 2, false);

		// stream = mmf.CreateViewStream();
		accessor = mmf.CreateViewAccessor();

		// start python scripts
		ProcessStartInfo processInfo = new ProcessStartInfo("cmd.exe", "/c " + clientProcessCommand + " " + imageTranferFileName + " " + petersonLockFileName);
		processInfo.WorkingDirectory = Application.dataPath;
		processInfo.CreateNoWindow = false;
		processInfo.UseShellExecute = true;
		processInfo.RedirectStandardError = false;
		processInfo.RedirectStandardOutput = false;

		mmfClientProcess = Process.Start(processInfo);
	}

	void Update() {
		string s = Convert.ToBase64String(ImageCapture.ScreenCapture());
		// string s = Convert.ToBase64String(ImageCapture.CameraCapture(cam));
		byte[] Buffer = ASCIIEncoding.ASCII.GetBytes(s);
		// Acquire lock
		mutex.acquire();
		accessor.WriteArray(0, Buffer, 0, Buffer.Length);
		// Release lock
		mutex.release();
	}

	void OnApplicationQuit() {
		// Kill child process, close opened files
		if (!mmfClientProcess.HasExited) {
			mmfClientProcess.Kill();
			mmfClientProcess.WaitForExit();
			UnityEngine.Debug.Log("killed");
		}

		// ReadOnlyCollectionBase all mmf resources
		accessor.Dispose();
		// stream.Dispose();
		mmf.Dispose();
		mutex.dispose();
		UnityEngine.Debug.Log("closed");
	}
}
