using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Threading;
using System.Diagnostics;
using System.Text;

public class laneFollowerCommunication : MonoBehaviour
{
	private Process clientProcess;
	private MemoryMappedFile mmfImageTransfer;
	// Lock
	private PetersonLock mutexImageTransfer;
	private MemoryMappedViewAccessor accessorImageTransfer;

	private MemoryMappedFile mmfRetrievedInfo;
	private PetersonLock mutexRetrievedInfo;
	private MemoryMappedViewAccessor accessorRetrievedInfo;

	public Camera cam;

	// Ex: "python Scripts/interProcessCommunication/MMF/laneFollower.py"
	public string clientProcessCommand;

	// Ex: "ImageTransferLF"
	public string imageTranferFileName;

	// Ex: "RetrievedInfoLF"
	public string retrievedInfoFileName;

	// Ex: "ImageTransferPetersonLF"
	public string imageTransferPetersonLockFileName;

	// Ex: "RetrievedInfoPetersonLF"
	public string retrievedInfoPetersonLockFileName;

	// Ex: "ServerStateLF"
	public string serverStateFilename;

	private MemoryMappedFile mmfServerState;
	private MemoryMappedViewAccessor accessorServerState;

	// Ex: "ClientStateLF"
	public string clientStateFilename;
	private MemoryMappedFile mmfClientState;
	private MemoryMappedViewAccessor accessorClientState;

	private int serverState = 0;
	private int clientState = 0;

	public float steeringAngle = 0.0f;

	public float laneOffset = 0.0f;

	void Start() {
		// create new mmf, takes filename and max size in bytes 
		// FIXME: fix size to max resolution of image, pass as argument maybe
		// creates file structure in memory only unless loaded from file
		mmfImageTransfer = MemoryMappedFile.CreateNew(imageTranferFileName, 1000000);

		// creating lock
		mutexImageTransfer = new PetersonLock(imageTransferPetersonLockFileName, 2, false);

		// stream = mmf.CreateViewStream();
		accessorImageTransfer = mmfImageTransfer.CreateViewAccessor();

		mmfRetrievedInfo = MemoryMappedFile.CreateNew(retrievedInfoFileName, 4*2);
		mutexRetrievedInfo = new PetersonLock(retrievedInfoPetersonLockFileName, 2, false);
		accessorRetrievedInfo = mmfRetrievedInfo.CreateViewAccessor();

		mmfServerState = MemoryMappedFile.CreateNew(serverStateFilename, 4);
		accessorServerState = mmfServerState.CreateViewAccessor();

		mmfClientState = MemoryMappedFile.CreateNew(clientStateFilename, 4);
		accessorClientState = mmfClientState.CreateViewAccessor();

		// start python scripts
		ProcessStartInfo processInfo = new ProcessStartInfo("cmd.exe", "/c " + clientProcessCommand + " " + imageTranferFileName + " " + imageTransferPetersonLockFileName + " " + retrievedInfoFileName + " " + retrievedInfoPetersonLockFileName + " " + serverStateFilename + " " + clientStateFilename);
		processInfo.WorkingDirectory = Application.dataPath;
		processInfo.CreateNoWindow = false;
		processInfo.UseShellExecute = true;
		processInfo.RedirectStandardError = false;
		processInfo.RedirectStandardOutput = false;

		clientProcess = Process.Start(processInfo);
	}

	void Update() {
		// Share Image
		string s = Convert.ToBase64String(ImageCapture.CameraCapture(cam));
		byte[] imageBuffer = ASCIIEncoding.ASCII.GetBytes(s);
		// Acquire lock
		mutexImageTransfer.acquire();
		accessorImageTransfer.WriteArray(0, imageBuffer, 0, imageBuffer.Length);
		// Release lock
		mutexImageTransfer.release();

		if (clientState != 1) {
			UnityEngine.Debug.Break();
		}

		byte[] clientStateBuffer = new byte[4];
		accessorClientState.ReadArray(0, clientStateBuffer, 0, 4);
		clientState = PetersonLock.bytesToInt(clientStateBuffer);

		if (serverState != 1) {
			byte[] serverStateBuffer = PetersonLock.intToBytes(1);
			accessorServerState.WriteArray(0, serverStateBuffer, 0, 4);
			serverState = 1;
		}
		
		// Get Info Back
		byte[] steeringAngleBuffer = new byte[4];
		byte[] laneOffsetBuffer = new byte[4];

		mutexRetrievedInfo.acquire();
		accessorRetrievedInfo.ReadArray(0, steeringAngleBuffer, 0, 4);
		accessorRetrievedInfo.ReadArray(4, laneOffsetBuffer, 0, 4);
		mutexRetrievedInfo.release();

		// if (BitConverter.IsLittleEndian) {
		// 	Array.Reverse(steeringAngleBuffer);
		// 	Array.Reverse(laneOffsetBuffer);
		// }

		steeringAngle = BitConverter.ToSingle(steeringAngleBuffer, 0);
		laneOffset = BitConverter.ToSingle(laneOffsetBuffer, 0);
	}

	void OnApplicationQuit() {
		// Kill child process, close opened files
		if (!clientProcess.HasExited) {
			byte[] serverStateBuffer = PetersonLock.intToBytes(0);
			accessorServerState.WriteArray(0, serverStateBuffer, 0, 4);
			serverState = 0;
			// clientProcess.Kill();
			clientProcess.WaitForExit();
			UnityEngine.Debug.Log("killed");
		}

		// ReadOnlyCollectionBase all mmf resources
		accessorImageTransfer.Dispose();
		// stream.Dispose();
		mmfImageTransfer.Dispose();
		mutexImageTransfer.dispose();

		accessorRetrievedInfo.Dispose();
		mmfRetrievedInfo.Dispose();
		mutexRetrievedInfo.dispose();

		accessorServerState.Dispose();
		mmfServerState.Dispose();

		accessorClientState.Dispose();
		mmfClientState.Dispose();

		UnityEngine.Debug.Log("closed");
	}
}
