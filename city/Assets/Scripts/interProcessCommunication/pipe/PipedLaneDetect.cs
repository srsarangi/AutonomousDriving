using UnityEngine;
using System;
using System.IO;
using System.IO.Pipes;
using System.Diagnostics;

public class PipedLaneDetect : MonoBehaviour
{
	// Start is called before the first frame update
	public Camera cam;
	private NamedPipeServerStream imageSendPipe;
	private StreamWriter imageSendPipeWriter;
	private Process laneDetector;
	private bool connected;
	void Start()
	{
		// setting up pipes
		UnityEngine.Debug.Log("Starting Server");
		// name of pipe is set below
		imageSendPipe = new NamedPipeServerStream("imageSendPipe");

		// start python laneDetector scripts
		ProcessStartInfo processInfo = new ProcessStartInfo("cmd.exe", "/c " + "python Scripts/pipedLaneDetector.py");
		processInfo.WorkingDirectory = Application.dataPath;
		processInfo.CreateNoWindow = false;
		processInfo.UseShellExecute = true;
		processInfo.RedirectStandardError = false;
		processInfo.RedirectStandardOutput = false;

		laneDetector = Process.Start(processInfo);

		// Create stream writer for pipe after it is connected
		UnityEngine.Debug.Log("Waiting");
		imageSendPipe.WaitForConnection();
		UnityEngine.Debug.Log("Done waiting");
		imageSendPipeWriter = new StreamWriter(imageSendPipe);
	}

	// Update is called once per frame
	void Update()
	{
		String s = Convert.ToBase64String(ImageCapture.CameraCapture(cam));
		int sLen =  System.Text.ASCIIEncoding.Unicode.GetByteCount(s);
		// UnityEngine.Debug.Log(sLen);
		// UnityEngine.Debug.Log(s);
		imageSendPipeWriter.WriteLine("{0:000000000}", sLen);
		imageSendPipeWriter.WriteLine(s);
	}

	void OnApplicationQuit() {
		// Kill child process, close opened files
		if (!laneDetector.HasExited) {
			laneDetector.Kill();
			laneDetector.WaitForExit();
			UnityEngine.Debug.Log("killed");
		}

		// Close and writer and pipe
		imageSendPipeWriter.Close();
		imageSendPipe.Disconnect();
		imageSendPipe.Close();
		UnityEngine.Debug.Log("closed");
	}
}
