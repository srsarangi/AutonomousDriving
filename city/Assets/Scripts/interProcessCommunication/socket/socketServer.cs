using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using SocketIO;
using System;

public class socketServer : MonoBehaviour
{
    public Camera cam;
    private SocketIOComponent socket;
    WaitForEndOfFrame frameEnd = new WaitForEndOfFrame();

    void Start()
    {
        GameObject go = GameObject.Find("SocketIO");
        socket = go.GetComponent<SocketIOComponent>();

        Debug.Log("Started");
        socket.On("beep", sendImage);
    }

    private void Update()
    {
        //socket.On("beep", TestBeep);
    }

    void sendImage(SocketIOEvent obj)
    {
        StartCoroutine(test(obj));
    }

    IEnumerator test(SocketIOEvent obj)
    {
        yield return frameEnd;

        Dictionary<string, string> data = new Dictionary<string, string>();
        data["img"] = Convert.ToBase64String(ImageCapture.CameraCapture(cam));
        socket.Emit("image", new JSONObject(data));
    }
}
