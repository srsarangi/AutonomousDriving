using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class ImageCapture
{
    public static byte[] CameraCapture(Camera cam)
    {
        RenderTexture targetTexture = cam.targetTexture;
        RenderTexture.active = cam.targetTexture;
        Texture2D texture = new Texture2D(targetTexture.width, targetTexture.height, TextureFormat.RGB24, false);
        texture.ReadPixels(new Rect(0, 0, targetTexture.width, targetTexture.height), 0, 0, false);
        texture.Apply();
        byte[] image = texture.EncodeToPNG();

        Object.DestroyImmediate(texture);
        return image;
    }

    public static byte[] ScreenCapture()
    {
        Texture2D texture = new Texture2D(Screen.width, Screen.height, TextureFormat.RGB24, false);
        texture.ReadPixels(new Rect(0, 0, Screen.width, Screen.height), 0, 0, false);
        texture.Apply();
        byte[] image = texture.EncodeToJPG();
        Object.DestroyImmediate(texture);
        return image;
    }
}
