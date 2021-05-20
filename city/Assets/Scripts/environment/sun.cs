/* Author :     Prabhleen Kaur
   Date   :     7th July 2019
   Title  :     Code for day and night cycle
*/
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class sun : MonoBehaviour
{
    //To create day and night cycle- rotate directional light at 360 degrees

    // Update is called once per frame
    void Update()
    {
        transform.RotateAround(Vector3.zero, Vector3.right, 1f*Time.deltaTime);
        transform.LookAt(Vector3.zero);
    }
}
