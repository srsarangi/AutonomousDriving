using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class tags : MonoBehaviour
{
    
    void Awake()
    {
        if (transform.childCount > 0)
            foreach (Transform t in transform)
                t.GetComponent<TrafficSystemNode>().isTurn = true;
    }
}
