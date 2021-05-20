using System.Collections;
using System.Collections.Generic;
using UnityEngine;



public class invisible : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        gameObject.GetComponent<Renderer>().enabled = false;

    }
}


//Edited by us  (Yash and Pranay) 
//Reason :- This script turns off the rendering of the nodes so that they are finally not shown in the Unity editor.If nodes are shown then it creates problems with lane detection