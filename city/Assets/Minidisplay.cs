using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Minidisplay : MonoBehaviour
{
    public Transform player;
    public Fusion player1;
    // Start is called before the first frame update
    void Start()
    {

    }

    private void Update()
    {
        // player1.TestCalculatedPath();
    }
    // Update is called once per frame
    public void OnDrawGizmos()
    {
        Gizmos.DrawCube(player.transform.forward, Vector3.one);
    }
    void LateUpdate()
    {

        transform.rotation = Quaternion.Euler(49f, player.eulerAngles.y, player.eulerAngles.z);
    }
}