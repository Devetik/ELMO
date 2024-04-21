using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using Unity.VisualScripting;
using UnityEngine;

public class Pressure_Plate : MonoBehaviour
{
    [SerializeField]
    GameObject Plate;
    private bool isNotTriggered = true;
    // Start is called before the first frame update
    void OnTriggerEnter(Collider col)
    {
        if(isNotTriggered)
        {
            Plate.transform.position += new UnityEngine.Vector3(0,-0.1f,0);
            isNotTriggered = false;
        }
    }
    // void OnTriggerExit(Collider col)
    // {
    //     Plate.transform.position += new UnityEngine.Vector3(0,0.1f,0);
    // }

}
