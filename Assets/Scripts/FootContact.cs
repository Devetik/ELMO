using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FootContact : MonoBehaviour
{
    public RobotWalk centralController;
    public bool LeftFootOnFloor;
    public bool RightFootOnFloor;
    public bool LeftHandOnFloor;
    public bool RightHandOnFloor;
    public bool BassinOnFloor;
    public bool AbdoOnFloor;
    public bool TorseOnFloor;
    public bool HeadOnFloor;
    public bool LeftTibiasOnFloor;
    public bool RightTibiasOnFloor;
    private void OnTriggerEnter(Collider other)
    {
        // Utilisez la référence centralController pour appeler la méthode HandleCollision
        if (centralController != null) // Assurez-vous que centralController n'est pas null
        {
            centralController.HandleCollision(gameObject.name, other);
        }
        else
        {
            Debug.LogWarning("centralController n'est pas défini sur " + gameObject.name);
        }
    }

    void OnCollisionEnter(Collision collision) 
    {
        // Le nom de l'objet sur lequel ce script est attaché
        string thisObjectName = gameObject.name;

        // Le nom de l'autre objet impliqué dans la collision
        string otherObjectName = collision.gameObject.name;
        //Debug.Log(thisObjectName + " collision with " + collision.gameObject.name);
        if(thisObjectName == "Pied.L" && otherObjectName == "Floor")
        {
            LeftFootOnFloor = true;
        }
        if(thisObjectName == "Pied.R" && otherObjectName == "Floor")
        {
            RightFootOnFloor = true;
        }
        if(thisObjectName == "Tibias.L" && otherObjectName == "Floor")
        {
            LeftTibiasOnFloor = true;
        }
        if(thisObjectName == "Tibias.R" && otherObjectName == "Floor")
        {
            RightTibiasOnFloor = true;
        }
        
        if(thisObjectName == "Head" && otherObjectName == "Floor")
        {
            HeadOnFloor = true;
        }
        if(thisObjectName == "Abdo" && otherObjectName == "Floor")
        {
            AbdoOnFloor = true;
        }
        if(thisObjectName == "Torse" && otherObjectName == "Floor")
        {
            TorseOnFloor = true;
        }
        if(thisObjectName == "Bassin" && otherObjectName == "Floor")
        {
            BassinOnFloor = true;
        }
        if(thisObjectName == "Main.R" && otherObjectName == "Floor")
        {
            RightHandOnFloor = true;
        }
        if(thisObjectName == "Main.L" && otherObjectName == "Floor")
        {
            LeftHandOnFloor = true;
        }






        if(otherObjectName == "Floor" && thisObjectName != "Pied.R" && thisObjectName != "Pied.L")
        {
            centralController.HandleCollision(thisObjectName, collision.gameObject.GetComponent<Collider>());
        }
    }
    void OnCollisionExit(Collision collision) 
    {
        // Le nom de l'objet sur lequel ce script est attaché
        string thisObjectName = gameObject.name;

        // Le nom de l'autre objet impliqué dans la collision
        string otherObjectName = collision.gameObject.name;
        //Debug.Log(thisObjectName + " Uncollision with " + collision.gameObject.name);
        if(thisObjectName == "Pied.L" && otherObjectName == "Floor")
        {
            LeftFootOnFloor = false;
        }
        if(thisObjectName == "Pied.R" && otherObjectName == "Floor")
        {
            RightFootOnFloor = false;
        }
        if(thisObjectName == "Tibias.L" && otherObjectName == "Floor")
        {
            LeftTibiasOnFloor = false;
        }
        if(thisObjectName == "Tibias.R" && otherObjectName == "Floor")
        {
            RightTibiasOnFloor = false;
        }
        if(thisObjectName == "Head" && otherObjectName == "Floor")
        {
            HeadOnFloor = false;
        }
        if(thisObjectName == "Abdo" && otherObjectName == "Floor")
        {
            AbdoOnFloor = false;
        }
        if(thisObjectName == "Torse" && otherObjectName == "Floor")
        {
            TorseOnFloor = false;
        }
        if(thisObjectName == "Bassin" && otherObjectName == "Floor")
        {
            BassinOnFloor = false;
        }
        if(thisObjectName == "Main.R" && otherObjectName == "Floor")
        {
            RightHandOnFloor = false;
        }
        if(thisObjectName == "Main.L" && otherObjectName == "Floor")
        {
            LeftHandOnFloor = false;
        }
    }
}
