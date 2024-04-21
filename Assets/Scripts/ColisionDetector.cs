using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ColisionDetector : MonoBehaviour
{
    public RobotWalk centralController;
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
}
