using UnityEngine;

public class FollowCamera : MonoBehaviour
{
    public Transform target;  // La cible que la caméra doit suivre (votre agent)
    public Vector3 offset = new Vector3(0, 5, -10);  // Décalage par rapport à la cible

    void LateUpdate()
    {
        // Mettre à jour la position de la caméra pour qu'elle suive la cible avec le décalage spécifié
        if (target != null)
        {
            transform.position = target.position + offset;
            transform.LookAt(target);  // Fait en sorte que la caméra regarde toujours vers la cible
        }
    }
}