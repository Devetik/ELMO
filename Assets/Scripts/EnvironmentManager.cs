using UnityEngine;

public class EnvironmentManager : MonoBehaviour
{
    public Transform[] agentsTransforms; // Assignez ceci dans l'éditeur
    public Vector3[] startingPositions; // Positions de départ pour chaque agent
    public Quaternion[] startingRotations; // Rotations de départ pour chaque agent

    // Ajoutez des références à d'autres objets de l'environnement que vous devez réinitialiser

    public void ResetEnvironment()
    {
        // Réinitialisez la position et la rotation de chaque agent
        for (int i = 0; i < agentsTransforms.Length; i++)
        {
            agentsTransforms[i].position = startingPositions[i];
            agentsTransforms[i].rotation = startingRotations[i];
        }

        // Ajoutez ici la logique pour réinitialiser d'autres objets de l'environnement

        // Par exemple, réinitialiser des obstacles, des cibles, etc.
    }
}
