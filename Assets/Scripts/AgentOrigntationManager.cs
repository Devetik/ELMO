using UnityEngine;

public class AgentOrientationManager : MonoBehaviour
{
    public Transform bodyRoot; // Racine du corps à partir de laquelle tous les membres sont attachés
    public Transform[] bodyParts; // Membres individuels qui peuvent nécessiter des ajustements de rotation spécifiques

    // Méthode pour orienter tout le corps avec des angles spécifiques
    public void OrientateAgentWithSpecificRotation(float xAngle, float yAngle, float zAngle)
    {
        Quaternion specificRotation = Quaternion.Euler(xAngle, yAngle, zAngle);
        bodyRoot.rotation = specificRotation;

        foreach (var part in bodyParts)
        {
            // Appliquer des ajustements si nécessaire
            AdjustPartRotation(part);
        }
    }

    // Méthode pour ajuster la rotation des membres individuels si nécessaire
    private void AdjustPartRotation(Transform part)
    {
        // Ici, vous pouvez ajouter des ajustements supplémentaires
        part.localRotation = Quaternion.identity; // Exemple pour remettre la rotation locale à zéro
    }
}
