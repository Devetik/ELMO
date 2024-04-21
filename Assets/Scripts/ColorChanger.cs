using UnityEngine;

public class ColorChanger : MonoBehaviour
{
    public Renderer objectRenderer; // Assurez-vous d'assigner cela dans l'inspecteur Unity
    private float rewardAccumulator = 0f; // Accumulateur pour les récompenses

    void Start()
    {
        // Initialisation de la couleur à blanc
        SetColor(Color.white);
    }

    public void UpdateColorBasedOnReward(float reward)
    {
        // Accumuler la récompense
        rewardAccumulator += reward;

        // Calculer le ratio pour les couleurs en fonction de la récompense accumulée
        if (rewardAccumulator > 0)
        {
            // Teinter en vert si la récompense est positive
            float greenRatio = Mathf.Clamp(rewardAccumulator / 100.0f, 0, 1);
            SetColor(new Color(1.0f - greenRatio, 1.0f, 1.0f - greenRatio));
        }
        else
        {
            // Teinter en rouge si la récompense est négative
            float redRatio = Mathf.Clamp(-rewardAccumulator / 100.0f, 0, 1);
            SetColor(new Color(1.0f, 1.0f - redRatio, 1.0f - redRatio));
        }

        // Diminuer progressivement la récompense accumulée pour permettre un retour à blanc
        rewardAccumulator *= 0.99f; // Ajustez ce facteur selon la vitesse de retour souhaitée
    }

    private void SetColor(Color color)
    {
        if (objectRenderer != null)
        {
            objectRenderer.material.color = color;
        }
    }
}
