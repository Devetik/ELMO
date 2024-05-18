using UnityEngine;
using Unity.MLAgents.Policies;
using Unity.Barracuda;
using System.Linq;
using UnityEngine.UI;
using TMPro;

public class ModelLoader : MonoBehaviour
{
    public string modelsFolder = "Assets/Resources/Models_V2";  // Sous-dossier dans Resources
    private string[] modelFiles;
    private int currentModelIndex = 0;
    private BehaviorParameters behaviorParameters;
    public TextMeshPro textGeneration;

    void Start()
    {
        // Obtenez tous les noms de fichiers de modèles dans le dossier Models sous Resources
        modelFiles = Resources.LoadAll<NNModel>(modelsFolder).Select(model => model.name).ToArray();
        if (modelFiles.Length == 0)
        {
            Debug.LogError("No .onnx model files found in the directory.");
            return;
        }

        behaviorParameters = GetComponent<BehaviorParameters>();
        LoadModel(0);
    }

public void LoadModel(int incrementStep = 0)
{
    // Assurez-vous que la liste des fichiers est triée numériquement
    modelFiles = modelFiles.OrderBy(x => int.Parse(x, System.Globalization.NumberStyles.Any)).ToArray();

    // Avant d'incrémenter, vérifiez si nous sommes déjà au maximum
    if (currentModelIndex >= modelFiles.Length) {
        currentModelIndex = 0; // Réinitialisation pour sécurité
    }

    // Appliquer l'increment step et ajuster pour bouclage
    currentModelIndex = (currentModelIndex + incrementStep + modelFiles.Length) % modelFiles.Length;

    string modelName = modelFiles[currentModelIndex];
    Debug.Log("Loading model: " + modelName);
    textGeneration.text = "Generation #" + modelName;

    NNModel model = Resources.Load<NNModel>($"{modelsFolder}/{modelName}");
    if (model == null)
    {
        Debug.LogError("Failed to load model: " + modelName);
        return;
    }

    behaviorParameters.Model = model;
}

    public void OnEpisodeEnd()
    {
        LoadModel();
    }

    public void Update()
    {
        if (Input.GetKeyDown(KeyCode.Home))
        {
            LoadModel(1);
            GetComponent<RobotWalk>().EndEpisode();
        }
        if (Input.GetKeyDown(KeyCode.End))
        {
            LoadModel(10);
            GetComponent<RobotWalk>().EndEpisode();
        }
        if (Input.GetKeyDown(KeyCode.PageUp))
        {
            LoadModel(-1);
            GetComponent<RobotWalk>().EndEpisode();
        }
        if (Input.GetKeyDown(KeyCode.PageDown))
        {
            LoadModel(-10);
            GetComponent<RobotWalk>().EndEpisode();
        }
    }
}
