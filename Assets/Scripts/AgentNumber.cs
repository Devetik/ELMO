using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class AgentNumber : MonoBehaviour
{
    public static int nextAgentId = 1;  // Compteur statique pour suivre l'ID du prochain agent
    public TextMeshPro dossardFront;  // Référence au composant TextMeshPro de cet agent
    public TextMeshPro dossardBack;
    // Start is called before the first frame update
    void Start()
    {
        int agentId = nextAgentId++;
        dossardFront.text = "|| " + agentId.ToString("D2")+" ||";  // "D2" formate le numéro avec deux chiffres, par exemple "01", "02", etc.
        dossardBack.text = agentId.ToString("D2");
    }
}
