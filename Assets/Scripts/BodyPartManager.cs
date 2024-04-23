// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;

// [System.Serializable]
// public class BodyPartManager : MonoBehaviour
// {
//     private List<Transform> parts = new List<Transform>();
//     private Dictionary<Transform, (Vector3 position, Quaternion rotation, bool isActive)> initialState = new Dictionary<Transform, (Vector3, Quaternion, bool)>();
//     public void Initialize(Transform parent)
//     {
//         foreach (Transform child in parent)
//         {
//             parts.Add(child);
//             initialState[child] = (child.localPosition, child.localRotation, child.gameObject.activeSelf);
//         }
//     }

//     IEnumerator ReleaseKinematic(Rigidbody partie, float time)
//     {
//         yield return new WaitForSeconds(time); // Attendez une seconde
//         partie.isKinematic = false; // Permet au torse d'être à nouveau affecté par la physique
//     }
//     public void ResetParts(MonoBehaviour monoBehaviour)
//     {
//         foreach (var part in parts)
//         {
//             var state = initialState[part];
//             part.localPosition = state.position;
//             part.localRotation = state.rotation;
//             part.gameObject.SetActive(state.isActive);

//             var rb = part.GetComponent<Rigidbody>();
//             if (rb != null && rb.isKinematic == false)
//             {
//                 rb.velocity = Vector3.zero;
//                 if(rb.isKinematic == false)
//                 {
//                     rb.angularVelocity = Vector3.zero;
//                 }
//                 //////////////////////
//                 rb.isKinematic = true; // Optionnel, dépend du besoin
//                 //Une coroutine pour réactiver la physique pourrait être lancée ici si nécessaire.
//                 StartCoroutine(ReleaseKinematic(rb, 0.05f));
//             }
//         }
//     }
// }



// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;

// [System.Serializable]
// public class BodyPartManager : MonoBehaviour
// {
//     public Transform parent; // Définissez ceci dans l'éditeur Unity pour spécifier le parent principal des parties du corps

//     public void Initialize(Transform parent)
//     {
//         this.parent = parent;
//     }

//     public void ResetParts(Vector3 newPosition, Quaternion newRotation)
//     {
//         if (parent != null)
//         {
//             parent.position = newPosition;
//             parent.rotation = newRotation;
            
//             // Réinitialisez également les Rigidbody pour éviter des mouvements physiques indésirables
//             Rigidbody[] rigidbodies = parent.GetComponentsInChildren<Rigidbody>();
//             foreach (var rb in rigidbodies)
//             {
//                 if (rb != null && rb.isKinematic == false)
//                 {
//                     rb.velocity = Vector3.zero;
//                     rb.angularVelocity = Vector3.zero;
//                     rb.isKinematic = true; // Temporairement cinématique pour stabiliser la réinitialisation
//                     StartCoroutine(ReleaseKinematic(rb, 0.05f)); // Reactivate physics shortly afterwards
//                 }
//             }
//         }
//     }

//     IEnumerator ReleaseKinematic(Rigidbody rb, float time)
//     {
//         yield return new WaitForSeconds(time);
//         rb.isKinematic = false;
//     }
// }
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class BodyPartManager : MonoBehaviour
{
    private Transform parent; // Le parent principal des parties du corps
    private Vector3 initialParentPosition;
    private Quaternion initialParentRotation;
    private Dictionary<Transform, (Vector3 position, Quaternion rotation)> initialRelativeStates = new Dictionary<Transform, (Vector3, Quaternion)>();

    public void Initialize(Transform parent)
    {
        this.parent = parent;
        initialParentPosition = parent.position;
        initialParentRotation = parent.rotation;

        foreach (Transform child in parent)
        {
            // Stocker l'état relatif initial de chaque enfant par rapport au parent
            initialRelativeStates[child] = (child.localPosition, child.localRotation);
        }
    }

    public void ResetParts(Vector3 newPosition, Quaternion newRotation)
    {
        if (parent != null)
        {
            // Déplacer et faire pivoter le parent à la nouvelle position et orientation

            // Réappliquer les états relatifs initiaux des enfants
            foreach (var child in initialRelativeStates)
            {
                // child.Key.localPosition = child.Value.position;
                //child.Key.localRotation = child.Value.rotation;

                // Réinitialiser les Rigidbody de manière sûre
                var rb = child.Key.GetComponent<Rigidbody>();
                child.Key.localPosition = child.Value.position;
                if (rb != null && rb.isKinematic == false)
                {
                    // rb.velocity = Vector3.zero;
                    // rb.angularVelocity = Vector3.zero;
                    rb.isKinematic = true; // Rendre cinématique temporairement
                    child.Key.localRotation = child.Value.rotation;
                    StartCoroutine(ReleaseKinematic(rb, 0.05f));
                }
            }
            parent.localPosition = newPosition;
            parent.rotation = newRotation;
        }
    }

    IEnumerator ReleaseKinematic(Rigidbody rb, float time)
    {
        yield return new WaitForSeconds(time);
        rb.isKinematic = false;
    }
}
