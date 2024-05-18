using UnityEngine;
using Random = UnityEngine.Random;
using Unity.MLAgents;
using UnityEngine.Events;
using System;
using Unity.VisualScripting;

namespace Unity.MLAgentsExamples
{
    /// <summary>
    /// Utility class to allow target placement and collision detection with an agent
    /// Add this script to the target you want the agent to touch.
    /// Callbacks will be triggered any time the target is touched with a collider tagged as 'tagToDetect'
    /// </summary>
    public class TargetController : MonoBehaviour
    {

        [Header("Collider Tag To Detect")]
        public string tagToDetect = "agent"; //collider tag to detect

        [Header("Target Placement")]
        public float minSpawnRadius; //The radius in which a target can be randomly spawned.
        public float maxSpawnRadius;
        public bool respawnIfTouched; //Should the target respawn to a different position when touched

        [Header("Target Fell Protection")]
        public bool respawnIfFallsOffPlatform = true; //If the target falls off the platform, reset the position.
        public float fallDistance = 10; //distance below the starting height that will trigger a respawn

        private Vector3 m_startingPos; //the starting position of the target
        public Transform agentPosition;

        [System.Serializable]
        public class TriggerEvent : UnityEvent<Collider>
        {
        }

        [Header("Trigger Callbacks")]
        public TriggerEvent onTriggerEnterEvent = new TriggerEvent();
        public TriggerEvent onTriggerStayEvent = new TriggerEvent();
        public TriggerEvent onTriggerExitEvent = new TriggerEvent();

        [System.Serializable]
        public class CollisionEvent : UnityEvent<Collision>
        {
        }

        [Header("Collision Callbacks")]
        public CollisionEvent onCollisionEnterEvent = new CollisionEvent();
        public CollisionEvent onCollisionStayEvent = new CollisionEvent();
        public CollisionEvent onCollisionExitEvent = new CollisionEvent();
        public RobotWalk robotWalk;

        // Start is called before the first frame update
        void OnEnable()
        {
            m_startingPos = transform.position;
            if (respawnIfTouched)
            {
                MoveTargetToRandomPosition();
            }
        }

        void Update()
        {
            if (respawnIfFallsOffPlatform)
            {
                if (transform.position.y < m_startingPos.y - fallDistance)
                {
                    Debug.Log($"{transform.name} Fell Off Platform");
                    MoveTargetToRandomPosition();
                }
            }
        }

        /// <summary>
        /// Moves target to a random position within specified radius.
        /// </summary>
        public void MoveTargetToRandomPosition()
        {
            
            // var newTargetPos = m_startingPos + (Random.insideUnitSphere * Random.Range(minSpawnRadius, maxSpawnRadius));
            // newTargetPos.y = m_startingPos.y;
            // transform.position = newTargetPos;

            // // Appliquer la nouvelle position
            // transform.position = newTargetPos;



            // Définir une nouvelle position x, y reste la hauteur maximale pour le raycasting
            var newTargetPos = m_startingPos;
            newTargetPos.x = m_startingPos.x + Random.Range(-10, 10);
            newTargetPos.z = m_startingPos.z + Random.Range(-10, 10);

            // Définir une hauteur maximale de départ pour le raycast si nécessaire
            float maxRaycastHeight = 5.0f; // Ajustez cette valeur en fonction de la configuration de votre scène

            // Créer un point haut pour le départ du rayon
            Vector3 rayStart = new Vector3(newTargetPos.x, maxRaycastHeight, newTargetPos.z);
            RaycastHit hit;

            // Effectuer le raycast vers le bas
            if (Physics.Raycast(rayStart, Vector3.down, out hit))
            {
                // Si le rayon touche quelque chose, définir la position y au point de contact
                newTargetPos.y = hit.point.y + 1.0f; // Ajoutez une petite marge pour éviter le clipping avec le sol
            }
            else
            {
                // Si rien n'est touché, utilisez la position y de startingPos ou une valeur par défaut
                newTargetPos.y = m_startingPos.y;
            }

            // Appliquer la nouvelle position
            transform.position = newTargetPos;

        }

        private void OnCollisionEnter(Collision col)
        {
            if (col.transform.CompareTag(tagToDetect))
            {
                onCollisionEnterEvent.Invoke(col);
                if (robotWalk != null)
                {
                    robotWalk.CatchTarget(); // Appeler la méthode TargetCatch
                }
                if (respawnIfTouched)
                {
                    MoveTargetToRandomPosition();
                }
            }
        }

        private void OnCollisionStay(Collision col)
        {
            if (col.transform.CompareTag(tagToDetect))
            {
                onCollisionStayEvent.Invoke(col);
            }
        }

        private void OnCollisionExit(Collision col)
        {
            if (col.transform.CompareTag(tagToDetect))
            {
                onCollisionExitEvent.Invoke(col);
            }
        }

        private void OnTriggerEnter(Collider col)
        {
            if (col.CompareTag(tagToDetect))
            {
                onTriggerEnterEvent.Invoke(col);
            }
        }

        private void OnTriggerStay(Collider col)
        {
            if (col.CompareTag(tagToDetect))
            {
                onTriggerStayEvent.Invoke(col);
            }
        }

        private void OnTriggerExit(Collider col)
        {
            if (col.CompareTag(tagToDetect))
            {
                onTriggerExitEvent.Invoke(col);
            }
        }
    }
}
