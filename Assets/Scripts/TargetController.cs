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
        public float fallDistance = 5; //distance below the starting height that will trigger a respawn

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
            int orientation = Random.Range(1,4);
            Vector3 newTargetPos = m_startingPos;
            //Vector3 newTargetPos = agentPosition.position;
            
            newTargetPos.y = m_startingPos.y;
            if(orientation <=2)//spwan en bas
            {
                newTargetPos.z +=  Random.Range(minSpawnRadius, maxSpawnRadius);
                if(orientation == 1)//spawn à gauche
                {
                    newTargetPos.x -= Random.Range(minSpawnRadius, maxSpawnRadius);
                }
                else//spwn à droite
                {
                    newTargetPos.x += Random.Range(minSpawnRadius, maxSpawnRadius);
                }
            }
            else//spawn en haut
            {
                newTargetPos.z -=  Random.Range(minSpawnRadius, maxSpawnRadius);
                if(orientation == 3)//spawn à gauche
                {
                    newTargetPos.x -= Random.Range(minSpawnRadius, maxSpawnRadius);
                }
                else//spwn à droite
                {
                    newTargetPos.x += Random.Range(minSpawnRadius, maxSpawnRadius);
                }
            }

            // var newTargetPos = m_startingPos + (Random.insideUnitSphere * Random.Range(minSpawnRadius, maxSpawnRadius));
            // newTargetPos.y = m_startingPos.y;
            // Debug.Log(m_startingPos + " / "+ newTargetPos);
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
