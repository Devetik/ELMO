// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;
// //using System;
// using UnityEngine.UI;
// using Unity.MLAgents;
// using Unity.MLAgents.Actuators;
// using Unity.MLAgents.Sensors;

// public class TestConfigJoint : Agent
// {
//     public Transform Env; // Assignez cela dans l'éditeur Unity
//     private BodyPartManager bodyPartManager;
//     private BodyPartManager envPartManager;
//     public Transform Body;
//     public Transform CenterBody;
//     public Transform Leg_D_1;
//     public Transform Leg_D_2;
//     public Transform Leg_D_3;
//     public Transform Leg_G_1;
//     public Transform Leg_G_2;
//     public Transform Leg_G_3;
//     public ConfigurableJoint Config_Joint_D_1; 
//     public ConfigurableJoint Config_Joint_D_2;
//     public ConfigurableJoint Config_Joint_D_3;
//     public ConfigurableJoint Config_Joint_G_1;
//     public ConfigurableJoint Config_Joint_G_2;
//     public ConfigurableJoint Config_Joint_G_3;
//     private Vector3 Origin_Body;
//     private Quaternion startRotation;
    

//     [SerializeField] private Transform target;
//     [SerializeField] private Transform target1;
//     [SerializeField] private Transform target2;
//     [SerializeField] private Transform target3;
//     private float maxRotation = 100f;
//     private float timeElapsed = 0f;
//     private float totalTime = 200f;
//     private Vector3 posOnStart;
//     private Vector3 lastPosition;
//     private float SlackRewardPenalty;
//     private float timeAccumulator = 0f;
//     public float actionInterval = 5f;

//     public override void Initialize()
//     {
//         bodyPartManager = new BodyPartManager(Body);
//         envPartManager = new BodyPartManager(Env);
//         //target.localPosition = new Vector3(Random.Range(-40f,40f), 1f,Random.Range(-40f,40f));
//     }
//     public override void OnEpisodeBegin()
//     {
//         timeElapsed = 0f;
//         SlackRewardPenalty = 1f;
//         target1.gameObject.SetActive(true);
//         target2.gameObject.SetActive(true);
//         target3.gameObject.SetActive(true);
//         // target1.localPosition = new Vector3(0f, 3f,-8f);
//         // target2.localPosition = new Vector3(0f, 3f,3.7f);
//         // target3.localPosition = new Vector3(0f, 3f,23f);
//         // target.localPosition = new Vector3(0f, 3f,43f);
//         // Body.localPosition = new Vector3(0f,2f,0f);
//         // target.localPosition = new Vector3(Random.Range(-40f,40f), 1f,Random.Range(-40f,40f));
//         bodyPartManager.ResetParts();
//         envPartManager.ResetParts();
//         posOnStart = CenterBody.localPosition;
//         startRotation = CenterBody.rotation;
//     }


//     public override void OnActionReceived(ActionBuffers actionBuffers)
//     {
//         ApplyRotation(Config_Joint_D_1, actionBuffers.ContinuousActions[0],actionBuffers.ContinuousActions[1]);
//         ApplyRotation(Config_Joint_G_1, actionBuffers.ContinuousActions[2],actionBuffers.ContinuousActions[3]);
//         ApplyRotation(Config_Joint_D_2, actionBuffers.ContinuousActions[4],actionBuffers.ContinuousActions[5]);
//         ApplyRotation(Config_Joint_G_2, actionBuffers.ContinuousActions[6],actionBuffers.ContinuousActions[7]);
//         ApplyRotation(Config_Joint_D_3, actionBuffers.ContinuousActions[8],actionBuffers.ContinuousActions[9]);
//         ApplyRotation(Config_Joint_G_3, actionBuffers.ContinuousActions[10],actionBuffers.ContinuousActions[11]);
//         timeElapsed += Time.fixedDeltaTime;
        

//         timeAccumulator += Time.deltaTime;

//         if (timeAccumulator >= actionInterval)
//         {
//             float distance = Vector3.Distance(lastPosition, CenterBody.localPosition);
//             float distanceZ = Mathf.Abs(lastPosition.z - CenterBody.localPosition.z);
//             timeAccumulator %= actionInterval;
//             if(distanceZ < 0.1)
//             {
//                 SlackRewardPenalty -= 0.75f;
//             }

//             lastPosition = CenterBody.localPosition;
//         }


        
//         if (timeElapsed >= totalTime)
//         {
//             float distanceInitial = Vector3.Distance(posOnStart, target.localPosition);
//             float distanceFinal = Vector3.Distance(CenterBody.localPosition, target.localPosition);
//             float targetProximityReward = (distanceInitial - distanceFinal);
//             //Debug.Log($"Reward: {targetProximityReward}");
//             float angleDifference = Quaternion.Angle(transform.rotation, startRotation);
//             float anglePenalty = (-angleDifference / 180f)*3;
//             Debug.Log($"anglePenalty: {anglePenalty+1} SlackRewardPenalty:{SlackRewardPenalty} Reward:{targetProximityReward * (anglePenalty + 1) * SlackRewardPenalty} ");
//             AddReward(targetProximityReward * System.Math.Max(0,(anglePenalty + 1)) * System.Math.Max(SlackRewardPenalty, 0) + targetProximityReward / 2);
//             EndEpisode();
//         }
//     }

//     void ApplyRotation(ConfigurableJoint joint, float actionValueX, float actionValueY)
//     {
//         // Création d'une nouvelle configuration de drive
//         JointDrive drive = new JointDrive();
//         drive.positionSpring = 150; // La rigidité du ressort
//         drive.positionDamper = 0; // L'amortissement
//         drive.maximumForce = 1000; // La force maximale que le drive peut appliquer

//         // Application de la configuration de drive à angularXDrive du joint
//         joint.angularXDrive = drive;
//         joint.angularYZDrive = drive;

//         // Définition de la targetPosition du drive pour appliquer une rotation
//         // Notez que cette méthode est plus symbolique et sert à illustrer le concept.
//         // Vous devrez adapter votre logique pour convertir actionValueX, actionValueY, actionValueZ en une cible viable pour votre scénario.
//         var targetRotation = new Vector3(actionValueX, actionValueY) * maxRotation;
//         joint.targetRotation = Quaternion.Euler(targetRotation);
//     }

//     public override void Heuristic(in ActionBuffers actionsOut)
//     {
//         var continuousActionsOut = actionsOut.ContinuousActions;

//         if (Input.GetKey(KeyCode.UpArrow))
//         {
//             continuousActionsOut[0] = 1.0f; // Action pour lever la patte avant gauche
//             continuousActionsOut[2] = 1.0f; // Action pour lever la patte avant gauche
//             continuousActionsOut[4] = 1.0f;
//             continuousActionsOut[6] = 1.0f;
//             continuousActionsOut[8] = 1.0f;
//             continuousActionsOut[10] = 1.0f;
//         }
//         else if (Input.GetKey(KeyCode.DownArrow))
//         {
//             continuousActionsOut[0] = -1.0f; // Action pour abaisser la patte avant gauche
//             continuousActionsOut[2] = -1.0f; // Action pour abaisser la patte avant gauche
//             continuousActionsOut[4] = -1.0f;
//             continuousActionsOut[6] = -1.0f;
//             continuousActionsOut[8] = -1.0f;
//             continuousActionsOut[10] = -1.0f;
//         }

//         if (Input.GetKey(KeyCode.RightArrow))
//         {
//             continuousActionsOut[1] = 1.0f; // Action pour lever la patte avant droite
//             continuousActionsOut[3] = 1.0f; // Action pour lever la patte avant droite
//             continuousActionsOut[5] = 1.0f;
//             continuousActionsOut[7] = 1.0f;
//             continuousActionsOut[9] = 1.0f;
//             continuousActionsOut[11] = 1.0f;
//         }
//         else if (Input.GetKey(KeyCode.LeftArrow))
//         {
//             continuousActionsOut[1] = -1.0f; // Action pour abaisser la patte avant droite
//             continuousActionsOut[3] = -1.0f; // Action pour lever la patte avant droite
//             continuousActionsOut[5] = -1.0f;
//             continuousActionsOut[7] = -1.0f;
//             continuousActionsOut[9] = -1.0f;
//             continuousActionsOut[11] = -1.0f;
//         }
//     }
//     public override void CollectObservations(VectorSensor sensor)
//     {
//         sensor.AddObservation(transform.localPosition);
//         sensor.AddObservation(Leg_D_1.localPosition);
//         sensor.AddObservation(Leg_D_2.localPosition);
//         sensor.AddObservation(Leg_D_3.localPosition);
//         sensor.AddObservation(Leg_G_1.localPosition);
//         sensor.AddObservation(Leg_G_2.localPosition);
//         sensor.AddObservation(Leg_G_3.localPosition);
//         sensor.AddObservation(CenterBody.localPosition);
//         sensor.AddObservation(CenterBody.GetComponent<Rigidbody>().velocity);
//         sensor.AddObservation(Leg_D_1.GetComponent<Rigidbody>().velocity);
//         sensor.AddObservation(Leg_D_2.GetComponent<Rigidbody>().velocity);
//         sensor.AddObservation(Leg_D_3.GetComponent<Rigidbody>().velocity);
//         sensor.AddObservation(Leg_G_1.GetComponent<Rigidbody>().velocity);
//         sensor.AddObservation(Leg_G_1.GetComponent<Rigidbody>().velocity);
//         sensor.AddObservation(Leg_G_1.GetComponent<Rigidbody>().velocity);
//     }
//     private void OnTriggerEnter(Collider other)
//     {
//         if(other.gameObject.tag == "Target3")
//         {
//             AddReward(1f);
//             target3.gameObject.SetActive(false);
//         }
//         if(other.gameObject.tag == "Target2")
//         {
//             AddReward(2f);
//             target2.gameObject.SetActive(false);
//         }
//         if(other.gameObject.tag == "Target1")
//         {
//             AddReward(30f);
//             target1.gameObject.SetActive(false);
//         }
//         if(other.gameObject.tag == "Target")
//         {
//             AddReward(50f);
//             target.localPosition = new Vector3(Random.Range(-40f,40f), 1f,Random.Range(-40f,40f));
//             EndEpisode();
//         }
//         if(other.gameObject.tag == "Walls")
//         {
//             AddReward(-25f);
//             EndEpisode();
//         }
//     }

// }
