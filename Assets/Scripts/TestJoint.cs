using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class TestJoint : Agent
{
    public HingeJoint Test_Joint; 
    private float maxVelocity = 2000f; // Ajustez selon le besoin
    private float maxForce = 100f; // Ajustez selon le besoin

    public override void Initialize()
    {

    }
    public override void OnEpisodeBegin()
    {


    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        ApplyMotorForce(Test_Joint, actionBuffers.ContinuousActions[0]);
    }

    void ApplyMotorForce(HingeJoint joint, float actionValue)
    {
        var motor = joint.motor;
        motor.targetVelocity = actionValue * maxVelocity;
        motor.force = maxForce;
        joint.motor = motor;
        joint.useMotor = true;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
         var continuousActionsOut = actionsOut.ContinuousActions;
        // //Initialiser toutes les actions à 0.0f;
        // for (int j = 0; j < continuousActionsOut.Length; j++)
        // {
        //     continuousActionsOut[j] = 0.0f;
        // }
        // //Debug.Log($"Action: {continuousActionsOut[0]}");

         if (Input.GetKey(KeyCode.UpArrow))
        {
            continuousActionsOut[0] = 1.0f; // Action pour lever la patte avant gauche
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            continuousActionsOut[0] = -1.0f; // Action pour abaisser la patte avant gauche
        }

        // if (Input.GetKey(KeyCode.RightArrow))
        // {
        //     continuousActionsOut[1] = 1.0f; // Action pour lever la patte avant droite
        // }
        // else if (Input.GetKey(KeyCode.LeftArrow))
        // {
        //     continuousActionsOut[1] = -1.0f; // Action pour abaisser la patte avant droite
        // }

        // if (Input.GetKey(KeyCode.A))
        // {
        //     continuousActionsOut[6] = 1.0f; // Action pour lever la patte avant gauche
        // }
        // else if (Input.GetKey(KeyCode.D))
        // {
        //     continuousActionsOut[6] = -1.0f; // Action pour abaisser la patte avant gauche
        // }

        // if (Input.GetKey(KeyCode.W))
        // {
        //     continuousActionsOut[3] = 1.0f; // Action pour lever la patte avant droite
        // }
        // else if (Input.GetKey(KeyCode.S))
        // {
        //     continuousActionsOut[3] = -1.0f; // Action pour abaisser la patte avant droite
        // }
    }
    public override void CollectObservations(VectorSensor sensor)
    {

    }

}
