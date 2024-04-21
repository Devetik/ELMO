using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class AgentTest : Agent
{
    //Ajout de chaque pattes avec leur joint respectif
    public Transform front_Left_Leg;
    public HingeJoint front_Left_Leg_Joint; 
    public Transform front_Right_Leg;
    public HingeJoint front_Right_Leg_Joint; 
    public Transform back_Left_Leg;
    public HingeJoint back_Left_Leg_Joint; 
    public Transform back_Right_Leg;
    public HingeJoint back_Right_Leg_Joint; 


    private float maxVelocity = 2000f; // Ajustez selon le besoin
    private float maxForce = 100f; // Ajustez selon le besoin
    [SerializeField] private Transform target;

    public Transform fullBody;
    private Vector3 posOnStart;
    private float timeElapsed = 0f;
    private float totalTime = 30f;


    public override void Initialize()
    {
        // var motor = jointCuisse.motor;
        // motor.targetVelocity = 2000f;
        target.localPosition = new Vector3(Random.Range(-40f,40f), 23f,Random.Range(-40f,40f));
    }
    public override void OnEpisodeBegin()
    {
        timeElapsed = 0f;
        
        posOnStart = fullBody.localPosition;
        //fullBody.localPosition = new Vector3(0f, 23f,0f);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        ApplyMotorForce(front_Left_Leg_Joint, actionBuffers.ContinuousActions[0]);
        ApplyMotorForce(front_Right_Leg_Joint, actionBuffers.ContinuousActions[1]);
        // Supposons que vous avez des références pour les cuisses gauche et droite, et tibias gauche et droite
        ApplyMotorForce(back_Left_Leg_Joint, actionBuffers.ContinuousActions[2]);
        ApplyMotorForce(back_Right_Leg_Joint, actionBuffers.ContinuousActions[3]);
        // float targetVelocity = actionBuffers.ContinuousActions[0] * maxVelocity;
        // var motor = jointCuisse.motor;
        // motor.targetVelocity = targetVelocity;
        // motor.force = maxForce;
        // jointCuisse.motor = motor;
        // jointCuisse.useMotor = true;
        Debug.Log(timeElapsed);
        timeElapsed += Time.fixedDeltaTime;
        if (timeElapsed >= totalTime)
        {
            float distance = Vector3.Distance(posOnStart, fullBody.position);
            AddReward(distance);
            EndEpisode();
        }
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
        // Initialiser toutes les actions à 0.0f
        for (int j = 0; j < continuousActionsOut.Length; j++)
        {
            continuousActionsOut[j] = 0.0f;
        }

        if (Input.GetKey(KeyCode.UpArrow))
        {
            continuousActionsOut[0] = 1.0f; // Action pour lever la patte avant gauche
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            continuousActionsOut[0] = -1.0f; // Action pour abaisser la patte avant gauche
        }

        if (Input.GetKey(KeyCode.RightArrow))
        {
            continuousActionsOut[1] = 1.0f; // Action pour lever la patte avant droite
        }
        else if (Input.GetKey(KeyCode.LeftArrow))
        {
            continuousActionsOut[1] = -1.0f; // Action pour abaisser la patte avant droite
        }

        if (Input.GetKey(KeyCode.A))
        {
            continuousActionsOut[2] = 1.0f; // Action pour lever la patte avant gauche
        }
        else if (Input.GetKey(KeyCode.D))
        {
            continuousActionsOut[2] = -1.0f; // Action pour abaisser la patte avant gauche
        }

        if (Input.GetKey(KeyCode.W))
        {
            continuousActionsOut[3] = 1.0f; // Action pour lever la patte avant droite
        }
        else if (Input.GetKey(KeyCode.S))
        {
            continuousActionsOut[3] = -1.0f; // Action pour abaisser la patte avant droite
        }
        //Debug.Log($"Action: {continuousActionsOut[0]}");
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        // Exemple: Ajouter l'angle actuel du joint comme observation
        //sensor.AddObservation(jointCuisse.angle);
        sensor.AddObservation(front_Left_Leg_Joint.angle);
        sensor.AddObservation(front_Right_Leg_Joint.angle);
        sensor.AddObservation(back_Left_Leg_Joint.angle);
        sensor.AddObservation(back_Right_Leg_Joint.angle);
    }

    private void OnTriggerEnter(Collider other)
    {
        if(other.gameObject.tag == "Target")
        {
            AddReward(50f);
            target.localPosition = new Vector3(Random.Range(-40f,40f), 23f,Random.Range(-40f,40f));
            EndEpisode();
        }
    }

}
