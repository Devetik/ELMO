using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class TestLearning : Agent
{
    public Transform Agent;
    public Transform Target;

    public override void OnEpisodeBegin()
    {
        Agent.localPosition = new Vector3(Random.Range(-1,-4), 0f,Random.Range(-1,-4));
        Target.localPosition = new Vector3(Random.Range(1,4), 1.25f,Random.Range(1,4));
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(Agent.localPosition);
        sensor.AddObservation(Target.localPosition);
    }
    public override void OnActionReceived(ActionBuffers action)
    {
        float moveX = action.ContinuousActions[0];
        float moveZ = action.ContinuousActions[1];

        // Agent.GetComponent<Rigidbody>().MovePosition(transform.position + transform.forward * moveForward *15 * Time.deltaTime);
        // Agent.Rotate(0f, moveRotate, 0f, Space.Self);
        // Agent.GetComponent<Rigidbody>().MovePosition(transform.position + transform.moveX * Time.deltaTime);
        transform.localPosition += new Vector3(moveX, 0f, moveZ)* Time.deltaTime * 10;
    }
    private void OnTriggerEnter(Collider other)
    {
        if(other.gameObject.tag == "Target")
        {
            AddReward(10f);
            EndEpisode();
        }
        if(other.gameObject.tag == "Walls")
        {
            AddReward(-1f);
            EndEpisode();
        }
    }

}
