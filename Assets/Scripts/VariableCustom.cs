using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VariableCustom 
{
    private RobotWalk agent;
    private StepReward stepReward;
    public Vector3 positionCoucheFace = new Vector3(0,2.5f,-40);
    public Quaternion rotationCoucheFace = Quaternion.Euler(UnityEngine.Random.Range(80,100), UnityEngine.Random.Range(-15,15), UnityEngine.Random.Range(-15,15));
    public Vector3 positionDebout = new Vector3(0,0.05f,-40);
    public Quaternion rotationDebout = Quaternion.Euler(UnityEngine.Random.Range(15,-15), UnityEngine.Random.Range(-15,15), UnityEngine.Random.Range(-15,15));
    public Vector3 positionCoucheDos = new Vector3(0,3,-40);
    public Quaternion rotationCoucheDos = Quaternion.Euler(UnityEngine.Random.Range(-90,-90), UnityEngine.Random.Range(0,0), UnityEngine.Random.Range(0,0));
    public Vector3 positionRandom = new Vector3(0,5,-40);
    public Quaternion rotationRandom = Quaternion.Euler(UnityEngine.Random.Range(-90,90), UnityEngine.Random.Range(-90,90), UnityEngine.Random.Range(-90,90));
    public VariableCustom(RobotWalk agent)
    {
        this.agent = agent;
    }

    public VariableCustom(StepReward stepReward)
    {
        this.stepReward = stepReward;
    }



}
