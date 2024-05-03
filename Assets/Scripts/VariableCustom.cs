using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VariableCustom 
{
    public int shoulderL1;
    public int shoulderR1;
    public int shoulderL2;
    public int shoulderR2;
    public int shoulderL3;
    public int shoulderR3;
    public int coudeL;
    public int coudeR;
    public int cuisseL1;
    public int cuisseR1;
    public int cuisseL2;
    public int cuisseR2;
    public int cuisseL3;
    public int cuisseR3;
    public int tibiasL;
    public int tibiasR;
    public int torse1;
    public int torse2;
    public int torse3;
    public int abdo1;
    public int abdo2;
    public int abdo3;
    public int head1;
    public int head2;
    public int head3;
    public int footL1;
    public int footL2;
    public int footL3;
    public int footR1;
    public int footR2;
    public int footR3;
    public int PecL;
    public int PecR;

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
    public enum PositionType
    {
        coucheFace,
        coucheDos,
        debout,
        deboutRandom,
        deboutDroit,
        random
    }

    public (Vector3, Quaternion) GetRandomPosition(PositionType positiontype)
    {
        if(positiontype == PositionType.random)
        {
            Vector3 positionRandom = new Vector3(0,3,-40);
            Quaternion rotationRandom = Quaternion.Euler(UnityEngine.Random.Range(-90,90), UnityEngine.Random.Range(-45,45), UnityEngine.Random.Range(-45,45));
            return (positionRandom, rotationRandom);
        }
        if(positiontype == PositionType.debout)
        {
            Vector3 positionRandom = new Vector3(0,0.3f,5);
            Quaternion rotationRandom = Quaternion.Euler(UnityEngine.Random.Range(-5,5), UnityEngine.Random.Range(0,0), UnityEngine.Random.Range(0,0));
            return (positionRandom, rotationRandom);
        }
        if(positiontype == PositionType.deboutRandom)
        {
            Vector3 positionRandom = new Vector3(UnityEngine.Random.Range(-10,10),0.3f,UnityEngine.Random.Range(-25,-50));
            Quaternion rotationRandom = Quaternion.Euler(UnityEngine.Random.Range(-5,5), UnityEngine.Random.Range(0,360), UnityEngine.Random.Range(-5,5));
            return (positionRandom, rotationRandom);
        }
        if(positiontype == PositionType.deboutDroit)
        {
            Vector3 positionRandom = new Vector3(0,0.25f,5);
            Quaternion rotationRandom = Quaternion.Euler(0, 0, 0);
            return (positionRandom, rotationRandom);
        }
        return (positionDebout,rotationDebout);
    }



}
