using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;

public class StepReward 
{
    private Vector3 positionCoucheFace = new Vector3(0,5,-20);
    private quaternion rotationCoucheFace ;
    private Vector3 positionDebout;
    private quaternion rotationDebout;
    private Vector3 positionCoucheDos;
    private quaternion rotationCoucheDos;
    private bool firstStepIn = false;
    private bool secondStepIn = false;
    private bool thirdStepIn = false;
    private bool stepQuatre = false;
    private bool stabilityStep = false;
    private RobotWalk agent;
    private VariableCustom vc;
    public StepReward(RobotWalk agent)
    {
        this.agent = agent;
        vc = new VariableCustom(this);
    }

    public void RiseBassin()
    {
        if(!firstStepIn)
        {
            Debug.Log("Step 0 RiseBassin");
            firstStepIn = true;
        }
        float bonus = 0f;

        float bassinHeight = agent.Bassin.position.y - ((agent.Foot_LEFT.position.y + agent.Foot_RIGHT.position.y)/2);

        bonus += bassinHeight;
        bonus += firstStepHandSupport();

        agent.reward(bonus);
    }

    public void RiseHead()
    {
        if(!firstStepIn)
        {
            Debug.Log("Step 0 RiseHead");
            firstStepIn = true;
        }
        float bonus = 0f;

        bonus += FirstStepHeadOnTop(penality:0, baseValue:1, timeExpo:false);
        float headHeight = Mathf.Abs(agent.Head.position.y - ((agent.Foot_LEFT.position.y + agent.Foot_RIGHT.position.y)/2));
        if(headHeight > 3.6f)
        {
            agent.reward(50f);
            agent.EndEpisode();
        }

        bonus += firstStepHandSupport();

        agent.reward(bonus);
    }

    public void CenterOfGravity()
    {
        if(!secondStepIn)
        {
            Debug.Log("Step 1 CoG");
            secondStepIn = true;
        }

        float bonus = 0f;
        // if(agent.faceOnTheFloor == 0 || agent.faceOnTheFloor == 1)
        // {

            //bonus += firstStepHandSupport()/agent.MaxStep;
            bonus += agent.CenterOfGravityReward()*2;
            // if(bonus >= 2f)
            // {
                bonus += agent.simpleHeadOnTop(penality:0, baseValue:1, timeExpo:false);
                float headHeight = Mathf.Abs(agent.Head.position.y - ((agent.Foot_LEFT.position.y + agent.Foot_RIGHT.position.y)/2));
                if(headHeight > 3.7f && (agent.leftFootContact.LeftFootOnFloor || agent.rightFootContact.RightFootOnFloor))
                {
                    Debug.Log("JACKPOT !!!");
                    agent.reward(500f);
                    //agent.EndEpisode();
                }
            // }
        // }
        // else
        // {
        //     agent.EndEpisode();
        //     //bonus += simpleHeadOnTop(penality:0, baseValue:1, timeExpo:false);
        // }
        bonus += firstStepHandSupport();
        agent.reward(bonus);
    }

    public void RiseFromBack()
    {
        float bonus = 0f;
        bonus += firstStepHandSupport();
        bonus += Math.Min(1, (agent.Head.position.y-1.6f) / 3.2f);//4.8f   0.29
        agent.reward(bonus);
        if(agent.OrientationTorseFloor() == 1)
        {
            agent.reward(100f);
            Debug.Log("Il s'est RETOURNE !!!");
            agent.EndEpisode();
        }
    }

    public void RiseBody()
    {
        if(!thirdStepIn)
        {
            Debug.Log("Step 2 Rise Body");
            thirdStepIn = true;
        }

        float bonus = 0f;
        bonus += agent.simpleHeadOnTop(penality:0, baseValue:3, timeExpo:false);
        bonus += CenterOfGravityRiseReward()/3;
        agent.reward(bonus);
    }


    public void Walk()
    {
        if(!stepQuatre)
        {
            Debug.Log("Step 3 Walk");
            stepQuatre = true;
        }

        float bonus = 0f;
        bonus += agent.isWalkingForward();
        bonus += Math.Min(1, (agent.Head.position.y-1.6f) / 3.2f);
        bonus += agent.DistanceToTarget();

        agent.reward(bonus);

    }

    public void Stability()
    {
        if(!stabilityStep)
        {
            Debug.Log("Step 4 Stability");
            stabilityStep = true;
        }

        float bonus = 0f;
        bonus += firstStepHandSupport();
        bonus += Math.Min(1, (agent.Head.position.y-1.6f) / 3.2f);
        bonus += CenterOfGravityRiseReward();

        agent.reward(bonus);
    }


    public float FirstStepHeadOnTop(int penality = 0, float baseValue = 1f, bool timeExpo = false)
    {
        float minHeight = -0.2f;  // Hauteur minimale au-dessus de la hauteur normale
        float maxHeight = 0.15f;   // Hauteur maximale au-dessus de la hauteur normale
        float tolerance = 3.7f;    // Tolérance au-delà de laquelle la récompense devient négative
        float headHeight = Mathf.Abs(agent.Head.position.y - ((agent.Foot_LEFT.position.y + agent.Foot_RIGHT.position.y)/2));

        float lowerBound = agent.normanHeadHeight + minHeight;
        float upperBound = agent.normanHeadHeight + maxHeight;

        // Calculer la distance hors de la plage optimale
        float distanceFromLowerBound = Mathf.Max(lowerBound - headHeight, 0);
        float distanceFromUpperBound = Mathf.Max(headHeight - upperBound, 0);
        float effectiveDistance = Mathf.Max(distanceFromLowerBound, distanceFromUpperBound);

        float reward;

        if (headHeight >= lowerBound && headHeight <= upperBound)
        {
            reward = baseValue;
        }
        else if (effectiveDistance <= tolerance)
        {
            reward = baseValue - (effectiveDistance / tolerance)*baseValue ;
        }
        else
        {
            // Au-delà de la tolérance, la récompense devient négative
            reward = -((effectiveDistance - tolerance) / tolerance);
        }

        // Appliquer une pénalité si spécifié
        if (penality != 0)
        {
            reward -= penality * (reward > 0 ? 0.1f : 2.0f);  // Pénalité réduite si positive, doublée si négative
        }
        else
        {
            reward = System.Math.Max(0, reward);
        }

        //Debug.Log("Reward: " + reward);

        return reward;
    }
    private float firstStepHandSupport()
    {
        float distanceHandHeadMax = 2.433f;
        float handHeadDistance = agent.Head.position.y - ((agent.Hand_LEFT.position.y + agent.Hand_RIGHT.position.y) / 2);
        return Math.Min(handHeadDistance / distanceHandHeadMax, 1f);
    }

    public float CenterOfGravityRiseReward()
    {
        Vector3 currentCoG = agent.CalculateCenterOfGravity(); // Assume que cela retourne le centre de gravité
        currentCoG = new Vector3(currentCoG.x, 0, currentCoG.z);
        Vector3 midpoint = (agent.Foot_LEFT.position + agent.Foot_RIGHT.position) / 2;
        midpoint = new Vector3(midpoint.x, 0, midpoint.z);
        float reward = 0f;

        if (agent.leftFootContact.LeftFootOnFloor && agent.rightFootContact.RightFootOnFloor)
        {
            if(agent.PerpendicularDistanceFromGoC >= 0)
            {
                //Debug.Log(2 -PerpendicularDistanceFromGoC);
                reward += 2 -agent.PerpendicularDistanceFromGoC;
            }
            else
            {
                //Debug.Log(PerpendicularDistanceFromGoC+2);
                reward += agent.PerpendicularDistanceFromGoC+2;
            }
            if(agent.leftFootParallel && agent.rightFootParallel)
            {
                reward += 1f;
            }
            else
            {
                reward -= 1f;
            }
        }
        else
        {
            reward-= 0.1f;
        }
        return reward;
    }

    public void Custom()
    {
        float bonus = 0f;
        float headHeight = Mathf.Abs(agent.Head.position.y - ((agent.Foot_LEFT.position.y + agent.Foot_RIGHT.position.y)/2));
        if(headHeight > 2.5f && (agent.leftFootContact.LeftFootOnFloor || agent.rightFootContact.RightFootOnFloor))
        {
            bonus += (headHeight -2.5f) / (agent.normanHeadHeight - 2.5f);
            //Debug.Log("JACKPOT !!! " + headHeight  +"  / " + (headHeight -2.5f) / (agent.normanHeadHeight - 2.5f));
        }
        if(CenterOfGravityRiseReward() > 2)
        {
            //Debug.Log("EQUILIBRE " + 1f * Math.Min(headHeight / 3.9f, 1f));
            bonus += 1f * Math.Min(headHeight / 3.9f, 1f);
        }
        if(headHeight > 3.5f && (agent.leftFootContact.LeftFootOnFloor || agent.rightFootContact.RightFootOnFloor))
        {
            bonus += 50f;
            Debug.Log("L'agent s'est levé !!!");
            agent.reward(bonus);
            agent.AddLog("Goal", 1f, Unity.MLAgents.StatAggregationMethod.Sum);
            agent.EndEpisode();
        }
        bonus += (-0.1f*agent.StepCount) / agent.MaxStep;
        agent.reward(bonus);
    }
}
