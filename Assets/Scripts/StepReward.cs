using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

public class StepReward 
{
    private bool firstStepIn = false;
    private bool secondStepIn = false;
    private bool thirdStepIn = false;
    private RobotWalk agent;
    public StepReward(RobotWalk agent)
    {
        this.agent = agent;
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
        if(agent.faceOnTheFloor == 0 || agent.faceOnTheFloor == 1)
        {
            if(agent.leftHandContact.LeftHandOnFloor && agent.rightHabdContact.RightHandOnFloor)
            {
                agent.reward(0.5f);
            }
            bonus += agent.CenterOfGravityReward();
            if(bonus > 2f)
            {
                bonus += agent.simpleHeadOnTop(penality:0, baseValue:1, timeExpo:false);
                float headHeight = Mathf.Abs(agent.Head.position.y - ((agent.Foot_LEFT.position.y + agent.Foot_RIGHT.position.y)/2));
                if(headHeight > 3.6f)
                {
                    agent.reward(100f);
                    agent.EndEpisode();
                }
                //Debug.Log("ADD head reward " + headOnTop(penality:0, baseValue:1, timeExpo:false));
            }
        }
        else
        {
            agent.EndEpisode();
            //bonus += simpleHeadOnTop(penality:0, baseValue:1, timeExpo:false);
        }
    }

    public void RiseBody()
    {

    }


    public void Walk()
    {

    }


    public float FirstStepHeadOnTop(int penality = 0, float baseValue = 1f, bool timeExpo = false)
    {
        float minHeight = -0.2f;  // Hauteur minimale au-dessus de la hauteur normale
        float maxHeight = 0.15f;   // Hauteur maximale au-dessus de la hauteur normale
        float tolerance = 3.5f;    // Tolérance au-delà de laquelle la récompense devient négative
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
        float handHeadDistance = agent.Head.position.y - ((agent.Hand_LEFT.position.y + agent.Hand_RIGHT.position.y) / 2);
        return handHeadDistance;
    }
}
