using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System;
using System.Net.Http.Headers;
using UnityEditor;
using TMPro;
using Unity.VisualScripting;
using Unity.MLAgents.SideChannels;
using BodyPart = Unity.MLAgentsExamples.BodyPart;
using Unity.MLAgentsExamples;



public class RobotWalk : Agent
{
    [Header("Target To Walk Towards")] public Transform target; //Target the agent will walk towards during training.
    [Header("Walk Speed")]
    [Range(0.1f, 5)]
    [SerializeField]
    //The walking speed to try and achieve
    private float m_TargetWalkingSpeed = 2f;

    public float MTargetWalkingSpeed // property
    {
        get { return m_TargetWalkingSpeed; }
        set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
    }
    const float m_maxWalkingSpeed = 10; //The max walking speed
    //Should the agent sample a new goal velocity each episode?
    //If true, walkSpeed will be randomly set between zero and m_maxWalkingSpeed in OnEpisodeBegin()
    //If false, the goal velocity will be walkingSpeed
    public bool randomizeWalkSpeedEachEpisode;
    private StepReward stepReward;
    string filePath = @"C:/Dev/Unity_ML-Agents/V3/ELMO/curriculum_step.txt";
    private VariableCustom vc;
    private int agentID;
    public Transform Agent; //Agent à reset à chaque tentative
    public Transform EnvToReset; //Environnement à reset à chaque tentative
    private BodyPartManager bodyPartManager;
    private BodyPartManager envPartManager;
    public Transform Target;
    public Transform Pressure_Plate;
    public ColorChanger colorChanger;
    [Header("Body Parts")] public Transform Bassin;
    public Transform Head;
    public Transform Foot_LEFT;
    public Transform Foot_RIGHT;
    public Transform Tibias_LEFT;
    public Transform Tibias_RIGHT;
    public Transform Cuisse_LEFT;
    public Transform Cuisse_RIGHT;
    //public Transform Bassin;
    public Transform Abdos;
    public Transform Torse;
    public Transform Arm_LEFT;
    public Transform Arm_RIGHT;
    public Transform Hand_LEFT;
    public Transform Hand_RIGHT;
    public Transform Coude_LEFT;
    public Transform Coude_RIGHT;
    public Transform Shoulder_LEFT;
    public Transform Shoulder_RIGHT;
    public Transform Forearm_LEFT;
    public Transform Forearm_RIGHT;


    public float TempsSession = 0f;
    private float initialReward = 1.0f;
    public float decayRate;
    private Vector3 lastPosition;
    private float distanceToTargetAtStart;
    public float normalHeadHeight = 4.13f;
    public float normalTorseHeight = 3.25f;
    private int runNumber = 0;
    public float forwardReward = 1f;
    public float lateralPenalty = -0.5f;
    public float backwardPenalty = -1f;
    private Quaternion lastRotation;
    //private bool resetInProgress = false;
    //private int lastDistanceToTarget = 0;
    /// <summary>
    /// ////////////////////
    // Implémentation de la fatigue
    ////////////////////////
    /// </summary>
    private float timeSinceLastStepLeft = 0f;
    private float timeSinceLastStepRight = 0f;
    private bool isLeftFootInFront = false;
    private bool isRightFootInFront = false;
    private bool stepInProgressRight = false;
    private bool stepInProgressLeft = false;
    private bool lastStepInProgress = false;
    private float maxStepDuration = 3f;  // Durée maximale pour un pied devant l'autre
    private float minStepDuration = 0.05f;
    private int enchainementStep = 1;
    private int maxEnchainements = 1;
    public bool leftFootParallel = false;
    public bool rightFootParallel = false;
    public float leftFootAngle = 0f;
    public float rightFootAngle = 0f;
    public Rigidbody[] bodyParts;
    public float PerpendicularDistanceFromGoC = 0f;
    public float distanceToCoGFromLeftFoot  = 0f;
    public float distanceToCoGFromRightFoot = 0f;
    public TextMeshPro textPlateforme;
    private float maxHeadHeight = 2f;
    public float currentProportionalHeadHeight = 0f;
    public float currentHeadHeight = 0f;
    public int faceOnTheFloor;
    private int curriculumStep;
    public Vector3 startPosition;
    public Quaternion startRotation;
    JointDriveController m_JdController;
    EnvironmentParameters m_ResetParams;
    Vector3 CoG;
    private Vector3 m_WorldDirToWalk = Vector3.right;
    private float targetConsecutive = 0;
    private bool canGoForward = false;
    float speedBeforeOrientate = 0f;
    private bool footLeftOnGround;
    private bool footRightOnGround;
    private int lastFootInProgress = 0; //1 = left -1 = right
    private float totalFatigue;
    

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;

    public override void Initialize()
    {
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();

        m_JdController = GetComponent<JointDriveController>();
        m_JdController.SetupBodyPart(Bassin, "Bassin");
        m_JdController.SetupBodyPart(Head, "Head");
        m_JdController.SetupBodyPart(Torse, "Torse");
        m_JdController.SetupBodyPart(Abdos, "Abdos");
        m_JdController.SetupBodyPart(Cuisse_LEFT, "Cuisse_LEFT");
        m_JdController.SetupBodyPart(Cuisse_RIGHT, "Cuisse_RIGHT");
        m_JdController.SetupBodyPart(Tibias_LEFT, "Tibias_LEFT");
        m_JdController.SetupBodyPart(Tibias_RIGHT, "Tibias_RIGHT");
        m_JdController.SetupBodyPart(Foot_LEFT, "Foot_LEFT");
        m_JdController.SetupBodyPart(Foot_RIGHT, "Foot_RIGHT");
        m_JdController.SetupBodyPart(Arm_LEFT, "Arm_LEFT");
        m_JdController.SetupBodyPart(Arm_RIGHT, "Arm_RIGHT");
        m_JdController.SetupBodyPart(Coude_LEFT, "Coude_LEFT");
        m_JdController.SetupBodyPart(Coude_RIGHT, "Coude_RIGHT");
        m_JdController.SetupBodyPart(Forearm_LEFT, "Forearm_LEFT");
        m_JdController.SetupBodyPart(Forearm_RIGHT, "Forearm_RIGHT");
        m_JdController.SetupBodyPart(Hand_LEFT, "Hand_LEFT");
        m_JdController.SetupBodyPart(Hand_RIGHT, "Hand_RIGHT");
        m_JdController.SetupBodyPart(Shoulder_LEFT, "Shoulder_LEFT");
        m_JdController.SetupBodyPart(Shoulder_RIGHT, "Shoulder_RIGHT");

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        stepReward = new StepReward(this);
        vc = new VariableCustom(this);
        // bodyPartManager = Agent.GetComponent<BodyPartManager>(); // Assurez-vous que le composant est attaché à `Agent`
        // //envPartManager = EnvToReset.GetComponent<BodyPartManager>(); // Assurez-vous que le composant est attaché à `EnvToReset`

        // if (bodyPartManager != null)
        //     bodyPartManager.Initialize(Agent); // Initialisez avec le Transform approprié

        // if (envPartManager != null)
        // Target.gameObject.SetActive(true);
        //distanceToTargetAtStart = Vector3.Distance((Foot_LEFT.position + Foot_RIGHT.position)/2, Target.position);
        // lastPosition = Torse.position;
        // lastRotation = Torse.rotation;
        // startPosition = new Vector3(0,0.5f,5);
        // startRotation = new Quaternion(0f,0f,0f,0f);
        //Pressure_Plate.localPosition = new Vector3(UnityEngine.Random.Range(-25,25),-1f,UnityEngine.Random.Range(-14,-64));
    }
    public override void OnEpisodeBegin()
    {
        Vector3 modification = new Vector3(UnityEngine.Random.Range(0, 0),0,UnityEngine.Random.Range(0, 0));
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart, modification);
        }
        //Random start rotation to help generalize
        Bassin.rotation = Quaternion.Euler(0, UnityEngine.Random.Range(0f, 0f), 0);
        UpdateOrientationObjects();

        //Set our goal walking speed
        MTargetWalkingSpeed =
            randomizeWalkSpeedEachEpisode ? UnityEngine.Random.Range(0.1f, m_maxWalkingSpeed) : MTargetWalkingSpeed;
        if(randomizeWalkSpeedEachEpisode || speedBeforeOrientate == 0f)
        {
            speedBeforeOrientate = randomizeWalkSpeedEachEpisode ? UnityEngine.Random.Range(0.1f, m_maxWalkingSpeed) : MTargetWalkingSpeed;
        }

        targetConsecutive = 0;
        enchainementStep = 1;
        totalFatigue = 0f;
        /*
        runNumber++;
        TempsSession = 0f;
        decayRate = 1f;
        //lastDistanceToTarget = 0;
        firstStep = false;
        if(startPosition == null)
        {
            (Vector3 startPosition, Quaternion startRotation) = vc.GetRandomPosition(VariableCustom.PositionType.deboutDroit);
        }
        if (bodyPartManager == null)
        {
            Debug.LogError("bodyPartManager is not assigned.");
        }
        else
        {
            bodyPartManager.ResetParts((startPosition, startRotation));
        }
        Pressure_Plate.gameObject.SetActive(true);
        */
    }

    public void reward(float value, string title="",StatAggregationMethod type=StatAggregationMethod.Sum)
    {
        AddReward(value);
        colorChanger.UpdateColorBasedOnReward(value);
        if(title != "")
        {
            AddLog(title, value, type);
        }
    }

    private bool IsInContact(Transform t)
    {
        bool ret = false;
        if (m_JdController.bodyPartsDict.TryGetValue(t, out BodyPart bodyPart)) 
        {
            if (bodyPart.groundContact.touchingGround) 
            {
                ret = true;
            } 
        } 
        else 
        {
            Debug.LogWarning(t+" n'est pas configuré correctement dans bodyPartsDict");
        }
        return ret;
    }


    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var bpDict = m_JdController.bodyPartsDict;
        var i = -1;

        var continuousActions = actionBuffers.ContinuousActions;
        //Left side
        bpDict[Cuisse_LEFT].SetJointTargetRotation(bpDict[Cuisse_LEFT],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.cuisseL1 = i-2;vc.cuisseL2 = i-1;vc.cuisseL3 = i;
        bpDict[Tibias_LEFT].SetJointTargetRotation(bpDict[Tibias_LEFT],continuousActions[++i], 0, 0, false);vc.tibiasL = i;
        bpDict[Foot_LEFT].SetJointTargetRotation(bpDict[Foot_LEFT],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.footL1=i-2;vc.footL2=i-1;vc.footL3=i;
        bpDict[Shoulder_LEFT].SetJointTargetRotation(bpDict[Shoulder_LEFT],continuousActions[++i], continuousActions[++i], continuousActions[++i]);vc.shoulderL1 = i-2;vc.shoulderL2 = i-1; vc.shoulderL3 = i;
        bpDict[Coude_LEFT].SetJointTargetRotation(bpDict[Coude_LEFT],continuousActions[++i], 0, 0);vc.coudeL = i;
        //Right side
        bpDict[Cuisse_RIGHT].SetJointTargetRotation(bpDict[Cuisse_RIGHT],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.cuisseR1 = i-2;vc.cuisseR2 = i-1;vc.cuisseR3 = i;
        bpDict[Tibias_RIGHT].SetJointTargetRotation(bpDict[Tibias_RIGHT],continuousActions[++i], 0, 0, false);vc.tibiasR = i;
        bpDict[Foot_RIGHT].SetJointTargetRotation(bpDict[Foot_RIGHT],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.footR1=i-2;vc.footR2=i-1;vc.footR3=i;
        bpDict[Shoulder_RIGHT].SetJointTargetRotation(bpDict[Shoulder_RIGHT],continuousActions[++i], continuousActions[++i], continuousActions[++i]);vc.shoulderR1 = i-2;vc.shoulderR2 = i-1; vc.shoulderR3 = i;
        bpDict[Coude_RIGHT].SetJointTargetRotation(bpDict[Coude_RIGHT],continuousActions[++i], 0, 0);vc.coudeR = i;
        //Center
        //bpDict[Bassin].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[Abdos].SetJointTargetRotation(bpDict[Abdos],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.abdo1 = i-2;vc.abdo2 = i-1;vc.abdo3 = i;
        bpDict[Torse].SetJointTargetRotation(bpDict[Torse],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.torse1 = i-2;vc.torse2 = i-1;vc.torse3 = i;
        bpDict[Head].SetJointTargetRotation(bpDict[Head],continuousActions[++i], continuousActions[++i], continuousActions[++i]);vc.head1 = i-2;vc.head2 = i-1;vc.head3= i;


        //update joint strength settings Left side
        bpDict[Cuisse_LEFT].SetJointStrength(continuousActions[++i]);
        bpDict[Tibias_LEFT].SetJointStrength(continuousActions[++i]);
        bpDict[Foot_LEFT].SetJointStrength(continuousActions[++i]);
        bpDict[Shoulder_LEFT].SetJointStrength(continuousActions[++i]);
        bpDict[Coude_LEFT].SetJointStrength(continuousActions[++i]);
        //update joint strength settings Right side
        bpDict[Cuisse_RIGHT].SetJointStrength(continuousActions[++i]);
        bpDict[Tibias_RIGHT].SetJointStrength(continuousActions[++i]);
        bpDict[Foot_RIGHT].SetJointStrength(continuousActions[++i]);
        bpDict[Shoulder_RIGHT].SetJointStrength(continuousActions[++i]);
        bpDict[Coude_RIGHT].SetJointStrength(continuousActions[++i]);
        //Center update joint strength settings
        //bpDict[Bassin].SetJointStrength(continuousActions[++i]);
        bpDict[Abdos].SetJointStrength(continuousActions[++i]);
        bpDict[Torse].SetJointStrength(continuousActions[++i]);
        bpDict[Head].SetJointStrength(continuousActions[++i]);
        //stepReward.Walk();//26
    }


    float CalculateFatigueEffect(float fatigue, float maxFatigue)
    {
        return 1.0f - (fatigue / maxFatigue);  // Réduction linéaire de la capacité basée sur la fatigue.
    }

    void FixedUpdate()
    {
        totalFatigue = 0f;
        int count = 0;
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            if (bodyPart.joint)
            {
                totalFatigue += bodyPart.muscleFatigue;
                count++;
            }
        }

        UpdateOrientationObjects();
        footLeftOnGround = IsInContact(Foot_LEFT);
        footRightOnGround = IsInContact(Foot_RIGHT);

        var cubeForward = m_OrientationCube.transform.forward;

        // b. Rotation alignment with target direction.
        //This reward will approach 1 if it faces the target direction perfectly and approach zero as it deviates
        var headForward = Head.forward;
        headForward.y = 0;
        // var lookAtTargetReward = (Vector3.Dot(cubeForward, head.forward) + 1) * .5F;
        var lookAtTargetReward = (Vector3.Dot(cubeForward, headForward) + 1) * .5F;

        //Check for NaNs
        if (float.IsNaN(lookAtTargetReward))
        {
            throw new ArgumentException(
                "NaN in lookAtTargetReward.\n" +
                $" cubeForward: {cubeForward}\n" +
                $" head.forward: {Head.forward}"
            );
        }

        if(lookAtTargetReward <= 0.8)
        {
            MTargetWalkingSpeed = 0f;
        }
        else
        {
            MTargetWalkingSpeed = speedBeforeOrientate;//*((lookAtTargetReward-0.9f)*10);
        }
        // Set reward for this step according to mixture of the following elements.
        // a. Match target speed
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        var matchSpeedReward = GetMatchingVelocityReward(cubeForward * MTargetWalkingSpeed, GetAvgVelocity());

        //Check for NaNs
        if (float.IsNaN(matchSpeedReward))
        {
            throw new ArgumentException(
                "NaN in moveTowardsTargetReward.\n" +
                $" cubeForward: {cubeForward}\n" +
                $" hips.velocity: {m_JdController.bodyPartsDict[Bassin].rb.velocity}\n" +
                $" maximumWalkingSpeed: {m_maxWalkingSpeed}"
            );
        }


        //AddReward(matchSpeedReward * lookAtTargetReward);
        //Debug.Log("lookAtTargetReward:"+lookAtTargetReward+"  /  matchSpeedReward:"+ (String.Format("{0:F6}", matchSpeedReward)) );
        //reward(matchSpeedReward * lookAtTargetReward, "Look at Target", StatAggregationMethod.Average);

        //reward(lookAtTargetReward * matchSpeedReward , "Look at Target/Matche Speed", StatAggregationMethod.Average);
        canGoForward = lookAtTargetReward > 0.9;
        //reward(CalculateCoGReward()*currentProportionalHeadHeight, "Centre de gravité / Head Height", StatAggregationMethod.Average);
        //reward(totalFatigue/1000);

        //reward(CalculateCoGReward()*matchSpeedReward);
        reward(currentProportionalHeadHeight);

        UpdateFootPosition();
        //reward(ApplyStepReward()*lookAtTargetReward, "Foot steps",StatAggregationMethod.Sum);
        //reward(matchSpeedReward, "Matche Speed", StatAggregationMethod.Average);
    }
    float CalculateCoGReward() 
    {
        // Vector3 midpoint = (Foot_LEFT.position + Foot_RIGHT.position) / 2;
        // midpoint.y = 0f;
        // Vector3 cog0 = CoG;
        // cog0.y = 0f;
        // float distanceToMidpoint = Vector3.Distance(cog0, midpoint);
        // //Debug.Log(distanceToMidpoint);
        // float threshold = 0.1f;  // Distance tolérable du CoG du midpoint

        // if (distanceToMidpoint < threshold) {
        //     return 1.0f;  // Récompense maximale si le CoG est bien centré
        // } else {
        //     // Pénaliser en fonction de la distance du CoG au midpoint
        //     return Mathf.Clamp(1 - distanceToMidpoint, 0, 1);
        // }
        float floorLevel = (Foot_LEFT.position.y + Foot_RIGHT.position.y) / 2;
        Vector3 floorCoG = CalculateCenterOfGravity();
        floorCoG = new Vector3(floorCoG.x, floorLevel, floorCoG.z);
        // Points en 2D pour le calcul
        Vector2 pointA = new Vector2(Foot_LEFT.position.x, Foot_LEFT.position.z-0.06f+MTargetWalkingSpeed/10);
        Vector2 pointB = new Vector2(Foot_RIGHT.position.x, Foot_RIGHT.position.z-0.06f+MTargetWalkingSpeed/10);
        Vector2 pointP = new Vector2(floorCoG.x, floorCoG.z);
        // Vecteur direction de la ligne
        Vector2 lineDirection = (pointB - pointA).normalized;

        // Calculer la composante perpendiculaire
        Vector2 perpendicular = Vector2.Perpendicular(lineDirection).normalized;

        // Trouver le point sur la ligne le plus proche de CoG
        float t = Vector2.Dot(pointP - pointA, lineDirection);
        Vector2 closestPoint = pointA + t * lineDirection;

        // Dessiner la ligne entre les deux pieds
        //Gizmos.color = Color.blue;
        //Gizmos.DrawLine(new Vector3(pointA.x, 0, pointA.y), new Vector3(pointB.x, 0, pointB.y));

        // Dessiner la perpendiculaire du CoG à la ligne
        //Gizmos.color = Color.red;
        //Gizmos.DrawLine(new Vector3(closestPoint.x, 0, closestPoint.y), new Vector3(pointP.x, 0, pointP.y));

        // Optionnel: Afficher la distance comme label
        PerpendicularDistanceFromGoC = Vector2.Dot(pointP - pointA, perpendicular);
        // GUIStyle style = new GUIStyle();
        // style.normal.textColor = Color.white;
        Vector3 midPoint = (new Vector3(closestPoint.x, 0, closestPoint.y) + new Vector3(pointP.x, 0, pointP.y)) / 2;
        //Handles.Label(midPoint, $"Distance: {PerpendicularDistanceFromGoC:F2}", style);
        Vector2 centreGauche = new Vector2(Foot_LEFT.position.x+0.3f, Foot_LEFT.position.z);
        Vector2 centreDroite = new Vector2(Foot_LEFT.position.x-0.3f, Foot_LEFT.position.z);
        distanceToCoGFromLeftFoot = Vector2.Distance(centreGauche, closestPoint);
        distanceToCoGFromRightFoot = Vector2.Distance(centreDroite, closestPoint);
        if(PerpendicularDistanceFromGoC <0)
        {
            return 1+PerpendicularDistanceFromGoC;
        }else
        {
            return 1-PerpendicularDistanceFromGoC;
        }
        
    }

    //normalized value of the difference in avg speed vs goal walking speed.
    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        //distance between our actual velocity and goal velocity
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, MTargetWalkingSpeed);

        //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / MTargetWalkingSpeed, 2), 2);
    }
    float CalculateActionIntensity(params float[] actions)
    {
        float totalIntensity = 0f;
        foreach (float action in actions)
        {
            totalIntensity += Mathf.Abs(action);  // Somme des valeurs absolues des actions
        }
        return totalIntensity;
    }

    /// <summary>
    /// Agent touched the target
    /// </summary>
    public void TouchedTarget()
    {
        // Debug.Log("Cube Touché");
        // reward(1f, "Cube touched",StatAggregationMethod.Sum);
    }



    // Convertit une valeur d'entrée de -1 à 1 en une valeur de rotation en degrés, basée sur la limite maximale spécifiée.
    private float ConvertInputToRotation(float inputValue, float limit)
    {
        return inputValue * limit;
    }

    //Returns the average velocity of all of the body parts
    //Using the velocity of the hips only has shown to result in more erratic movement from the limbs, so...
    //...using the average helps prevent this erratic movement
    Vector3 GetAvgVelocity()
    {
        Vector3 velSum = Vector3.zero;

        //ALL RBS
        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            velSum += item.rb.velocity;
        }

        var avgVel = velSum / numOfRb;
        return avgVel;
    }


    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        if (Input.GetKey(KeyCode.UpArrow))
        {
            continuousActionsOut[vc.shoulderL1] = 1.0f;
            continuousActionsOut[vc.shoulderR1] = 1.0f;
            continuousActionsOut[vc.torse1] = 1.0f;
            continuousActionsOut[vc.abdo1] = 1.0f;
            continuousActionsOut[vc.head1] = 1.0f;
            continuousActionsOut[vc.cuisseL1] = 1.0f;
            continuousActionsOut[vc.cuisseR1] = 1.0f;

            continuousActionsOut[vc.coudeL] = 1.0f;
            continuousActionsOut[vc.coudeR] = 1.0f;
            continuousActionsOut[vc.footL1] = 1.0f;
            continuousActionsOut[vc.footR1] = 1.0f;
            continuousActionsOut[vc.tibiasL] = 1.0f;
            continuousActionsOut[vc.tibiasR] = 1.0f;
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            continuousActionsOut[vc.shoulderL1] = -1.0f;
            continuousActionsOut[vc.shoulderR1] = -1.0f;
            continuousActionsOut[vc.torse1] = -1.0f;
            continuousActionsOut[vc.abdo1] = -1.0f;
            continuousActionsOut[vc.head1] = -1.0f;
            continuousActionsOut[vc.cuisseL1] = -1.0f;
            continuousActionsOut[vc.cuisseR1] = -1.0f;

            continuousActionsOut[vc.coudeL] = -1.0f;
            continuousActionsOut[vc.coudeR] = -1.0f;
            continuousActionsOut[vc.footL1] = -1.0f;
            continuousActionsOut[vc.footR1] = -1.0f;
            continuousActionsOut[vc.tibiasL] = -1.0f;
            continuousActionsOut[vc.tibiasR] = -1.0f;
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
            // continuousActionsOut[vc.shoulderL2] = 1.0f;
            // continuousActionsOut[vc.shoulderR2] = 1.0f;
            // continuousActionsOut[vc.tibiasL] = 1.0f;
            // continuousActionsOut[vc.tibiasR] = 1.0f;
            
            // continuousActionsOut[vc.torse2] = 1.0f;
            // continuousActionsOut[vc.abdo2] = 1.0f;
            // continuousActionsOut[vc.head2] = 1.0f;
            continuousActionsOut[vc.footL1] = 1.0f;
            continuousActionsOut[vc.footR1] = 1.0f;
            continuousActionsOut[vc.footL2] = 1.0f;
            continuousActionsOut[vc.footR2] = 1.0f;
            continuousActionsOut[vc.footL3] = 1.0f;
            continuousActionsOut[vc.footR3] = 1.0f;
        }
        else if (Input.GetKey(KeyCode.LeftArrow))
        {
            // continuousActionsOut[vc.shoulderL2] = -1.0f;
            // continuousActionsOut[vc.shoulderR2] = -1.0f;
            // continuousActionsOut[vc.tibiasL] = -1.0f;
            // continuousActionsOut[vc.tibiasR] = -1.0f;
            
            // continuousActionsOut[vc.torse2] = -1.0f;
            // continuousActionsOut[vc.abdo2] = -1.0f;
            // continuousActionsOut[vc.head2] = -1.0f;
            continuousActionsOut[vc.footL1] = -1.0f;
            continuousActionsOut[vc.footR1] = -1.0f;
            continuousActionsOut[vc.footL2] = -1.0f;
            continuousActionsOut[vc.footR2] = -1.0f;
            continuousActionsOut[vc.footL3] = -1.0f;
            continuousActionsOut[vc.footR3] = -1.0f;
        }


        if (Input.GetKey(KeyCode.C))
        {
            // continuousActionsOut[vc.coudeL] = 1.0f;
            // continuousActionsOut[vc.coudeR] = 1.0f;
            
            // continuousActionsOut[vc.torse3] = 1.0f;
            // continuousActionsOut[vc.abdo3] = 1.0f;
            // continuousActionsOut[vc.head3] = 1.0f;
            continuousActionsOut[vc.cuisseL3] = 1.0f;
            continuousActionsOut[vc.cuisseR3] = 1.0f;
        }
        else if (Input.GetKey(KeyCode.V))
        {
            // continuousActionsOut[vc.coudeL] = -1.0f;
            // continuousActionsOut[vc.coudeR] = -1.0f;
            // continuousActionsOut[vc.torse3] = -1.0f;
            // continuousActionsOut[vc.abdo3] = -1.0f;
            // continuousActionsOut[vc.head3] = -1.0f;
            
            continuousActionsOut[vc.cuisseL3] = -1.0f;
            continuousActionsOut[vc.cuisseR3] = -1.0f;
        }

        if (Input.GetKey(KeyCode.Y))
        {
            continuousActionsOut[vc.shoulderL3] = 1.0f;
            continuousActionsOut[vc.shoulderR3] = 1.0f;
        }
        if (Input.GetKey(KeyCode.X))
        {
            continuousActionsOut[vc.shoulderL3] = -1.0f;
            continuousActionsOut[vc.shoulderR3] = -1.0f;
        }

        if (Input.GetKey(KeyCode.R))
        {
            EndEpisode();
        }
    }

    private float MeasureGroundDistance(Transform footTransform, float maxDistance)
    {
        RaycastHit hit;
        int groundLayerMask = 1 << LayerMask.NameToLayer("Devetik_Floor");  // Assurez-vous que ce layer est correctement configuré

        // Envoyer un rayon depuis chaque pied vers le bas
        if (Physics.Raycast(footTransform.position, Vector3.down, out hit, maxDistance, groundLayerMask))
        {
            float groundHeight = hit.distance;

            return groundHeight/maxDistance;  // Ajouter la distance au sol pour ce pied
        }
        else
        {
            return maxDistance/maxDistance;  // Aucun sol détecté dans la limite de distance
        }
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        CoG = CalculateCenterOfGravity();
        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }

        var cubeForward = m_OrientationCube.transform.forward;
        var velGoal = cubeForward * MTargetWalkingSpeed;
        var avgVel = GetAvgVelocity();
        sensor.AddObservation(canGoForward ? 1f:0f);

        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));
        sensor.AddObservation(Quaternion.FromToRotation(Bassin.forward, cubeForward));
        sensor.AddObservation(Quaternion.FromToRotation(Head.forward, cubeForward));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(target.transform.position));
        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));

        sensor.AddObservation(MeasureGroundDistance(transform, 4f));
        sensor.AddObservation(MeasureGroundDistance(Torse, normalTorseHeight));
        sensor.AddObservation(MeasureGroundDistance(Foot_LEFT, 2f));
        sensor.AddObservation(MeasureGroundDistance(Foot_RIGHT, 2f));
        sensor.AddObservation(currentProportionalHeadHeight);

        // sensor.AddObservation(CalculateCenterOfGravity());
        sensor.AddObservation(CalculateCoGReward() );
        Vector3 CoGPlat = CoG;
        CoGPlat.y = 0f;
        sensor.AddObservation(Foot_LEFT.position - CoGPlat);
        sensor.AddObservation(Foot_RIGHT.position - CoGPlat);
        sensor.AddObservation(CoG);
    }
        /// <summary>
    /// Add relevant information on each body part to observations.
    /// </summary>
    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        Vector3 center = CalculateCenterOfGravity();
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground
        //sensor.AddObservation(bp.rb.position - CoG);
        //Debug.Log(CoG);
        if(bp.joint)
        {
            sensor.AddObservation(bp.muscleFatigue / 100f);
        }

        //Get velocities in the context of our orientation cube's space
        //Note: You can get these velocities in world space as well but it may not train as well.
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.velocity));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.angularVelocity));

        //Get position relative to hips in the context of our orientation cube's space
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.position - Bassin.position));

        if (bp.rb.transform != Bassin && bp.rb.transform != Hand_LEFT && bp.rb.transform != Hand_RIGHT)
        {
            sensor.AddObservation(bp.rb.transform.localRotation);
            sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
        }
    }

    IEnumerator endEpisodeIfHeadNotRise(float time)
    {
        yield return new WaitForSeconds(time);
        //resetInProgress = false;
        if(Head.localPosition.y < (normalHeadHeight * 0.6f))
        {
            EndEpisode();
        }
    }
    private void OnTriggerEnter(Collider other)
    {
        if(other.gameObject.tag == "Walls")
        {
            //Debug.Log("Wall");
            reward(-1000f);
            EndEpisode();
        }
        // if(other.gameObject.tag == "Target")
        // {
        //     Debug.Log("Cube Touché");
        //     reward(1f, "Cube touched",StatAggregationMethod.Sum);
        // }
    }

    public float isWalkingForward()
    {
        // Calculer la direction et le déplacement relatif
        Vector3 displacement = Torse.position - lastPosition;
        float forwardDistance = Vector3.Dot(displacement, transform.up);
        //Debug.Log(Vector3.Dot(displacement, transform.forward) +" - "+ Vector3.Dot(displacement, transform.up)  +" - "+ Vector3.Dot(displacement, transform.right));
        float lateralDistance = Vector3.Dot(displacement, transform.right);
        float ret = 0f;
        string log = "";
        // Récompenser ou pénaliser en fonction de la direction du déplacement
        if (forwardDistance > 0.01f)
        {
            ret -= 0.1f;
            log = "Arrière";
        }
        else if (forwardDistance < -0.01f)
        {
            //ret += 5f;
            ret += System.Math.Abs(forwardDistance  + 1f);
            log = "Avant" + System.Math.Abs(forwardDistance + 1f);
        }
        else
        {
            log = "Static";
            ret -= 0.01f;
        }

        if (Mathf.Abs(lateralDistance) > 0.05f)
        {
            //reward(Mathf.Abs(lateralDistance) * lateralPenalty);
            ret -= System.Math.Abs(lateralDistance * 10f + 1f);
            log += " et latérale " + lateralDistance;
        }

        // Mise à jour des positions et rotations pour la prochaine évaluation
        lastPosition = Torse.position;
        lastRotation = Torse.rotation;
        //Debug.Log(log);
        return ret;
    }

    // public float headOnTop(int penality = 0, float baseValue = 1f, bool timeExpo = false, float tolerance = 1f)
    // {
    //     float minHeight = -0.2f;  // Hauteur minimale au-dessus de la hauteur normale
    //     float maxHeight = 0.15f;   // Hauteur maximale au-dessus de la hauteur normale

    //     float headHeight = Mathf.Abs(Head.position.y - ((Foot_LEFT.position.y + Foot_RIGHT.position.y)/2));

    //     float lowerBound = normalHeadHeight + minHeight;
    //     float upperBound = normalHeadHeight + maxHeight;

    //     // Calculer la distance hors de la plage optimale
    //     float distanceFromLowerBound = Mathf.Max(lowerBound - headHeight, 0);
    //     float distanceFromUpperBound = Mathf.Max(headHeight - upperBound, 0);
    //     float effectiveDistance = Mathf.Max(distanceFromLowerBound, distanceFromUpperBound);

    //     float reward;

    //     if (headHeight >= lowerBound && headHeight <= upperBound)
    //     {
    //         // La tête est dans la plage idéale
    //         if(timeExpo)
    //         {
    //             reward = baseValue * ((TempsSession*5) + 1);
    //         }
    //         else
    //         {
    //             reward = baseValue;
    //         }
    //         if(leftFootContact.LeftFootOnFloor )
    //         {
    //             if(Vector3.Angle(Tibias_LEFT.up, Vector3.up) < 25)
    //             {
    //                 if(Vector3.Angle(Cuisse_LEFT.up, Vector3.up) < 60)
    //                 {
    //                     if(Vector3.Angle(Bassin.up, Vector3.up) < 25)
    //                     {
    //                         reward += reward * 2f;
    //                     }
    //                     else
    //                     {
    //                         reward += reward * 1.5f;
    //                     }
    //                 }
    //                 else
    //                 {
    //                     reward += reward ;
    //                 }
    //             }
    //             else
    //             {
    //                 reward += reward;
    //             }
    //         }
    //         else if(rightFootContact.RightFootOnFloor )
    //         {
    //             if(Vector3.Angle(Tibias_RIGHT.up, Vector3.up) < 25)
    //             {
    //                 if(Vector3.Angle(Cuisse_RIGHT.up, Vector3.up) < 60)
    //                 {
    //                     if(Vector3.Angle(Bassin.up, Vector3.up) < 25)
    //                     {
    //                         reward += reward * 2f;
    //                     }
    //                     else
    //                     {
    //                         reward += reward * 1.5f;
    //                     }
    //                 }
    //                 else
    //                 {
    //                     reward += reward ;
    //                 }
    //             }
    //             else
    //             {
    //                 reward += reward;
    //             }
    //         }
    //     }
    //     else if (effectiveDistance <= tolerance)
    //     {
    //         // La récompense diminue linéairement de 1 à 0 à mesure que la distance augmente jusqu'à la tolérance
    //         if(timeExpo)
    //         {
    //             reward = baseValue* ((TempsSession*2) + 1) - (effectiveDistance / tolerance)*baseValue ;
    //         }
    //         else
    //         {
    //             reward = baseValue - (effectiveDistance / tolerance)*baseValue ;
    //         }

    //         //Debug.Log("baseValue:"+baseValue+ " -"+(effectiveDistance / tolerance)+" * TempsSession:"+TempsSession+"*2+1");
    //     }
    //     else
    //     {
    //         // Au-delà de la tolérance, la récompense devient négative
    //         reward = -((effectiveDistance - tolerance) / tolerance);
    //     }

    //     // Appliquer une pénalité si spécifié
    //     if (penality != 0)
    //     {
    //         reward -= penality * (reward > 0 ? 0.1f : 2.0f);  // Pénalité réduite si positive, doublée si négative
    //     }
    //     else
    //     {
    //         reward = Math.Max(0, reward);
    //     }

    //     //Debug.Log("Reward: " + reward);

    //     return reward;
    // }
    public float simpleHeadOnTop(int penality = 0, float baseValue = 1f, bool timeExpo = false)
    {
        float minHeight = -0.2f;  // Hauteur minimale au-dessus de la hauteur normale
        float maxHeight = 0.15f;   // Hauteur maximale au-dessus de la hauteur normale
        float tolerance = 3f;    // Tolérance au-delà de laquelle la récompense devient négative
        float headHeight = Mathf.Abs(Head.position.y - ((Foot_LEFT.position.y + Foot_RIGHT.position.y)/2));

        float lowerBound = normalHeadHeight + minHeight;
        float upperBound = normalHeadHeight + maxHeight;

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
            reward = Math.Max(0, reward);
        }

        //Debug.Log("Reward: " + reward);

        return reward;
    }

    private float verticalStability(float angle, float exposant)
    {
        float ret = 0f;
        float verticalAlignment = Vector3.Dot(Torse.up, Vector3.up);
        float cos15 = Mathf.Cos(angle * Mathf.Deg2Rad); // Convertir 15 degrés en radians puis en cosinus

        if (verticalAlignment >= cos15) {
            // Si verticalAlignment est dans le range +/- 15°
            ret = 1f;
        } else {
            // Hors de ce range, calculez une pénalité basée sur l'écart au cosinus de 15 degrés
            float penalty = (cos15 - verticalAlignment) / (1f - cos15); // Normalisez la pénalité par rapport à la plage max possible hors de 15°
            ret -= penalty;
        }
        return ret * exposant;
    }
    public float DistanceToTarget()
    {
        //float reward = 0f;
        float distanceToTargetRestante = Vector3.Distance(Torse.position, Target.position);
        float distanceParcourue = distanceToTargetAtStart - distanceToTargetRestante ;
        float proportionTravel = distanceParcourue / distanceToTargetAtStart ;

        return proportionTravel;
    }

    void UpdateFootPosition()
    {
        //Contrôle si le pied est parallel au sol
        Vector3 LeftfootUp = Foot_LEFT.up;
        Vector3 RightfootUp = Foot_RIGHT.up;
        Vector3 worldUp = Vector3.up;
        leftFootAngle = Vector3.Angle(LeftfootUp, worldUp);
        rightFootAngle = Vector3.Angle(RightfootUp, worldUp);

        leftFootParallel = leftFootAngle < 5f;
        rightFootParallel = rightFootAngle < 5f;

        // Positions des pieds en coordonnées locales par rapport au Bassin
        Vector3 leftFootPosition = Bassin.InverseTransformPoint(Foot_LEFT.position);
        Vector3 rightFootPosition = Bassin.InverseTransformPoint(Foot_RIGHT.position);

        // Utilisation de la composante z pour déterminer quel pied est en avant
        isLeftFootInFront = leftFootPosition.z - rightFootPosition.z > 0.005;
        isRightFootInFront = rightFootPosition.z - leftFootPosition.z > 0.005;
        //Debug.Log(leftFootPosition.z - rightFootPosition.z);
        //Debug.Log(isLeftFootInFront ? "Gauche en avant": isRightFootInFront ? "Droit en avant": "Position NULL");

        if(isLeftFootInFront)
        {
            stepInProgressLeft = true;
            stepInProgressRight = false;
        }
        else if(isRightFootInFront)
        {
            stepInProgressLeft = false;
            stepInProgressRight = true;  
        }
        else
        {
            stepInProgressLeft = false;
            stepInProgressRight = false;
        }

        if (isLeftFootInFront && stepInProgressLeft == true && lastFootInProgress != 1)
        {
            stepInProgressLeft = true;
            stepInProgressRight = false;
            timeSinceLastStepLeft += Time.deltaTime;
            timeSinceLastStepRight = 0;
        }
        else if (isRightFootInFront && stepInProgressRight == true && lastFootInProgress != -1)
        {
            stepInProgressLeft = false;
            stepInProgressRight = true;
            timeSinceLastStepRight += Time.deltaTime;
            timeSinceLastStepLeft = 0;  // Réinitialiser le timer si les positions changent
        }
        else
        {
            timeSinceLastStepLeft = 0;
            timeSinceLastStepRight = 0;
        }
    }

    public float ApplyStepReward()
    {
        float bonus = 0f;
        if(enchainementStep > maxEnchainements)
        {
            Debug.Log("ID:" + agentID + " - " + enchainementStep);
            maxEnchainements = enchainementStep;
        }
        // Si le pied est resté en avant moins de 2 secondes, récompenser l'agent
        if (timeSinceLastStepLeft > minStepDuration && timeSinceLastStepLeft < maxStepDuration && stepInProgressLeft == true && footLeftOnGround && leftFootParallel && (Vector3.Angle(Tibias_LEFT.up, Vector3.up) < 25) && (Vector3.Angle(Bassin.up, Vector3.up) < 25) && (Vector3.Angle(Cuisse_LEFT.up, Vector3.up) < 25))
        {
            lastFootInProgress = 1;
            bonus += 1f;  // Récompense pour maintenir un pied devant l'autre
            enchainementStep++;
            stepInProgressLeft = false;
        }
        // Pénaliser l'agent si le pied reste en avant plus de 2 secondes
        else if (timeSinceLastStepLeft >= maxStepDuration)
        {
            lastFootInProgress = 1;
            timeSinceLastStepLeft = 0;  // Réinitialiser pour éviter des pénalités continues
            stepInProgressLeft = false;
        }
        else if (timeSinceLastStepRight > minStepDuration && timeSinceLastStepRight < maxStepDuration && stepInProgressRight == true && footRightOnGround && rightFootParallel && (Vector3.Angle(Tibias_RIGHT.up, Vector3.up) < 25) && (Vector3.Angle(Bassin.up, Vector3.up) < 25) && (Vector3.Angle(Cuisse_LEFT.up, Vector3.up) < 25))
        {
            lastFootInProgress = -1;
            bonus += 1f;  // Récompense pour maintenir un pied devant l'autre
            enchainementStep++;
            stepInProgressRight = false;
        }
        // Pénaliser l'agent si le pied reste en avant plus de 2 secondes
        else if (timeSinceLastStepRight >= maxStepDuration)
        {
            lastFootInProgress = -1;
            timeSinceLastStepRight = 0;  // Réinitialiser pour éviter des pénalités continues
            stepInProgressRight = false;
        }
        return bonus;
    }
    // private Vector3 CalculateCenterOfGravity()
    // {
    //     Vector3 totalCenterOfMass = Vector3.zero;
    //     float totalMass = 0f;


    //     foreach (BodyPart bodyPart in m_JdController.bodyPartsList)
    //     {
    //         Rigidbody rb = bodyPart.rb.GetComponent<Rigidbody>();
    //         if (rb != null)
    //         {
    //             totalCenterOfMass += rb.worldCenterOfMass * rb.mass;
    //             totalMass += rb.mass;
    //         }
    //     }

    //     if (totalMass > 0)
    //     {
    //         return totalCenterOfMass / totalMass;
    //     }
    //     return Vector3.zero; // Retourne l'origine si la masse totale est nulle pour éviter une division par zéro
    // }

    public Vector3 CalculateCenterOfGravity()
    {
        Vector3 totalCenterOfMass = Vector3.zero;
        float totalMass = 0f;

        foreach (Rigidbody rb in bodyParts)
        {
            totalCenterOfMass += rb.worldCenterOfMass * rb.mass;
            totalMass += rb.mass;
        }

        if (totalMass > 0)
        {
            return totalCenterOfMass / totalMass;
        }
        return Vector3.zero; // Retourne l'origine si la masse totale est nulle pour éviter une division par zéro
    }

    private float CalculateHorizontalDistanceToCoG()
    {
        Vector3 centerOfGravity = CalculateCenterOfGravity();
        Vector3 midpoint = (Foot_LEFT.position + Foot_RIGHT.position) / 2;
        midpoint.y = 0; // Ignorer la composante verticale

        Vector3 centerOfGravityHorizontal = new Vector3(centerOfGravity.x, 0, centerOfGravity.z);
        return Vector3.Distance(midpoint, centerOfGravityHorizontal);
    }

    void Update()
    {
        RaycastHit hit;
        int groundLayerMask = 1 << LayerMask.NameToLayer("Devetik_Floor");  // Assurez-vous que ce layer est correctement configuré

        // Envoyer un rayon depuis chaque pied vers le bas
        if (Physics.Raycast(Head.position, Vector3.down, out hit, 6f, groundLayerMask))
        {
            currentProportionalHeadHeight = System.Math.Min((hit.distance-0.55f) / (normalHeadHeight-0.55f), 1);
            //Debug.Log(currentProportionalHeadHeight);
            currentHeadHeight = hit.distance;
            //Debug.DrawRay(Head.position, Vector3.down , Color.red, 2f);
        }


        // Vector3 min = Vector3.Min(Foot_LEFT.position, Foot_RIGHT.position);
        // Vector3 max = Vector3.Max(Foot_LEFT.position, Foot_RIGHT.position);

        // // Créer le rectangle en utilisant les min et max
        // Rect footRect = new Rect(min.x, min.z, max.x - min.x, max.z - min.z);

        // // Vérifier si le centre de gravité est dans le rectangle
        // if (footRect.Contains(new Vector2(CoG.x, CoG.z))) {
        //     Debug.Log("Le centre de gravité est à l'intérieur du rectangle");
        // } else {
        //     Debug.Log("Le centre de gravité est à l'extérieur du rectangle");
        // }


        // Vector3 min = Vector3.Min(Foot_LEFT.position, Foot_RIGHT.position);
        // Vector3 max = Vector3.Max(Foot_LEFT.position, Foot_RIGHT.position);
        // Vector3 center = (min + max) / 2;
        // float width = Mathf.Abs(max.x - min.x);
        // float length = Mathf.Abs(max.z - min.z);

        // // Définir la couleur du Gizmo en fonction de la position du centre de gravité
        // Rect footRect = new Rect(min.x, min.z, width, length);
        // if (footRect.Contains(new Vector2(CoG.x, CoG.z))) {
        //     Gizmos.color = Color.green;
        // } else {
        //     Gizmos.color = Color.red;
        // }

        // // Dessiner le rectangle
        // Gizmos.DrawWireCube(new Vector3(center.x, 0.01f, center.z), new Vector3(width, 0.02f, length));
    }

    public float CenterOfGravityReward()
    {
        Vector3 currentCoG = CalculateCenterOfGravity(); // Assume que cela retourne le centre de gravité
        currentCoG = new Vector3(currentCoG.x, 0, currentCoG.z);
        Vector3 midpoint = (Foot_LEFT.position + Foot_RIGHT.position) / 2;
        midpoint = new Vector3(midpoint.x, 0, midpoint.z);
        float reward = 0f;

        if (true)//(leftFootContact.LeftFootOnFloor && rightFootContact.RightFootOnFloor)
        {
            // // Les deux pieds sont au sol
            // float distanceToMidpoint = Vector3.Distance(currentCoG, midpoint);
            // if(Math.Abs(midpoint.z - currentCoG.z) < 0.2 )
            // {
            //     reward += 1f;
            // }
            // else
            // {
            //     reward -= 1f + distanceToMidpoint;
            // }
            // if(Math.Abs(midpoint.x - currentCoG.x) < 0.2 )
            // {
            //     reward += 1f;
            // }
            // else
            // {
            //     reward -= 1f + distanceToMidpoint;
            // }
            //Si penché en avant
            if(PerpendicularDistanceFromGoC >= 0)
            {
                //Debug.Log(2 -PerpendicularDistanceFromGoC);
                reward += 1 -PerpendicularDistanceFromGoC;
            }
            else
            {
                //Debug.Log(PerpendicularDistanceFromGoC+2);
                reward += PerpendicularDistanceFromGoC+1;
            }
            if(leftFootParallel && rightFootParallel)
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
        // else if ( leftFootContact.LeftFootOnFloor || rightFootContact.RightFootOnFloor)
        // {
        //     // Un pied est levé
        //     Transform supportingFoot = leftFootContact.LeftFootOnFloor ? Foot_LEFT : Foot_RIGHT;
        //     Vector3 supportingFootZero = new Vector3(supportingFoot.position.x, 0f, supportingFoot.position.z+0.2f);
        //     float distanceToSupportingFoot = leftFootContact.LeftFootOnFloor ? distanceToCoGFromLeftFoot : distanceToCoGFromRightFoot;
        //     float distanceZ = Math.Abs(currentCoG.z - supportingFootZero.z);
        //     float distanceX = Math.Abs(currentCoG.x - supportingFootZero.x);
        //     float supportingFootAngle = leftFootContact.LeftFootOnFloor ? leftFootAngle : rightFootAngle;
        //     if(supportingFootAngle < 45)
        //     {
        //         float CustomPerpendicularDistanceFromGoC = PerpendicularDistanceFromGoC - 0.2f;
        //         if(PerpendicularDistanceFromGoC >= 0)
        //         {
        //             //Debug.Log(2 -CustomPerpendicularDistanceFromGoC + " distanceToCoGFromLeftFoot:" + (1-distanceToSupportingFoot) + " Angle:" + supportingFootAngle);
        //             reward += 2 -CustomPerpendicularDistanceFromGoC;
        //         }
        //         else
        //         {
        //             //Debug.Log(CustomPerpendicularDistanceFromGoC+2+ " distanceToCoGFromLeftFoot:" + (1-distanceToSupportingFoot) + " Angle:" + supportingFootAngle);
        //             reward += CustomPerpendicularDistanceFromGoC+2;
        //         }
        //     }
        //     else
        //     {
        //         reward -= 0.1f;
        //     }
            /*
                //Calcule la distance entre un point et la parallel entre les deux pieds sur le plan horizontale
                Vector2 pointA = new Vector2(Foot_LEFT.position.x, Foot_LEFT.position.z);
                Vector2 pointB = new Vector2(Foot_RIGHT.position.x, Foot_RIGHT.position.z);
                Vector2 pointP = new Vector2(currentCoG.x, currentCoG.z);

                // Vecteur direction de la ligne
                Vector2 lineDirection = (pointB - pointA).normalized;

                // Vecteur du début de la ligne au point
                Vector2 pointToLineStart = pointP - pointA;

                // Calculer la composante perpendiculaire
                Vector2 perpendicular = Vector2.Perpendicular(lineDirection);
                float distanceFromPointToLine = Mathf.Abs(Vector2.Dot(pointToLineStart, perpendicular.normalized));
            */

        //}
        // else
        // {
        //     reward = -1f;
        // }

        // Appliquez la récompense ou la pénalité
        //return reward;
    }
    void OnDrawGizmos()
    {
        // if (Application.isPlaying) // Assurez-vous que cela est dessiné uniquement si l'application est en cours d'exécution
        // {
            Gizmos.color = Color.red;
            float floorLevel = (Foot_LEFT.position.y + Foot_RIGHT.position.y) / 2;
            Vector3 floorCoG = CalculateCenterOfGravity();
            floorCoG = new Vector3(floorCoG.x, floorLevel, floorCoG.z);
            Gizmos.DrawSphere(floorCoG, 0.1f); // Dessine une petite sphère au centre de gravité

            // Gizmos.color = Color.red;
            // Gizmos.DrawSphere(floorCoG, 0.1f);
            // Gizmos.color = Color.blue;
            // Gizmos.DrawSphere(midpoint, 0.1f);
            // if (supportingFoot != null) {
            //     Gizmos.color = Color.green;
            //     Gizmos.DrawSphere(supportingFoot.position, 0.1f);
            // }
            if (Foot_LEFT == null || Foot_RIGHT == null || floorCoG == null)
            return;

            // Points en 2D pour le calcul
            Vector2 pointA = new Vector2(Foot_LEFT.position.x, Foot_LEFT.position.z-0.06f+MTargetWalkingSpeed/10);
            Vector2 pointB = new Vector2(Foot_RIGHT.position.x, Foot_RIGHT.position.z-0.06f+MTargetWalkingSpeed/10);
            Vector2 pointP = new Vector2(floorCoG.x, floorCoG.z);

            // Vecteur direction de la ligne
            Vector2 lineDirection = (pointB - pointA).normalized;

            // Calculer la composante perpendiculaire
            Vector2 perpendicular = Vector2.Perpendicular(lineDirection).normalized;

            // Trouver le point sur la ligne le plus proche de CoG
            float t = Vector2.Dot(pointP - pointA, lineDirection);
            Vector2 closestPoint = pointA + t * lineDirection;

            // Dessiner la ligne entre les deux pieds
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(new Vector3(pointA.x, 0, pointA.y), new Vector3(pointB.x, 0, pointB.y));

            // Dessiner la perpendiculaire du CoG à la ligne
            Gizmos.color = Color.red;
            Gizmos.DrawLine(new Vector3(closestPoint.x, 0, closestPoint.y), new Vector3(pointP.x, 0, pointP.y));

            // Optionnel: Afficher la distance comme label
            PerpendicularDistanceFromGoC = Vector2.Dot(pointP - pointA, perpendicular);
            GUIStyle style = new GUIStyle();
            style.normal.textColor = Color.white;
            Vector3 midPoint = (new Vector3(closestPoint.x, 0, closestPoint.y) + new Vector3(pointP.x, 0, pointP.y)) / 2;
            Handles.Label(midPoint, $"Distance: {PerpendicularDistanceFromGoC:F2}", style);
            Vector2 centreGauche = new Vector2(Foot_LEFT.position.x+0.3f, Foot_LEFT.position.z);
            Vector2 centreDroite = new Vector2(Foot_LEFT.position.x-0.3f, Foot_LEFT.position.z);
            distanceToCoGFromLeftFoot = Vector2.Distance(centreGauche, closestPoint);
            distanceToCoGFromRightFoot = Vector2.Distance(centreDroite, closestPoint);
        //}
        // Vector3 min = Vector3.Min(Foot_LEFT.position, Foot_RIGHT.position);
        // Vector3 max = Vector3.Max(Foot_LEFT.position, Foot_RIGHT.position);
        // Vector3 center = (min + max) / 2;
        // float width = Mathf.Abs(max.x - min.x);
        // float length = Mathf.Abs(max.z - min.z);

        // // Définir la couleur du Gizmo en fonction de la position du centre de gravité
        // Rect footRect = new Rect(min.x, min.z, width, length);
        // if (footRect.Contains(new Vector2(CoG.x, CoG.z))) {
        //     Gizmos.color = Color.green;
        // } else {
        //     Gizmos.color = Color.red;
        // }

        // // Dessiner le rectangle
        // Gizmos.DrawWireCube(new Vector3(center.x, 0.01f, center.z), new Vector3(width, 0.02f, length));
        ///////////////////////////////////////////////////////////
        ///


        // Vector3 positionLeft;
        // Vector3 positionRight;
        // if(footLeftOnGround == footRightOnGround)
        // {
        //     positionLeft = Foot_LEFT.position;
        //     positionLeft.x += 0.2f;
        //     positionRight = Foot_RIGHT.position;
        //     positionRight.x -= 0.2f;
        // }
        // else if(footLeftOnGround)
        // {
        //     positionLeft = Foot_LEFT.position;
        //     positionLeft.x += 0.2f;
        //     Vector3 test = Foot_RIGHT.position;
        //     test.x -= 1f;
        //     test.y = Foot_LEFT.position.y;
        //     positionRight = test;
        // }
        // else
        // {
        //     positionLeft = Foot_RIGHT.position;
        //     positionRight = Foot_RIGHT.position;   
        // }

        // // Calcul de l'orientation et de la distance
        // Vector3 direction = (positionRight - positionLeft).normalized;
        // float distance = Vector3.Distance(positionLeft, positionRight);

        // // Dimensions du rectangle
        // float width = Mathf.Max(1f, distance + 0.5f);  // Largeur constante du rectangle
        // float length =  0.6f;// Longueur adaptée à la distance entre les pieds

        // // Centre et rotation
        // Vector3 center = (positionLeft + positionRight) / 2;
        // Quaternion rotation = Quaternion.LookRotation(direction);

        // // Transformer le point CoG en coordonnées locales du rectangle
        // Vector3 localCoG = Quaternion.Inverse(rotation) * (CoG - center);

        // // Vérification si le CoG est à l'intérieur du rectangle en coordonnées locales
        // if (localCoG.x >= -length / 2 && localCoG.x <= length / 2 && localCoG.z >= -width / 2 && localCoG.z <= width / 2) {
        //     Gizmos.color = Color.green;
        // } else {
        //     Gizmos.color = Color.red;
        // }

        // // Dessiner le rectangle
        // Gizmos.matrix = Matrix4x4.TRS(center, rotation, Vector3.one);
        // Gizmos.DrawWireCube(Vector3.zero, new Vector3(length, 0.02f, width)); // Notez l'inversion de length et width ici
        // Gizmos.matrix = Matrix4x4.identity;


        // Vector3 footLeftPosition = Foot_LEFT.position;
        // Vector3 footRightPosition = Foot_RIGHT.position;

        // // Obtenir la rotation du pied gauche
        // Quaternion footRotation = Foot_LEFT.rotation;

        // // Calculer la direction et la distance entre les pieds
        // Vector3 direction = (footRightPosition - footLeftPosition).normalized;
        // float distance = Vector3.Distance(footLeftPosition, footRightPosition);

        // // Déterminer la largeur et la longueur du rectangle
        // float width = 0.5f; // Supposons que la largeur du pied est de 0.5 mètre
        // float length = distance; // La longueur entre les deux pieds

        // // Utiliser la rotation du pied pour ajuster les points
        // Vector3 forward = footRotation * Vector3.forward * length / 2;
        // Vector3 right = footRotation * Vector3.right * width / 2;

        // Vector3[] corners = new Vector3[4];
        // corners[0] = footLeftPosition - right - forward;
        // corners[1] = footLeftPosition + right - forward;
        // corners[2] = footLeftPosition + right + forward;
        // corners[3] = footLeftPosition - right + forward;

        // // Dessiner le rectangle
        // Gizmos.color = Color.green;
        // for (int i = 0; i < 4; i++) {
        //     Gizmos.DrawLine(corners[i], corners[(i + 1) % 4]);
        // }

        // Calculer la direction et la position moyenne entre les deux pieds
        // Vector3 midPoint2 = (Foot_LEFT.position + Foot_RIGHT.position) / 2;
        // Vector3 direction = (Foot_RIGHT.position - Foot_LEFT.position).normalized;
        // float distance = Vector3.Distance(Foot_LEFT.position, Foot_RIGHT.position);
        // Quaternion rotation = Quaternion.LookRotation(direction);

        // // Déterminer la largeur et la longueur du rectangle
        // float width = 0.9f;  // Supposons que la largeur d'un pied est de 0.5 mètre
        // float length = footLeftOnGround && footRightOnGround ? distance : distance * 0.55f;  // 75% de la distance si un pied est en l'air

        // // Ajuster le centre en fonction des pieds au sol
        // Vector3 center = midPoint2;
        // if (!footLeftOnGround || !footRightOnGround) {
        //     center = footLeftOnGround ? Foot_LEFT.position + (direction * (length / 2)) : Foot_RIGHT.position - (direction * (length / 2));
        // }

        // // Transformer le CoG pour la vérification
        // Vector3 localCoG = Quaternion.Inverse(rotation) * (CoG - center);

        // // Vérifier si le centre de gravité est dans le rectangle
        // bool isCoGInside = Mathf.Abs(localCoG.x) <= (width / 2) && Mathf.Abs(localCoG.z) <= (length / 2);
        // Gizmos.color = isCoGInside ? Color.green : Color.red;

        // // Dessiner le rectangle
        // Vector3 localForward = rotation * Vector3.forward * length / 2;
        // Vector3 localRight = rotation * Vector3.right * width / 2;

        // Vector3[] corners = new Vector3[4];
        // corners[0] = center - localRight - localForward;
        // corners[1] = center + localRight - localForward;
        // corners[2] = center + localRight + localForward;
        // corners[3] = center - localRight + localForward;

        // for (int i = 0; i < 4; i++) {
        //     Gizmos.DrawLine(corners[i], corners[(i + 1) % 4]);
        // }

    }

    private void CaluculCog()
    {
        float floorLevel = (Foot_LEFT.position.y + Foot_RIGHT.position.y) / 2;
        Vector3 floorCoG = CalculateCenterOfGravity();
        floorCoG = new Vector3(floorCoG.x, floorLevel, floorCoG.z);
        Vector2 pointA = new Vector2(Foot_LEFT.position.x, Foot_LEFT.position.z-0.06f);
        Vector2 pointB = new Vector2(Foot_RIGHT.position.x, Foot_RIGHT.position.z-0.06f);
        Vector2 pointP = new Vector2(floorCoG.x, floorCoG.z);
        Vector2 lineDirection = (pointB - pointA).normalized;
        float t = Vector2.Dot(pointP - pointA, lineDirection);
        Vector2 closestPoint = pointA + t * lineDirection;

        Vector2 centreGauche = new Vector2(Foot_LEFT.position.x+0.3f, Foot_LEFT.position.z);
        Vector2 centreDroite = new Vector2(Foot_LEFT.position.x-0.3f, Foot_LEFT.position.z);

        distanceToCoGFromLeftFoot = Vector2.Distance(centreGauche, closestPoint);
        distanceToCoGFromRightFoot = Vector2.Distance(centreDroite, closestPoint);
    }


    public int OrientationTorseFloor()
    {
        // Calcul de l'angle entre le forward du torse et l'up global
        float angleWithVertical = Vector3.Angle(Torse.forward, Vector3.up);

        // Détermination si l'agent est couché ou debout
        if ((angleWithVertical > 80 && angleWithVertical < 140) || currentProportionalHeadHeight > 0.8)
        {
            // L'agent est debout ou à l'envers
            faceOnTheFloor = 0;
            return 0;
        }
        else
        {
            // L'agent est couché
            //Debug.Log("L'agent est couché");
            // Utilisation du produit scalaire pour déterminer si sur le dos ou le ventre
            float dotProduct = Vector3.Dot(Torse.forward, Vector3.down);
            Vector3 CentreDeGravité = CalculateCenterOfGravity();
            //Vector3 piedsEnAvant = Bassin.InverseTransformPoint((Foot_LEFT.position + Foot_RIGHT.position)/2);
            Vector3 piedsEnAvant = (Foot_LEFT.position + Foot_RIGHT.position)/2 - CentreDeGravité;
            //Debug.Log(piedsEnAvant);

            if (dotProduct > 0 && piedsEnAvant.z < 0) // Regarde vers le bas
            {
                faceOnTheFloor = 1;
                return 1;
            }
            else
            {
                faceOnTheFloor = -1;
                return -1;
            }
        }
    }

    public void AddLog(string def, float value, StatAggregationMethod type=StatAggregationMethod.Average)
    {
        Academy.Instance.StatsRecorder.Add(def, value, type);
    }

     //Update OrientationCube and DirectionIndicator
    void UpdateOrientationObjects()
    {
        m_WorldDirToWalk = target.position - Bassin.position;
        m_OrientationCube.UpdateOrientation(Bassin, target);
        if (m_DirectionIndicator)
        {
            m_DirectionIndicator.MatchOrientation(m_OrientationCube.transform);
        }
    }
    public void CatchTarget()
    {
        reward(targetConsecutive*100 + 1f, "Cube touched",StatAggregationMethod.Sum);
        targetConsecutive++;
    }
}

