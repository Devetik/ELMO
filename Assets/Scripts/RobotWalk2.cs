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
using Unity.Mathematics;
using System.Linq;
using Unity.Burst.Intrinsics;
using Unity.Barracuda;
using Unity.MLAgents.Policies;



public class RobotWalk : Agent
{
    public bool enTest;
    public bool enTestUnit = false;
    private Vector3[] corners;
    private bool isCoGInside = false;
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
    const float m_maxWalkingSpeed = 5; //The max walking speed
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
    public ConfigurableJoint ankleLeft;
    public ConfigurableJoint ankleRight;
    public ConfigurableJoint kneeLeft;
    public ConfigurableJoint kneeRight;
    [Header("Body Parts")] 
    public Transform Head;
    public Transform Bassin;
    public Transform Foot_LEFT;
    public Transform Foot_RIGHT;
    public Transform Tibias_LEFT;
    public Transform Tibias_RIGHT;
    public Transform Cuisse_LEFT;
    public Transform Cuisse_RIGHT;
    //public Transform Abdos;
    public Transform V1;
    public Transform V2;
    public Transform V3;
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
    private const float normalHeadHeight = 4.50f;
    private const float normalBassinHeight = 2.43f;
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
    private Transform footInFront;
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
    public TextMeshPro textPosition;
    public float currentProportionalHeadHeight = 0f;
    public float currentProportionalBassinHeight = 0f;
    public float currentHeadHeight = 0f;
    public float currentBassinHeight = 0f;
    public int faceOnTheFloor;
    private int lastFaceOnTheFloor;
    private int curriculumStep;
    public Vector3 startPosition;
    public Quaternion startRotation;
    JointDriveController m_JdController;
    EnvironmentParameters m_ResetParams;
    Vector3 CoG;
    private Vector3 m_WorldDirToWalk = Vector3.right;
    private bool canGoForward = false;
    float speedBeforeOrientate = 0f;
    private bool footLeftOnGround;
    private bool footRightOnGround;
    private int lastFootInProgress = 0; //1 = left -1 = right
    private float totalFatigue;
    private int startPositionNumber;
    Vector2 avgPositionAtStart;
    float distanceFromCoG;
    bool isInCoGCircle;
    private Transform currentAdvancingFoot;
    private Transform currentSupportFoot;
    private float currentLeftFootHeight;
    private float currentRightFootHeight;
    private float maxVelocity = 5f;
    private float lastAdvancingFootVelocity;
    private float lastAdvancingFootHeight;
    private float topAdvancedFootHeight;
    private float supportValue = 0f;
    private float supportFootCoG = 0f;
    private float positionZFeetAtStart;
    private bool leftFootStable;
    private bool rightFootStable;
    private Transform lastAdvancingFoot;
    public float timeSinceLastStep;
    public TargetController targetController;
    private Vector3 m_OriniePosition;
    private Quaternion m_OriginRotation;
    private Vector3 AverageVelocity;
    private float distanceFootLeftToHandRight;
    private float distanceFootRightToHandLeft;
    public NNModel riseFromback;
    public NNModel riseFromFace;
    public NNModel walk;
    private BehaviorParameters behaviorParameters;
    private int targetCount;
    

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;

    //The indicator graphic gameobject that points towards the target
    DirectionIndicator m_DirectionIndicator;

    public override void Initialize()
    {
        behaviorParameters = GetComponent<BehaviorParameters>();
        enTest = false;
        enTestUnit = GameObject.Find("Test") != null? true:false;
        m_OriniePosition = Agent.localPosition;
        m_OriginRotation = Agent.localRotation;
        
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
        m_DirectionIndicator = GetComponentInChildren<DirectionIndicator>();

        m_JdController = GetComponent<JointDriveController>();
        m_JdController.SetupBodyPart(Bassin, "Bassin");
        m_JdController.SetupBodyPart(Head, "Head");
        m_JdController.SetupBodyPart(Torse, "Torse");
        //m_JdController.SetupBodyPart(Abdos, "Abdos");
        m_JdController.SetupBodyPart(V1, "V1");
        m_JdController.SetupBodyPart(V2, "V2");
        m_JdController.SetupBodyPart(V3, "V3");
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
    }

    public override void OnEpisodeBegin()
    {
        decayRate = 1f;
        targetCount = 1;
        timeSinceLastStep = 0f;
        currentAdvancingFoot = null;
        currentSupportFoot = null;
        lastAdvancingFoot = null;
        //startPositionNumber = UnityEngine.Random.Range(enTestUnit?2:0, 10);
        startPositionNumber = 2;

        if(startPositionNumber >= 2)//debout
        {
            float footWidth = 2.0f;  // Largeur des pieds
            float footLength = 1f; // Longueur des pieds
            float maxRaycastHeight = 5.0f;
            Vector3 newPosition = m_OriniePosition;  // Position de départ pour les calculs
            newPosition.x = enTestUnit ? m_OriniePosition.x : m_OriniePosition.x + UnityEngine.Random.Range(-1, 1);
            newPosition.z = enTestUnit ? m_OriniePosition.z : m_OriniePosition.z + UnityEngine.Random.Range(-1, 1);
            Quaternion randomRotation = enTestUnit? m_OriginRotation:  Quaternion.Euler(UnityEngine.Random.Range(m_OriginRotation.eulerAngles.x-0, m_OriginRotation.eulerAngles.x+20), UnityEngine.Random.Range(m_OriginRotation.eulerAngles.y-30, enTestUnit||enTest?0f:m_OriginRotation.eulerAngles.y+30), m_OriginRotation.eulerAngles.z);
            Quaternion rotation = randomRotation;
            Vector3 forward = rotation * Vector3.forward;
            Vector3 right = rotation * Vector3.right;

            Vector3 forwardLeft = newPosition + forward * footLength / 2 - right * footWidth / 2;
            Vector3 forwardRight = newPosition + forward * footLength / 2 + right * footWidth / 2;
            Vector3 backLeft = newPosition - forward * footLength / 2 - right * footWidth / 2;
            Vector3 backRight = newPosition - forward * footLength / 2 + right * footWidth / 2;

            Vector3[] points = { forwardLeft, forwardRight, backLeft, backRight };
            float[] heights = new float[4];
  
            int layerMask = 1 << LayerMask.NameToLayer("Agent");
            layerMask = ~layerMask;

            for (int i = 0; i < points.Length; i++) {
                RaycastHit hit;
                if (Physics.Raycast(points[i] + Vector3.up * maxRaycastHeight, Vector3.down, out hit,maxRaycastHeight + 10f, layerMask)) {
                    heights[i] = hit.point.y;
                    //Vector3 rayStart = points[i] + Vector3.up * maxRaycastHeight;
                    //Debug.DrawRay(rayStart, Vector3.down * maxRaycastHeight, hit.collider ? Color.blue : Color.red, 1f);
                } 
                else 
                {
                    heights[i] = newPosition.y; // Utilisez la position y de départ comme valeur par défaut si rien n'est touché
                }
            }

            newPosition.y = heights.Max() + 0.0f;  // Ajoute une petite marge pour éviter le clipping avec le sol
            Agent.localPosition = newPosition;

            // Ajustement de la rotation
            Agent.localRotation = randomRotation;

            
        }
        else if(startPositionNumber == 0)//sur le dos
        {
            Agent.localPosition = enTestUnit? Agent.localPosition: new Vector3(UnityEngine.Random.Range(-1,1),m_OriniePosition.y+0.75f,m_OriniePosition.z);
            Agent.localRotation = Quaternion.Euler(UnityEngine.Random.Range(-20, -60), UnityEngine.Random.Range(0f, 360f), UnityEngine.Random.Range(-30, 30));
        }
        else if(startPositionNumber == 1)//face en avant
        {
            Agent.localPosition = enTestUnit? Agent.localPosition: new Vector3(UnityEngine.Random.Range(-1,1),m_OriniePosition.y+0.75f,m_OriniePosition.z);
            Agent.localRotation = Quaternion.Euler(UnityEngine.Random.Range(20, 90), UnityEngine.Random.Range(0f, 360f), UnityEngine.Random.Range(-30, 30));
        }

        targetController.MoveTargetToRandomPosition();
        
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }
        UpdateOrientationObjects();

        //Set our goal walking speed
        MTargetWalkingSpeed =
            randomizeWalkSpeedEachEpisode ? UnityEngine.Random.Range(1f, m_maxWalkingSpeed) : MTargetWalkingSpeed;
        if(randomizeWalkSpeedEachEpisode || speedBeforeOrientate == 0f)
        {
            speedBeforeOrientate = randomizeWalkSpeedEachEpisode ? UnityEngine.Random.Range(1f, m_maxWalkingSpeed) : MTargetWalkingSpeed;
        }

        enchainementStep = 1;
        totalFatigue = 0f;
        avgPositionAtStart = new Vector2(GetAvgPosition().x, GetAvgPosition().z);
        distanceToTargetAtStart = Vector3.Distance(GetAvgPosition(), target.position);
        m_OrientationCube.transform.rotation = Agent.localRotation;

    }

    public void Reward(float value, string title="",StatAggregationMethod type=StatAggregationMethod.Sum)
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
        bpDict[Shoulder_LEFT].SetJointTargetRotation(bpDict[Shoulder_LEFT],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.shoulderL1 = i-2;vc.shoulderL2 = i-1; vc.shoulderL3 = i;
        bpDict[Coude_LEFT].SetJointTargetRotation(bpDict[Coude_LEFT],continuousActions[++i], 0, 0, false);vc.coudeL = i;
        //Right side
        bpDict[Cuisse_RIGHT].SetJointTargetRotation(bpDict[Cuisse_RIGHT],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.cuisseR1 = i-2;vc.cuisseR2 = i-1;vc.cuisseR3 = i;
        bpDict[Tibias_RIGHT].SetJointTargetRotation(bpDict[Tibias_RIGHT],continuousActions[++i], 0, 0, false);vc.tibiasR = i;
        bpDict[Foot_RIGHT].SetJointTargetRotation(bpDict[Foot_RIGHT],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.footR1=i-2;vc.footR2=i-1;vc.footR3=i;
        bpDict[Shoulder_RIGHT].SetJointTargetRotation(bpDict[Shoulder_RIGHT],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.shoulderR1 = i-2;vc.shoulderR2 = i-1; vc.shoulderR3 = i;
        bpDict[Coude_RIGHT].SetJointTargetRotation(bpDict[Coude_RIGHT],continuousActions[++i], 0, 0, false);vc.coudeR = i;
        //Center
        //bpDict[Bassin].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        //bpDict[Abdos].SetJointTargetRotation(bpDict[Abdos],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.abdo1 = i-2;vc.abdo2 = i-1;vc.abdo3 = i;
        bpDict[V1].SetJointTargetRotation(bpDict[V1],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.V11 = i-2;vc.V12 = i-1;vc.V13 = i;
        bpDict[V2].SetJointTargetRotation(bpDict[V2],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.V21 = i-2;vc.V22 = i-1;vc.V23 = i;
        bpDict[V3].SetJointTargetRotation(bpDict[V3],continuousActions[++i], continuousActions[++i], continuousActions[++i], false);vc.V31 = i-2;vc.V32 = i-1;vc.V33 = i;
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
        //bpDict[Abdos].SetJointStrength(continuousActions[++i]);
        bpDict[V1].SetJointStrength(continuousActions[++i]);
        bpDict[V2].SetJointStrength(continuousActions[++i]);
        bpDict[V3].SetJointStrength(continuousActions[++i]);
        bpDict[Torse].SetJointStrength(continuousActions[++i]);
        bpDict[Head].SetJointStrength(continuousActions[++i]);
        //stepReward.Walk();//26
    }


    void FixedUpdate()
    {
        decayRate -= 0.8f / Math.Max(MaxStep, 10);
        Vector2 currentPosition = new Vector2(GetAvgPosition().x, GetAvgPosition().z);

        totalFatigue = 0f;
        var bpDict = m_JdController.bodyPartsDict;
        int count = 0;
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            if (bodyPart.joint)
            {
                totalFatigue += bodyPart.muscleFatigue;
                count++;
            }
        }
        totalFatigue /= 13f;
        UpdateOrientationObjects();
        footLeftOnGround = IsInContact(Foot_LEFT);
        footRightOnGround = IsInContact(Foot_RIGHT);

        var cubeForward = m_OrientationCube.transform.forward;

        // b. Rotation alignment with target direction.
        //This Reward will approach 1 if it faces the target direction perfectly and approach zero as it deviates
        var headForward = Head.forward;
        headForward.y = 0;
        // var lookAtTargetReward = (Vector3.Dot(cubeForward, head.forward) + 1) * .5F;
        //var lookAtTargetReward = (Vector3.Dot(cubeForward, headForward) + 1) * .5F;
        Vector3 targetDirrection = target.position - Bassin.position;
        targetDirrection.y = 0f;
        Vector3 bassinDirrection = Bassin.forward;
        bassinDirrection.y = 0f;
        var lookAtTargetReward = 1f-Vector3.Angle(bassinDirrection, targetDirrection)/180f;//(Vector3.Dot(cubeForward, headForward) + 1) * .5F;
        //Debug.Log(lookAtTargetReward);

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
            MTargetWalkingSpeed = 0f;//MTargetWalkingSpeed = speedBeforeOrientate;//*((lookAtTargetReward-0.9f)*10);
        }
        // Set Reward for this step according to mixture of the following elements.
        // a. Match target speed
        //This Reward will approach 1 if it matches perfectly and approach zero as it deviates
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
        AddLog("Match Speed", matchSpeedReward);
        AddLog("Look At Target", lookAtTargetReward);
        
        if(faceOnTheFloor == 1) // Si l'agent est face au sol
        {
            if(lastFaceOnTheFloor != faceOnTheFloor)
            {
                // if(lastFaceOnTheFloor == -1)
                // {
                //     Reward(500f);
                // }
                lastFaceOnTheFloor = faceOnTheFloor;
            }
            Reward(-5f);
            EndEpisode();
            float distancePenalty = Vector2.Distance(avgPositionAtStart, currentPosition) < 3f? 0.1f: Vector2.Distance(avgPositionAtStart, currentPosition)*-1f+3f;
            Reward(distancePenalty*2f);
            //Reward(-0.1f);
            //behaviorParameters.Model = riseFromFace;
            Reward(FaceOnGroundWakeUpReward()*decayRate);
        }
        else if( faceOnTheFloor == -1) // Si l'agent est sur le dos
        {
            if(lastFaceOnTheFloor != faceOnTheFloor)
            {
                lastFaceOnTheFloor = faceOnTheFloor;
            }
            Reward(-5f);
            EndEpisode();
            //m_JdController.maxJointSpring = 1000000;
            //behaviorParameters.Model = riseFromback;
            Reward(lookAtTargetReward*10F);
            Reward(OnBackWakeUpReward()*decayRate);
        }
        else
        {
            if(lastFaceOnTheFloor != faceOnTheFloor)
            {
                // if(lastFaceOnTheFloor == -1 || lastFaceOnTheFloor == 1)
                // {
                //     Reward(1000f);
                // }
                lastFaceOnTheFloor = faceOnTheFloor;
            }
            //behaviorParameters.Model = walk;
            //Reward(50f * currentProportionalHeadHeight*decayRate);
            //EndEpisode();
            //textPosition.text = "Debout";
            // Reward(1-distanceFromCoG);
            Reward(isInCoGCircle ? 1f:-1f);
            Reward(matchSpeedReward*100f * lookAtTargetReward, "Look at Target", StatAggregationMethod.Average);
            Reward(StandingUpReward());
            //Reward(lookAtTargetReward*decayRate*5f);
            //float distancePenalty = Vector2.Distance(avgPositionAtStart, currentPosition) < 1f? 0.1f: Vector2.Distance(avgPositionAtStart, currentPosition)*-1f+1f;
            //Reward(distancePenalty);
            // if(lookAtTargetReward > 0.95f)
            // {
            //     Reward(500f*targetCount);
            //     targetController.MoveTargetToRandomPosition();
            //     targetCount++;
            //     //EndEpisode();
            // }
            //Debug.Log(StandingUpReward());
            
            // Reward(GetArmFeetSyncReward());
            // Reward(ShoulderStabilityReward());
        }

        Reward(totalFatigue/1000);
        //Reward(-1f / MaxStep);

        canGoForward = lookAtTargetReward > 0.9;


        // //Reward(CalculateCoGReward()*matchSpeedReward);
        UpdateFootPosition();
        if(!footLeftOnGround && !footRightOnGround)
        {
            Reward(-1f);
        }
        //Reward((0.5f-distanceFromCoG)*100f);
        // //Reward(ApplyStepReward()*lookAtTargetReward, "Foot steps",StatAggregationMethod.Sum);
        // //Reward(matchSpeedReward, "Matche Speed", StatAggregationMethod.Average);

        // //Debug.Log(Math.Max(0f,Math.Min(CalculateCoGReward(),1f)));
        // Vector2 currentTorsePosition = new Vector2(Torse.position.x, Torse.position.z);
        // Reward(-Vector2.Distance(currentTorsePosition, positionTorse)/10f);
        // if(currentProportionalHeadHeight > 0.9f && (bpDict[Foot_LEFT].groundContact.touchingGround || bpDict[Foot_RIGHT].groundContact.touchingGround))
        // {
        //     Reward(1f);
        // }

    }
    float StandingUpReward()
    {
        float ret = 0f;
        ret += (100f-Vector3.Angle(Bassin.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(V1.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(V2.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(V3.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(Torse.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(Head.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(Shoulder_LEFT.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(Shoulder_RIGHT.up, Vector3.up))/1000f;
        ret += currentProportionalHeadHeight *0.1f;
        ret += currentProportionalBassinHeight *0.1f;
        return ret;
    }
    float LeveGenouxReward()
    {
        float ret = 0f;  // Valeur de départ de la récompense

        Vector3 pelvisDirection = Bassin.up;
        Vector3 thighLeftDirection = Cuisse_LEFT.up;
        Vector3 shinLeftDirection = Tibias_LEFT.up;
        Vector3 thighRightDirection = Cuisse_RIGHT.up;
        Vector3 shinRightDirection = Tibias_RIGHT.up;

        // Calculer les angles entre les directions pertinentes
        float pelvisThighLeft = Vector3.Angle(pelvisDirection, thighLeftDirection);
        float pelvisThighRight = Vector3.Angle(pelvisDirection, thighRightDirection);
        float kneeAngleLeft = Vector3.Angle(thighLeftDirection, shinLeftDirection);
        float kneeAngleRight = Vector3.Angle(thighRightDirection, shinRightDirection);

        // Calculer la récompense pour chaque côté en prenant le minimum des deux angles et en le normalisant sur 30°
        float leftReward = Mathf.Min(pelvisThighLeft, kneeAngleLeft) / 60.0f;
        float rightReward = Mathf.Min(pelvisThighRight, kneeAngleRight) / 60.0f;

        // S'assurer que la récompense ne dépasse pas 1
        leftReward = Mathf.Clamp(leftReward, 0, 1);
        rightReward = Mathf.Clamp(rightReward, 0, 1);

        // Utiliser la moyenne des récompenses des deux côtés
        ret = Math.Max(leftReward, rightReward);

        return ret;
    }
    float ShoulderStabilityReward()
    {
        float ret = 0f;
        float velocityThreshold = 0.5f;
        float shoulderLeftVelocity = m_JdController.bodyPartsDict[Shoulder_LEFT].rb.velocity.y;
        float shoulderRightVelocity = m_JdController.bodyPartsDict[Shoulder_RIGHT].rb.velocity.y;

        if (Mathf.Abs(shoulderLeftVelocity) <= velocityThreshold && Mathf.Abs(shoulderRightVelocity) <= velocityThreshold)
        {
            // Récompenser l'agent si les deux vélocités sont en dessous du seuil
             ret = 0.1f;  // Vous pouvez ajuster cette valeur selon le cas
        }
        else
        {
            // Calculer la pénalité proportionnelle si une des vélocités dépasse le seuil
            if (Mathf.Abs(shoulderLeftVelocity) > velocityThreshold)
            {
                ret = -Mathf.Abs(shoulderLeftVelocity - velocityThreshold) * 0.1f; // Pénalité proportionnelle
            }
            if (Mathf.Abs(shoulderRightVelocity) > velocityThreshold)
            {
                ret = -Mathf.Abs(shoulderRightVelocity - velocityThreshold) * 0.1f; // Pénalité proportionnelle
            }
        }
        AddLog("Shoulder Velocity", ret);
        //Debug.Log(ret);
        return ret;
    }
    float LeveGenouxRewardV2()
    {
        float ret = 0f;
        Vector2 rotuleLeft = new Vector2(Tibias_LEFT.TransformPoint(kneeLeft.anchor).x, Tibias_LEFT.TransformPoint(kneeLeft.anchor).z);
        Vector3 rotuleRight = new Vector2(Tibias_RIGHT.TransformPoint(kneeRight.anchor).x,Tibias_RIGHT.TransformPoint(kneeRight.anchor).z);

        Vector3 chevilleLeft = new Vector2(Foot_LEFT.TransformPoint(ankleLeft.anchor).x,Foot_LEFT.TransformPoint(ankleLeft.anchor).z);
        Vector3 chevilleGauche = new Vector2(Foot_RIGHT.TransformPoint(ankleRight.anchor).x,Foot_RIGHT.TransformPoint(ankleRight.anchor).z);

        //textPosition.text = 
        return ret;
    }
    float OnBackWakeUpReward()
    {

        // float angleTorseCuisse = Vector3.Angle(Cuisse_RIGHT.up, Torse.up);
        // ret += angleTorseCuisse/156f;

        // float angleCuisseTibiasL = Vector3.Angle(Cuisse_RIGHT.up, Tibias_RIGHT.up)/117f;
        // ret += angleCuisseTibiasL;
        // float angleCuisseTibiasR = Vector3.Angle(Cuisse_LEFT.up, Tibias_LEFT.up)/117f;
        // ret += angleCuisseTibiasR;


        //ret += currentProportionalHeadHeight;

        // ret += footLeftOnGround && footRightOnGround ? 1f : 0f;

        // float distancePiedBassin = Vector3.Distance((Foot_LEFT.localPosition + Foot_RIGHT.localPosition)/2, Bassin.localPosition)/1.94f;
        
        // Reward(-1f*(distancePiedBassin-1f)*5f);
        // Vector3 pelvisDirection = Abdos.up;
        // Vector3 thighLeftDirection = Cuisse_LEFT.up;
        // Vector3 thighRightDirection = Cuisse_RIGHT.up;
        // float abdoLeftFoot = Vector3.Angle(pelvisDirection, thighLeftDirection)/130f;
        // float abdoRightFoot = Vector3.Angle(pelvisDirection, thighRightDirection)/130f;
        // float leftReward = Mathf.Min(leftReward, kneeAngleLeft) / 60.0f;
        // leftReward = Mathf.Clamp(leftReward, 0, 1);
        // if(footLeftOnGround || footRightOnGround)
        // {
        //     Reward((abdoLeftFoot+abdoRightFoot)*100 );
        // }
        float ret = 0f;
        float handShoulderLeftDistance = (Shoulder_LEFT.position.y - Hand_LEFT.position.y)/1.8f;
        float leftArmDown = Vector3.Dot((Arm_LEFT.up + Forearm_LEFT.up)/2f, Vector3.up);
        float rightArmUp = Vector3.Dot((Arm_RIGHT.up + Forearm_RIGHT.up)/2f, Vector3.down);
        float handShoulderRightDistance = (Hand_RIGHT.position.y-Shoulder_RIGHT.position.y)/1.8f;

        ret += handShoulderLeftDistance;
        ret += handShoulderRightDistance;
        ret += leftArmDown;
        ret += rightArmUp;

        float retournementLateral = Math.Max(Math.Abs(Vector3.Dot(Torse.forward, Vector3.left)), Math.Abs(Vector3.Dot(Torse.forward, Vector3.forward)));
        float retournementLateralBassin = Math.Max(Math.Abs(Vector3.Dot(Bassin.forward, Vector3.left)), Math.Abs(Vector3.Dot(Bassin.forward, Vector3.forward)));
        float retournementLateralCL = Math.Max(Math.Abs(Vector3.Dot(Cuisse_LEFT.forward, Vector3.left)), Math.Abs(Vector3.Dot(Cuisse_LEFT.forward, Vector3.forward)));
        float retournementLateralCR = Math.Max(Math.Abs(Vector3.Dot(Cuisse_RIGHT.forward, Vector3.left)), Math.Abs(Vector3.Dot(Cuisse_RIGHT.forward, Vector3.forward)));
        float retournementLateralTL = Math.Max(Math.Abs(Vector3.Dot(Tibias_LEFT.forward, Vector3.left)), Math.Abs(Vector3.Dot(Tibias_LEFT.forward, Vector3.forward)));
        float retournementLateralTR = Math.Max(Math.Abs(Vector3.Dot(Tibias_RIGHT.forward, Vector3.left)), Math.Abs(Vector3.Dot(Tibias_RIGHT.forward, Vector3.forward)));

        ret += retournementLateral*2f;
        ret += retournementLateralBassin;
        ret += retournementLateralCL;
        ret += retournementLateralCR;
        ret += retournementLateralTL;
        ret += retournementLateralTR;
        ret += currentProportionalHeadHeight;

        return ret;
    }
    float FaceOnGroundWakeUpReward()
    {
        float ret = 0f;
        ret += (currentProportionalHeadHeight-0.5f)*currentProportionalBassinHeight;
        ret += (currentProportionalBassinHeight-0.5f)*5f;

        ret += (100f-Vector3.Angle(Foot_LEFT.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(Foot_RIGHT.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(Tibias_LEFT.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(Tibias_RIGHT.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(Cuisse_LEFT.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(Cuisse_RIGHT.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(Torse.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(Bassin.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(V1.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(V2.up, Vector3.up))/1000f;
        ret += (100f-Vector3.Angle(V3.up, Vector3.up))/1000f;
        //ret += Math.Max(Math.Abs(Vector3.Dot(Cuisse_LEFT.forward, Vector3.left, Math.Abs(Vector3.Dot(Vector3))))

        float handShoulderLeftDistance = (Shoulder_LEFT.position.y - Hand_LEFT.position.y)/1.8f;
        float handShoulderRightDistance = (Shoulder_RIGHT.position.y - Hand_RIGHT.position.y)/1.8f;
        ret += handShoulderLeftDistance + handShoulderRightDistance;

        Vector3 worldAnchorLeft = new Vector3(Foot_LEFT.TransformPoint(ankleLeft.anchor).x,0f, Foot_LEFT.TransformPoint(ankleLeft.anchor).z);
        Vector3 worldAnchorRight = new Vector3(Foot_RIGHT.TransformPoint(ankleRight.anchor).x,0f, Foot_RIGHT.TransformPoint(ankleRight.anchor).z);
        Vector3 CoGFloor = CoG;
        CoGFloor.y = 0f;
        float distanceCoGLeft = 1f-Vector3.Distance(worldAnchorLeft, CoGFloor);
        float distanceCoGRight = 1f-Vector3.Distance(worldAnchorRight, CoGFloor);
        ret += distanceCoGLeft * currentProportionalBassinHeight;
        ret += distanceCoGRight * currentProportionalBassinHeight;
        Vector3 velo = GetAvgVelocity();
        float totalVelocity = Math.Abs(velo.x)+Math.Abs(velo.y)+Math.Abs(velo.z);
        float velocityReward = totalVelocity < 4f ? 0.1f: totalVelocity*-1f+4f;
        return ret;
    }
    float CalculateCoGReward(bool staticPosition = false) 
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

        Vector2 gaucheZ = pointA;
        Vector2 droiteZ = pointB;
        gaucheZ.x = 0;
        droiteZ.x = 0;
        float distancePieds = Vector2.Distance(gaucheZ, droiteZ);//Distance max de 2.8f

        if(staticPosition)
        {
            if(distancePieds < 0.5f)
            {
                if(PerpendicularDistanceFromGoC <0)
                {
                    return 1+PerpendicularDistanceFromGoC;
                }else
                {
                    return 1-PerpendicularDistanceFromGoC;
                }
            }
        }
        else
        {
            if(PerpendicularDistanceFromGoC <0)
            {
                return 1+PerpendicularDistanceFromGoC;
            }else
            {
                return 1-PerpendicularDistanceFromGoC;
            }
        }
        return 0f;
        
    }

    //normalized value of the difference in avg speed vs goal walking speed.
    public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
    {
        //distance between our actual velocity and goal velocity
        var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, MTargetWalkingSpeed);

        //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //This Reward will approach 1 if it matches perfectly and approach zero as it deviates
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
        // Reward(1f, "Cube touched",StatAggregationMethod.Sum);
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

    
    Vector3 GetAvgPosition()
    {
        Vector3 posSum = Vector3.zero;

        //ALL RBS
        int numOfRb = 0;
        foreach (var item in m_JdController.bodyPartsList)
        {
            numOfRb++;
            posSum += item.rb.transform.position;
        }

        var avgVel = posSum / numOfRb;
        return avgVel;
    }


    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        if (Input.GetKey(KeyCode.UpArrow))
        {
            continuousActionsOut[vc.V11] = 1.0f;
            continuousActionsOut[vc.V21] = 1.0f;
            continuousActionsOut[vc.V31] = 1.0f;
            continuousActionsOut[vc.torse1] = 1.0f;
            continuousActionsOut[vc.shoulderL1] = 1.0f;
            continuousActionsOut[vc.shoulderR1] = 1.0f;
            //continuousActionsOut[vc.abdo1] = -1.0f;
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
            continuousActionsOut[vc.V11] = -1.0f;
            continuousActionsOut[vc.V21] = -1.0f;
            continuousActionsOut[vc.V31] = -1.0f;
            continuousActionsOut[vc.torse1] = -1.0f;
            continuousActionsOut[vc.shoulderL1] = -1.0f;
            continuousActionsOut[vc.shoulderR1] = -1.0f;
            //continuousActionsOut[vc.abdo1] = -1.0f;
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
            continuousActionsOut[vc.torse2] = 1.0f;
            continuousActionsOut[vc.V12] = 1.0f;
            continuousActionsOut[vc.V22] = 1.0f;
            continuousActionsOut[vc.V32] = 1.0f;
            // continuousActionsOut[vc.tibiasL] = 1.0f;
            // continuousActionsOut[vc.tibiasR] = 1.0f;
            // continuousActionsOut[vc.tibiasL] = 1.0f;
            // continuousActionsOut[vc.tibiasR] = 1.0f;
            
            // continuousActionsOut[vc.torse2] = 1.0f;
            // continuousActionsOut[vc.abdo2] = 1.0f;
            // continuousActionsOut[vc.head2] = 1.0f;
            // continuousActionsOut[vc.footL1] = 1.0f;
            // continuousActionsOut[vc.footR1] = 1.0f;
            // continuousActionsOut[vc.footL2] = 1.0f;
            // continuousActionsOut[vc.footR2] = 1.0f;
            // continuousActionsOut[vc.footL3] = 1.0f;
            // continuousActionsOut[vc.footR3] = 1.0f;
        }
        else if (Input.GetKey(KeyCode.LeftArrow))
        {
            continuousActionsOut[vc.torse2] = -1.0f;
            continuousActionsOut[vc.V12] = -1.0f;
            continuousActionsOut[vc.V22] = -1.0f;
            continuousActionsOut[vc.V32] = -1.0f;
            // continuousActionsOut[vc.tibiasL] = -1.0f;
            // continuousActionsOut[vc.tibiasR] = -1.0f;
            // continuousActionsOut[vc.tibiasL] = -1.0f;
            // continuousActionsOut[vc.tibiasR] = -1.0f;
            
            // continuousActionsOut[vc.torse2] = -1.0f;
            // continuousActionsOut[vc.abdo2] = -1.0f;
            // continuousActionsOut[vc.head2] = -1.0f;
            // continuousActionsOut[vc.footL1] = -1.0f;
            // continuousActionsOut[vc.footR1] = -1.0f;
            // continuousActionsOut[vc.footL2] = -1.0f;
            // continuousActionsOut[vc.footR2] = -1.0f;
            // continuousActionsOut[vc.footL3] = -1.0f;
            // continuousActionsOut[vc.footR3] = -1.0f;
        }


        if (Input.GetKey(KeyCode.C))
        {
            continuousActionsOut[vc.torse3] = 1.0f;
            continuousActionsOut[vc.V13] = 1.0f;
            continuousActionsOut[vc.V23] = 1.0f;
            continuousActionsOut[vc.V33] = 1.0f;
            // continuousActionsOut[vc.shoulderL3] = 1.0f;
            // continuousActionsOut[vc.shoulderR3] = 1.0f;
            // continuousActionsOut[vc.head3] = 1.0f;
        }
        else if (Input.GetKey(KeyCode.V))
        {
            continuousActionsOut[vc.torse3] = -1.0f;
            continuousActionsOut[vc.V13] = -1.0f;
            continuousActionsOut[vc.V23] = -1.0f;
            continuousActionsOut[vc.V33] = -1.0f;
            // continuousActionsOut[vc.shoulderL3] = -1.0f;
            // continuousActionsOut[vc.shoulderR3] = -1.0f;
            // continuousActionsOut[vc.head3] = -1.0f;
        }

        if (Input.GetKey(KeyCode.B))
        {
            continuousActionsOut[vc.coudeL] = 1.0f;
            continuousActionsOut[vc.coudeR] = 1.0f;
        }
        else if (Input.GetKey(KeyCode.N))
        {
            continuousActionsOut[vc.coudeL] = -1.0f;
            continuousActionsOut[vc.coudeR] = -1.0f;
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
        sensor.AddObservation(timeSinceLastStep/10f);
        sensor.AddObservation((Bassin.InverseTransformPoint(Foot_LEFT.position).z - Bassin.InverseTransformPoint(Hand_RIGHT.position).z)*10f);
        sensor.AddObservation((Bassin.InverseTransformPoint(Foot_RIGHT.position).z - Bassin.InverseTransformPoint(Hand_LEFT.position).z)*10f);

        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(avgVel));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(velGoal));
        sensor.AddObservation(Quaternion.FromToRotation(Bassin.forward, cubeForward));
        sensor.AddObservation(Quaternion.FromToRotation(Head.forward, cubeForward));
        sensor.AddObservation(m_OrientationCube.transform.InverseTransformPoint(target.transform.position));
        sensor.AddObservation(Vector3.Distance(velGoal, avgVel));

        sensor.AddObservation(currentHeadHeight);
        sensor.AddObservation(currentBassinHeight);
        sensor.AddObservation(MeasureGroundDistance(Foot_LEFT, 2f));
        sensor.AddObservation(MeasureGroundDistance(Foot_RIGHT, 2f));
        sensor.AddObservation(currentProportionalHeadHeight);
        sensor.AddObservation(currentProportionalBassinHeight);

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
            //Reward(Mathf.Abs(lateralDistance) * lateralPenalty);
            ret -= System.Math.Abs(lateralDistance * 10f + 1f);
            log += " et latérale " + lateralDistance;
        }

        // Mise à jour des positions et rotations pour la prochaine évaluation
        lastPosition = Torse.position;
        lastRotation = Torse.rotation;
        //Debug.Log(log);
        return ret;
    }

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

        float Reward;

        if (headHeight >= lowerBound && headHeight <= upperBound)
        {
            Reward = baseValue;
        }
        else if (effectiveDistance <= tolerance)
        {
            Reward = baseValue - (effectiveDistance / tolerance)*baseValue ;
        }
        else
        {
            // Au-delà de la tolérance, la récompense devient négative
            Reward = -((effectiveDistance - tolerance) / tolerance);
        }

        // Appliquer une pénalité si spécifié
        if (penality != 0)
        {
            Reward -= penality * (Reward > 0 ? 0.1f : 2.0f);  // Pénalité réduite si positive, doublée si négative
        }
        else
        {
            Reward = Math.Max(0, Reward);
        }

        //Debug.Log("Reward: " + Reward);

        return Reward;
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
        //float Reward = 0f;
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
        Transform currentFootInFront = isLeftFootInFront ? Foot_LEFT : isRightFootInFront ? Foot_RIGHT : null;
        if(footInFront != currentFootInFront && currentFootInFront != null)
        {
            timeSinceLastStep = currentFootInFront != null ? 0f: timeSinceLastStep;
            footInFront = currentFootInFront != null ?currentFootInFront : null;
            //Debug.Log(footInFront == Foot_LEFT ? "Pied gauche en avant":footInFront == Foot_RIGHT? "Pied droite en avant": "Aucuns pieds en avant");
        }
        else
        {
            timeSinceLastStep += Time.deltaTime;
            float minustime = 0.475f + 0.254f * (6-m_TargetWalkingSpeed);
            if(timeSinceLastStep > minustime)
            {
                float penalty = -10f * (timeSinceLastStep-minustime);
                //Reward(penalty);
                //Debug.Log(penalty+ " after "+(timeSinceLastStep - minustime) + " temps pour pas = "+ minustime);
            }
            //Debug.Log(footInFront == Foot_LEFT ? "Pied gauche en avant depuis "+ timeSinceLastStep + " secondes":footInFront == Foot_RIGHT? "Pied droite en avant depuis "+ timeSinceLastStep + " secondes": "Aucuns pieds en avant");
        }

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



    public float CenterOfGravityReward()
    {
        Vector3 currentCoG = CalculateCenterOfGravity(); // Assume que cela retourne le centre de gravité
        currentCoG = new Vector3(currentCoG.x, 0, currentCoG.z);
        Vector3 midpoint = (Foot_LEFT.position + Foot_RIGHT.position) / 2;
        midpoint = new Vector3(midpoint.x, 0, midpoint.z);
        float Reward = 0f;

        if (true)//(leftFootContact.LeftFootOnFloor && rightFootContact.RightFootOnFloor)
        {
            // // Les deux pieds sont au sol
            // float distanceToMidpoint = Vector3.Distance(currentCoG, midpoint);
            // if(Math.Abs(midpoint.z - currentCoG.z) < 0.2 )
            // {
            //     Reward += 1f;
            // }
            // else
            // {
            //     Reward -= 1f + distanceToMidpoint;
            // }
            // if(Math.Abs(midpoint.x - currentCoG.x) < 0.2 )
            // {
            //     Reward += 1f;
            // }
            // else
            // {
            //     Reward -= 1f + distanceToMidpoint;
            // }
            //Si penché en avant
            if(PerpendicularDistanceFromGoC >= 0)
            {
                //Debug.Log(2 -PerpendicularDistanceFromGoC);
                Reward += 1 -PerpendicularDistanceFromGoC;
            }
            else
            {
                //Debug.Log(PerpendicularDistanceFromGoC+2);
                Reward += PerpendicularDistanceFromGoC+1;
            }
            if(leftFootParallel && rightFootParallel)
            {
                Reward += 1f;
            }
            else
            {
                Reward -= 1f;
            }
        }
        else
        {
            Reward-= 0.1f;
        }
        return Reward;
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
        //             Reward += 2 -CustomPerpendicularDistanceFromGoC;
        //         }
        //         else
        //         {
        //             //Debug.Log(CustomPerpendicularDistanceFromGoC+2+ " distanceToCoGFromLeftFoot:" + (1-distanceToSupportingFoot) + " Angle:" + supportingFootAngle);
        //             Reward += CustomPerpendicularDistanceFromGoC+2;
        //         }
        //     }
        //     else
        //     {
        //         Reward -= 0.1f;
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
        //     Reward = -1f;
        // }

        // Appliquez la récompense ou la pénalité
        //return Reward;
    }
    void OnDrawGizmos()
    {
        //Dessice une sphere sur le sol qui indique le centre de gravité
        Gizmos.color = Color.red;
        Vector3 floorCoG = CalculateCenterOfGravity();
        floorCoG = new Vector3(floorCoG.x, 0f, floorCoG.z);
        Gizmos.DrawSphere(floorCoG, 0.05f); // Dessine une petite sphère au centre de gravité
        Vector3 worldAnchorLeft = new Vector3(Foot_LEFT.TransformPoint(ankleLeft.anchor).x,0f, Foot_LEFT.TransformPoint(ankleLeft.anchor).z+(AverageVelocity.z)/7f);
        Vector3 worldAnchorRight = new Vector3(Foot_RIGHT.TransformPoint(ankleRight.anchor).x,0f, Foot_RIGHT.TransformPoint(ankleRight.anchor).z+(AverageVelocity.z)/7f);

        Gizmos.color = Color.green;
        Gizmos.DrawLine(worldAnchorLeft, worldAnchorRight);
        Gizmos.color = leftFootStable? Color.green: Color.red;
        Gizmos.DrawSphere(worldAnchorLeft, 0.05f);
        Gizmos.color = rightFootStable? Color.green: Color.red;
        Gizmos.DrawSphere(worldAnchorRight, 0.05f);


        float circlePosition = supportFootCoG;
        //Debug.Log(supportFootCoG);
        Vector3 circleCenter = Vector3.Lerp(worldAnchorLeft, worldAnchorRight, (circlePosition + 1f) / 2f);
        distanceFromCoG = Vector3.Distance(circleCenter, floorCoG);
        isInCoGCircle = distanceFromCoG < 0.5f;
        DrawGizmoCircle(circleCenter, 0.5f, isInCoGCircle);
        //Debug.Log(DetermineSupportFootV2());

    }
    private void DetemineSupportFootV4()
    {
        const float maxCoGPosition = 0.3f;
        const float minCogPosition = -0.3f;
        Vector3 leftVelocity = m_JdController.bodyPartsDict[Foot_LEFT].rb.velocity;
        Vector3 rightVelocity = m_JdController.bodyPartsDict[Foot_RIGHT].rb.velocity;
        
        //Détermine si un pied est stable ou non
        leftFootStable = leftVelocity.magnitude < 0.4f && currentLeftFootHeight < 0.3f && footLeftOnGround && leftFootAngle < 10f;
        rightFootStable = rightVelocity.magnitude < 0.4f && currentRightFootHeight < 0.3f && footRightOnGround && rightFootAngle < 10f;

        // Positions des pieds en coordonnées locales par rapport au Bassin
        Vector3 leftFootPosition = Bassin.InverseTransformPoint(Foot_LEFT.position);
        Vector3 rightFootPosition = Bassin.InverseTransformPoint(Foot_RIGHT.position);

        // Utilisation de la composante z pour déterminer quel pied est en avant
        isLeftFootInFront = leftFootPosition.z - rightFootPosition.z > 0.005;
        isRightFootInFront = rightFootPosition.z - leftFootPosition.z > 0.005;
        
        if (true)
        {
            if (!leftFootStable && rightFootStable && lastAdvancingFoot != Foot_LEFT)
            {
                currentAdvancingFoot = Foot_LEFT;
                lastAdvancingFootHeight = currentLeftFootHeight;
                lastAdvancingFootVelocity = leftVelocity.magnitude;
                supportFootCoG = maxCoGPosition ;
                if(rightFootPosition.z - leftFootPosition.z < 0.001)
                {
                    supportFootCoG += (rightFootPosition.z - leftFootPosition.z)*250f;
                    supportFootCoG = Math.Max(supportFootCoG, minCogPosition);
                }
            }
            else if (!rightFootStable && leftFootStable && lastAdvancingFoot != Foot_RIGHT)
            {
                currentAdvancingFoot = Foot_RIGHT;
                lastAdvancingFootHeight = currentRightFootHeight;
                lastAdvancingFootVelocity = rightVelocity.magnitude;
                supportFootCoG = minCogPosition;
                if(rightFootPosition.z - leftFootPosition.z > 0.001)
                {
                    supportFootCoG += (rightFootPosition.z - leftFootPosition.z)*250f;
                    supportFootCoG = Math.Min(supportFootCoG, maxCoGPosition);
                }
            }
        }
    }
    private void DetemineSupportFootV3()
    {
        const float maxCoGPosition = 0.5f;
        const float minCogPosition = -0.5f;
        Vector3 leftVelocity = m_JdController.bodyPartsDict[Foot_LEFT].rb.velocity;
        Vector3 rightVelocity = m_JdController.bodyPartsDict[Foot_RIGHT].rb.velocity;
        
        //Détermine si un pied est stable ou non
        leftFootStable = leftVelocity.magnitude < 0.3f && currentLeftFootHeight < 0.2f && footLeftOnGround && leftFootAngle < 10f;
        rightFootStable = rightVelocity.magnitude < 0.3f && currentRightFootHeight < 0.2f && footRightOnGround && rightFootAngle < 10f;

        //Reset du pied d'avance si le pied stable ne l'est plus.
        if((currentAdvancingFoot == Foot_LEFT && !rightFootStable) || (currentAdvancingFoot == Foot_RIGHT && !leftFootStable))
        {
            currentAdvancingFoot = null;
            currentSupportFoot = null;
        }
        
        if (currentAdvancingFoot == null)
        {
            if (!leftFootStable && rightFootStable && lastAdvancingFoot != Foot_LEFT)
            {
                currentAdvancingFoot = Foot_LEFT;
                lastAdvancingFootHeight = currentLeftFootHeight;
                lastAdvancingFootVelocity = leftVelocity.magnitude;
                supportFootCoG = currentSupportFoot == null ? 0.0f: currentSupportFoot == Foot_LEFT? minCogPosition : maxCoGPosition;
                positionZFeetAtStart = Foot_LEFT.localPosition.z;
            }
            else if (leftFootStable && !rightFootStable && lastAdvancingFoot != Foot_RIGHT)
            {
                currentAdvancingFoot = Foot_RIGHT;
                lastAdvancingFootHeight = currentRightFootHeight;
                lastAdvancingFootVelocity = rightVelocity.magnitude;
                supportFootCoG = currentSupportFoot == null ? -0.0f: currentSupportFoot == Foot_LEFT? minCogPosition : maxCoGPosition;
                positionZFeetAtStart = Foot_RIGHT.localPosition.z;
            }
            else
            {
                supportFootCoG = currentSupportFoot == null ? 0f: currentSupportFoot == Foot_LEFT? minCogPosition : maxCoGPosition;
                lastAdvancingFootHeight = 0f;
                lastAdvancingFootVelocity = 0f;
            }

        }
        else if(currentAdvancingFoot == Foot_LEFT)
        {
            if(currentLeftFootHeight > lastAdvancingFootHeight)
            {
                topAdvancedFootHeight = Math.Max(topAdvancedFootHeight, currentLeftFootHeight);
                float heightDifference = currentLeftFootHeight - lastAdvancingFootHeight;
                //float normalizedChange = heightDifference ;/// topAdvancedFootHeight;
                float adjustmentFactor = 1f;
                supportFootCoG += heightDifference * adjustmentFactor;
                supportFootCoG = Mathf.Clamp(supportFootCoG, minCogPosition ,  maxCoGPosition);
            }
            else if(currentLeftFootHeight < lastAdvancingFootHeight)
            {

                float heightDifference = lastAdvancingFootHeight - currentLeftFootHeight;
                float normalizedChange = heightDifference / topAdvancedFootHeight;

                // Ajuster supportFootCoG en utilisant le changement normalisé, multiplié par un facteur d'ajustement pour contrôler la vitesse de changement
                float adjustmentFactor = 1.0f; // Ajustez cette valeur pour contrôler la vitesse de la transition
                supportFootCoG -= normalizedChange * adjustmentFactor;

                // Clamper supportFootCoG entre -1 et 1 pour s'assurer qu'il ne dépasse pas ces bornes
                supportFootCoG = Mathf.Clamp(supportFootCoG, minCogPosition ,  maxCoGPosition);
            }

            if(leftFootStable)
            {
                currentAdvancingFoot = null;
                currentSupportFoot = Foot_LEFT;
                lastAdvancingFootHeight = 0f;
                lastAdvancingFootVelocity = 0f;
                topAdvancedFootHeight = 0f;
                float positionZFeetAtEnd = Foot_LEFT.localPosition.z;
                float footTravel =positionZFeetAtStart - positionZFeetAtEnd;
                //Reward(footTravel*100000f);
            }
            else
            {
                lastAdvancingFootHeight = currentLeftFootHeight;
                lastAdvancingFootVelocity = leftVelocity.magnitude;
            }
        }
        else if(currentAdvancingFoot == Foot_RIGHT)
        {
            if(currentRightFootHeight > lastAdvancingFootHeight)
            {
                topAdvancedFootHeight = Math.Max(topAdvancedFootHeight, currentRightFootHeight);

                float heightDifference = lastAdvancingFootHeight - currentRightFootHeight;
                float normalizedChange = heightDifference / topAdvancedFootHeight;

                // Ajuster supportFootCoG en utilisant le changement normalisé, multiplié par un facteur d'ajustement pour contrôler la vitesse de changement
                float adjustmentFactor = 1f; // Ajustez cette valeur pour contrôler la vitesse de la transition
                supportFootCoG += normalizedChange * adjustmentFactor;

                // Clamper supportFootCoG entre -1 et 1 pour s'assurer qu'il ne dépasse pas ces bornes
                supportFootCoG = Mathf.Clamp(supportFootCoG, minCogPosition ,  maxCoGPosition);
            }
            else if(currentRightFootHeight < lastAdvancingFootHeight)
            {
                float heightDifference =  currentRightFootHeight -lastAdvancingFootHeight;
                float normalizedChange = heightDifference / topAdvancedFootHeight;

                // Ajuster supportFootCoG en utilisant le changement normalisé, multiplié par un facteur d'ajustement pour contrôler la vitesse de changement
                float adjustmentFactor = 1f; // Ajustez cette valeur pour contrôler la vitesse de la transition
                supportFootCoG -= normalizedChange * adjustmentFactor;

                // Clamper supportFootCoG entre -1 et 1 pour s'assurer qu'il ne dépasse pas ces bornes
                supportFootCoG = Mathf.Clamp(supportFootCoG, minCogPosition ,  maxCoGPosition);
            }
            

            if(rightFootStable)
            {
                currentAdvancingFoot = null;
                currentSupportFoot = Foot_RIGHT;
                lastAdvancingFootHeight = 0f;
                lastAdvancingFootVelocity = 0f;
                topAdvancedFootHeight = 0f;
                float positionZFeetAtEnd = Foot_RIGHT.localPosition.z;
                float footTravel =positionZFeetAtStart - positionZFeetAtEnd;
                //Reward(footTravel*100000f);
            }
            else
            {
                lastAdvancingFootHeight = currentRightFootHeight;
                lastAdvancingFootVelocity = rightVelocity.magnitude;
            }
        }
    }
    // public float DetermineSupportFootV2()
    // {
    //     Vector3 leftVelocity = m_JdController.bodyPartsDict[Foot_LEFT].rb.velocity;
    //     Vector3 rightVelocity = m_JdController.bodyPartsDict[Foot_RIGHT].rb.velocity;
    //     float leftHeight = Foot_LEFT.position.y;  // Assumer que la hauteur est mesurée sur l'axe y
    //     float rightHeight = Foot_RIGHT.position.y;

    //     // Déterminer le pied en avancement basé sur la vélocité et la hauteur
    //     if (currentAdvancingFoot == null)
    //     {
    //         if (leftVelocity.magnitude > rightVelocity.magnitude || leftHeight > rightHeight)
    //         {
    //             currentAdvancingFoot = Foot_LEFT;
    //             lastSupportFoot = Foot_RIGHT;
    //         }
    //         else
    //         {
    //             currentAdvancingFoot = Foot_RIGHT;
    //             lastSupportFoot = Foot_LEFT;
    //         }
    //     }

    //     // Calculer la valeur de support en fonction du pied en avancement
    //     if (currentAdvancingFoot == Foot_LEFT)
    //     {
    //         // Plus le pied gauche avance, plus cette valeur se rapproche de 1
    //         supportValue -= Mathf.Clamp01(leftVelocity.magnitude / maxVelocity);
    //     }
    //     else if (currentAdvancingFoot == Foot_RIGHT)
    //     {
    //         // Plus le pied droit avance, plus cette valeur se rapproche de -1
    //         supportValue += Mathf.Clamp01(rightVelocity.magnitude / maxVelocity);
    //     }

    //     // Normaliser la valeur de support pour qu'elle ne dépasse jamais les limites de -1 et 1
    //     supportValue = Mathf.Clamp(supportValue, -1, 1);

    //     // Vérifier si le pied en avancement doit devenir le pied de support
    //     if (Mathf.Abs(currentAdvancingFoot.position.y - lastSupportFoot.position.y) < 0.1f &&
    //         currentAdvancingFoot.GetComponent<Rigidbody>().velocity.magnitude < 0.1f)
    //     {
    //         lastSupportFoot = currentAdvancingFoot;
    //         currentAdvancingFoot = null;  // Réinitialiser le pied en avancement
    //         supportValue = (lastSupportFoot == Foot_LEFT) ? 1 : -1;  // Mettre à jour la valeur de support
    //     }
    //     supportFootCoG = supportValue;
    //     return supportValue;
    // }

    private float NormalizeHeight(float footHeight)
    {
        return Mathf.Clamp01(footHeight / maxVelocity);
    }


   
    

    public float DetermineSupportFoot()
    {
        // Calculer les distances du bassin aux pieds sur les axes X et Z
        float distanceToLeftFoot = Vector2.Distance(
            new Vector2(Bassin.position.x, Bassin.position.z), 
            new Vector2(Foot_LEFT.position.x, Foot_LEFT.position.z));

        float distanceToRightFoot = Vector2.Distance(
            new Vector2(Bassin.position.x, Bassin.position.z), 
            new Vector2(Foot_RIGHT.position.x, Foot_RIGHT.position.z));

        // Calculer la différence des distances normalisée
        if (distanceToLeftFoot == distanceToRightFoot)
        {
            return 0;  // Les pieds sont à équidistance
        }
        else
        {
            // Retourne une valeur proportionnelle où -1 et 1 sont les extrêmes
            float normalizedDifference = (distanceToRightFoot - distanceToLeftFoot) / (distanceToLeftFoot + distanceToRightFoot);
            float sensitivity = 2.5f;
            normalizedDifference *= sensitivity;
            // Clamper la valeur entre -1 et 1 pour éviter des extrêmes hors de cet intervalle
            normalizedDifference = Mathf.Clamp(normalizedDifference, -1, 1);
            return -normalizedDifference; // Inverser pour que -1 soit pied gauche et 1 pied droit
        }
    }
    void DrawGizmoCircle(Vector3 position, float radius, bool inCircle)
    {
        const int segmentCount = 20;  // Nombre de segments pour dessiner le cercle
        Vector3 prevPos = position + new Vector3(radius, 0, 0);

        for (int i = 1; i <= segmentCount; i++)
        {
            float angle = i * Mathf.PI * 2f / segmentCount;
            Vector3 newPos = position + new Vector3(Mathf.Cos(angle) * radius, 0, Mathf.Sin(angle) * radius);
            Gizmos.color = inCircle? Color.green: Color.red;
            Gizmos.DrawLine(prevPos, newPos);
            prevPos = newPos;
        }
    }

    Vector3[] GetFootBounds(Transform  footLEFT, Transform  footRIGHT = null)
    {
        Renderer rendererL = footLEFT.GetComponent<Renderer>();
        Vector3 centerL = rendererL.bounds.center;
        Vector3 extentsL = rendererL.bounds.extents;
        Quaternion rotationL = footLEFT.transform.rotation;  // Rotation du pied gauche

        Vector3[] corners = new Vector3[4];

        if (footRIGHT != null)
        {
            Renderer rendererR = footRIGHT.GetComponent<Renderer>();
            Vector3 centerR = rendererR.bounds.center;
            Vector3 extentsR = rendererR.bounds.extents;
            Quaternion rotationR = footRIGHT.transform.rotation;  // Rotation du pied droit

            // Applique la rotation du pied à chaque coin
            corners[0] = centerR + rotationR * new Vector3(extentsR.x, 0, extentsR.z);  // Front Right
            corners[1] = centerL + rotationL * new Vector3(-extentsL.x, 0, extentsL.z); // Front Left
            corners[2] = centerL + rotationL * new Vector3(-extentsL.x, 0, -extentsL.z); // Back Left
            corners[3] = centerR + rotationR * new Vector3(extentsR.x, 0, -extentsR.z); // Back Right
        }
        else
        {
            float decalageLeft = footRightOnGround ? -.5f :0;
            float decalageRight = footRightOnGround ? 0:.5f;
            float distanceAutrePied = footLeftOnGround ? Foot_RIGHT.localPosition.z - Foot_LEFT.localPosition.z: Foot_LEFT.localPosition.z - Foot_RIGHT.localPosition.z;
            float distanceOnLeftSide = footLeftOnGround?  0f:Foot_LEFT.localPosition.z - Foot_RIGHT.localPosition.z;
            float diatanceOnRightSide = footLeftOnGround? Foot_RIGHT.localPosition.z - Foot_LEFT.localPosition.z : 0f;
            // S'il n'y a qu'un pied, appliquez simplement la rotation de ce pied à tous les coins
            corners[0] = centerL + rotationL * new Vector3(extentsL.x+decalageRight, 0, extentsL.z+diatanceOnRightSide);// Front Right
            corners[1] = centerL + rotationL * new Vector3(-extentsL.x+decalageLeft, 0, extentsL.z+distanceOnLeftSide);// Front Left
            corners[2] = centerL + rotationL * new Vector3(-extentsL.x+decalageLeft, 0, -extentsL.z+distanceOnLeftSide/2);// Back Left
            corners[3] = centerL + rotationL * new Vector3(extentsL.x+decalageRight, 0, -extentsL.z+diatanceOnRightSide/2);// Back Right
        }

        return corners;
    }


    void DrawPolygon(Vector3[] corners)
    {
        for (int i = 0; i < corners.Length; i++)
        {
            Gizmos.DrawLine(corners[i], corners[(i + 1) % corners.Length]);
        }
    }
    bool IsPointInPolygon(int n, Vector3[] polygon, Vector3 p)
    {
        int counter = 0;
        int i;
        double xinters;
        Vector3 p1, p2;

        p1 = polygon[0];
        for (i = 1; i <= n; i++)
        {
            p2 = polygon[i % n];
            if (p.z > Mathf.Min(p1.z, p2.z))
            {
                if (p.z <= Mathf.Max(p1.z, p2.z))
                {
                    if (p.x <= Mathf.Max(p1.x, p2.x))
                    {
                        if (p1.z != p2.z)
                        {
                            xinters = (p.z - p1.z) * (p2.x - p1.x) / (p2.z - p1.z) + p1.x;
                            if (p1.x == p2.x || p.x <= xinters)
                                counter++;
                        }
                    }
                }
            }
            p1 = p2;
        }

        return counter % 2 != 0;
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
        if (angleWithVertical > 65 && angleWithVertical < 140 && currentProportionalHeadHeight > 0.8 && distanceFromCoG < 1f)
        {
            // L'agent est debout ou à l'envers
            faceOnTheFloor = 0;
            textPosition.text = "Debout\nCoG: " + $"{distanceFromCoG:0.00}"+"\nAngle: "+$"{angleWithVertical:0.0}"+"\nHead : "+$"{currentProportionalHeadHeight:0.00}";
            return 0;
        }
        else
        {
            float dotProductY = Vector3.Dot((
                Cuisse_LEFT.forward + Cuisse_RIGHT.forward + 
                Bassin.forward + Torse.forward)/4, Vector3.down);

            if (dotProductY > 0.2f ) // Face au sol
            {
                faceOnTheFloor = 1;
                textPosition.text = "Face au sol\nCoG: " + $"{distanceFromCoG:0.00}";
                return 1;
            }
            else if (dotProductY < 0.2f ) // Sur le dos
            {
                faceOnTheFloor = -1;
                textPosition.text = "Sur le dos\nCoG: " + $"{distanceFromCoG:0.00}";
                return -1;
            }
            else
            {
                string pos = faceOnTheFloor == 1? "Face au sol U": "Sur le dos U";
                textPosition.text = $"{pos}\nCoG: " + $"{distanceFromCoG:0.00}"+"\n";
                return faceOnTheFloor;
            }
        }
    }


     //Update OrientationCube and DirectionIndicator
    public void UpdateOrientationObjects()
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
        if(enTestUnit)
        {
            Reward(100f);
            EndEpisode();
        }
        else
        {
            Reward(1f, "Cube touched",StatAggregationMethod.Sum);
        }
        EndEpisode();
    }

    public void AddLog(string def, float value, StatAggregationMethod type=StatAggregationMethod.Average)
    {
        Academy.Instance.StatsRecorder.Add(def, value, type);
    }
    void Update()
    {
        RaycastHit hit;
        int groundLayerMask = 1 << LayerMask.NameToLayer("Devetik_Floor");  // Assurez-vous que ce layer est correctement configuré

        // Envoyer un rayon depuis chaque pied vers le bas
        if (Physics.Raycast(Head.position, Vector3.down, out hit, 6f, groundLayerMask))
        {
            currentProportionalHeadHeight = System.Math.Min((hit.distance - 0.64f) / (normalHeadHeight - 0.64f), 1f);
            currentHeadHeight = hit.distance;
        }
        if (Physics.Raycast(Bassin.position, Vector3.down, out hit, 6f, groundLayerMask))
        {
            currentProportionalBassinHeight = System.Math.Min((hit.distance-0.26f) / (normalBassinHeight-0.26f), 1);
            currentBassinHeight = hit.distance;
        }
        if (Physics.Raycast(Foot_LEFT.position, Vector3.down, out hit, 6f, groundLayerMask))
        {
            currentLeftFootHeight = hit.distance-0.061856f;
        }
        if (Physics.Raycast(Foot_RIGHT.position, Vector3.down, out hit, 6f, groundLayerMask))
        {
            currentRightFootHeight = hit.distance-0.061856f;
        }
        AddLog("Bassin",currentProportionalBassinHeight);
        AddLog("Head", currentProportionalHeadHeight);
        DetemineSupportFootV3();
        OrientationTorseFloor();
        AverageVelocity = GetAvgVelocity();
        distanceFootLeftToHandRight = (Bassin.InverseTransformPoint(Foot_LEFT.position).z - Bassin.InverseTransformPoint(Hand_RIGHT.position).z)*100f;
        distanceFootRightToHandLeft = (Bassin.InverseTransformPoint(Foot_RIGHT.position).z - Bassin.InverseTransformPoint(Hand_LEFT.position).z)*100f;
        AddLog("Hand/Feet Sync", distanceFootLeftToHandRight+distanceFootRightToHandLeft);
    }

    private float GetArmFeetSyncReward()
    {
        distanceFootLeftToHandRight = (Bassin.InverseTransformPoint(Foot_LEFT.position).z - Bassin.InverseTransformPoint(Hand_RIGHT.position).z)*100f;
        distanceFootRightToHandLeft = (Bassin.InverseTransformPoint(Foot_RIGHT.position).z - Bassin.InverseTransformPoint(Hand_LEFT.position).z)*100f;
        float proportionalDistanceFootLeftToHandRight = distanceFootLeftToHandRight * Math.Abs(Bassin.InverseTransformPoint(Foot_LEFT.position).z);
        float proportionalDistanceFootRightToHandLeft = distanceFootRightToHandLeft * Math.Abs(Bassin.InverseTransformPoint(Foot_RIGHT.position).z);
        return ((2f-Math.Abs(proportionalDistanceFootLeftToHandRight))/2 + (2f-Math.Abs(proportionalDistanceFootRightToHandLeft))/2)/2;
        //return ((2f-Math.Abs(distanceFootLeftToHandRight))/2 + (2f-Math.Abs(distanceFootRightToHandLeft))/2)/2;
    }
}

