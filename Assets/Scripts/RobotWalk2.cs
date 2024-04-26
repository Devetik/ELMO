using System.Collections;
using System.Collections.Generic;
using UnityEngine;
//using System;
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



public class RobotWalk : Agent
{
    public StatsRecorder wakeUp;
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
    public Transform Head;
    public Transform Foot_LEFT;
    public Transform Foot_RIGHT;
    public Transform Tibias_LEFT;
    public Transform Tibias_RIGHT;
    public Transform Cuisse_LEFT;
    public Transform Cuisse_RIGHT;
    public Transform Bassin;
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
    // public Transform berceauAvant;
    // public Transform berceauArriere;
    public ConfigurableJoint Head_JOINT;
    public ConfigurableJoint Foot_LEFT_JOINT;
    public ConfigurableJoint Foot_RIGHT_JOINT;
    public ConfigurableJoint Tibias_LEFT_JOINT;
    public ConfigurableJoint Tibias_RIGHT_JOINT;
    public ConfigurableJoint Cuisse_LEFT_JOINT;
    public ConfigurableJoint Cuisse_RIGHT_JOINT;
    public ConfigurableJoint Bassin_JOINT;
    public ConfigurableJoint Abdos_JOINT;
    public ConfigurableJoint Torse_JOINT;
    public ConfigurableJoint Shoulder_LEFT_JOINT;
    public ConfigurableJoint Shoulder_RIGHT_JOINT;
    public ConfigurableJoint Coude_LEFT_JOINT;
    public ConfigurableJoint Coude_RIGHT_JOINT;
    public float TempsSession = 0f;
    private float initialReward = 1.0f;
    public float decayRate;
    private Vector3 lastPosition;
    private float distanceToTargetAtStart;
    public float normalHeadHeight = 4.10f;
    public float normalTorseHeight = 3.25f;
    public FootContact leftFootContact;
    public FootContact rightFootContact;
    public FootContact leftTibiasContact;
    public FootContact rightTibiasContact;
    public FootContact leftHandContact;
    public FootContact rightHabdContact;
    public FootContact headContact;
    public FootContact torseContact;
    public FootContact abdoContact;
    public FootContact bassinContact;
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
    private const float MaxFatigue = 100f;
    private const float MinFatigue = -100f;
    private const float FatigueIncrement = 200f;
    private const float RecoveryRate = 1f;
    private const float MaxPower = 200f;
    private float Fatigue_Head = 0.0f;
    // private float Fatigue_Foot_LEFT = 0.0f;
    // private float Fatigue_Foot_RIGHT = 0.0f;
    private float Fatigue_Tibias_LEFT = 0.0f;
    private float Fatigue_Tibias_RIGHT = 0.0f;
    private float Fatigue_Cuisse_LEFT = 0.0f;
    private float Fatigue_Cuisse_RIGHT = 0.0f;
    private float Fatigue_Bassin = 0.0f;
    private float Fatigue_Abdos = 0.0f;
    private float Fatigue_Torse = 0.0f;
    private float Fatigue_Foot_LEFT = 0.0f;
    private float Fatigue_Foot_RIGHT = 0.0f;
    private float Fatigue_Shoulder_LEFT = 0.0f;
    private float Fatigue_Shoulder_RIGHT = 0.0f;
    private float Fatigue_Coude_RIGHT = 0.0f;
    private float Fatigue_Coude_LEFT = 0.0f;
    private float timeSinceLastStepLeft = 0f;
    private float timeSinceLastStepRight = 0f;
    private bool isLeftFootInFront = false;
    private bool stepInProgressRight = false;
    private bool stepInProgressLeft = false;
    private bool lastStepInProgress = false;
    private bool firstStep = true;
    private float maxStepDuration = 3f;  // Durée maximale pour un pied devant l'autre
    private float minStepDuration = 0.25f;
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
    private bool hasLanded = false;
    public int faceOnTheFloor;
    private int curriculumStep;
    public Vector3 startPosition;
    public Quaternion startRotation;


    public override void Initialize()
    {
        wakeUp = Academy.Instance.StatsRecorder;

        stepReward = new StepReward(this);
        vc = new VariableCustom(this);
        bodyPartManager = Agent.GetComponent<BodyPartManager>(); // Assurez-vous que le composant est attaché à `Agent`
        //envPartManager = EnvToReset.GetComponent<BodyPartManager>(); // Assurez-vous que le composant est attaché à `EnvToReset`

        if (bodyPartManager != null)
            bodyPartManager.Initialize(Agent); // Initialisez avec le Transform approprié

        if (envPartManager != null)
        Target.gameObject.SetActive(true);
        distanceToTargetAtStart = Vector3.Distance((Foot_LEFT.position + Foot_RIGHT.position)/2, Target.position);
        lastPosition = Torse.position;
        lastRotation = Torse.rotation;
        startPosition = new Vector3(0,0.5f,5);
        startRotation = new Quaternion(0f,0f,0f,0f);
        //Pressure_Plate.localPosition = new Vector3(UnityEngine.Random.Range(-25,25),-1f,UnityEngine.Random.Range(-14,-64));
    }
    public override void OnEpisodeBegin()
    {
        hasLanded = false;
        enchainementStep = 1;
        Fatigue_Head = 100.0f;
        Fatigue_Tibias_LEFT = 100.0f;
        Fatigue_Tibias_RIGHT = 100.0f;
        Fatigue_Cuisse_LEFT = 100.0f;
        Fatigue_Cuisse_RIGHT = 100.0f;
        Fatigue_Bassin = 100.0f;
        Fatigue_Abdos = 100.0f;
        Fatigue_Torse = 100.0f;
        Fatigue_Shoulder_LEFT = 100.0f;
        Fatigue_Shoulder_RIGHT = 100.0f;
        Fatigue_Coude_RIGHT = 100.0f;
        Fatigue_Coude_LEFT = 100.0f;
        Fatigue_Foot_RIGHT = 100.0f;
        Fatigue_Foot_LEFT = 100.0f;
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


    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        base.OnActionReceived(actionBuffers);
        if(!hasLanded)
        {
            hasLanded = CheckIfLanded();
        }
        else
        {
            try
            {
                curriculumStep = int.Parse(System.IO.File.ReadAllText(filePath));
            }
            catch
            {
                Console.WriteLine("An error occurred while reading the file:");
            }

            AddLog("Average Head Height", Head.position.y);

            switch (curriculumStep)
            {
                case -1:
                    stepReward.Custom();
                    break;
                case 0:
                    stepReward.Stability(); // -> 1000 pts
                    break;
                case 1:
                    stepReward.Walk();
                    break;
                case 2:
                    stepReward.WakeUp();
                    break;
                case 3:
                    stepReward.RiseFromBack();
                    break;
                case 4:
                    stepReward.Stability();
                    break;
                case 5:
                    stepReward.Walk();
                    break;
            }

            TempsSession += Time.fixedDeltaTime;
            decayRate = 1f - ((float)StepCount / (float)MaxStep);

            AdjustJointRotation(Tibias_LEFT_JOINT,ref Fatigue_Tibias_LEFT, actionBuffers.ContinuousActions[0]);
            AdjustJointRotation(Tibias_RIGHT_JOINT,ref Fatigue_Tibias_RIGHT, actionBuffers.ContinuousActions[1]);
            AdjustJointRotation(Cuisse_LEFT_JOINT, ref Fatigue_Cuisse_LEFT, actionBuffers.ContinuousActions[2],actionBuffers.ContinuousActions[3],actionBuffers.ContinuousActions[4]);
            AdjustJointRotation(Cuisse_RIGHT_JOINT,ref Fatigue_Cuisse_RIGHT, actionBuffers.ContinuousActions[5],actionBuffers.ContinuousActions[6],actionBuffers.ContinuousActions[7]);
            AdjustJointRotation(Bassin_JOINT, ref Fatigue_Bassin, actionBuffers.ContinuousActions[8],actionBuffers.ContinuousActions[9]);
            AdjustJointRotation(Abdos_JOINT,ref Fatigue_Abdos, actionBuffers.ContinuousActions[10]);
            AdjustJointRotation(Head_JOINT,ref Fatigue_Head, actionBuffers.ContinuousActions[11],actionBuffers.ContinuousActions[12]);
            AdjustJointRotation(Shoulder_LEFT_JOINT, ref Fatigue_Shoulder_LEFT, actionBuffers.ContinuousActions[13],actionBuffers.ContinuousActions[14],zValue:actionBuffers.ContinuousActions[15], xSpring:360, xForce:360);
            AdjustJointRotation(Shoulder_RIGHT_JOINT, ref Fatigue_Shoulder_RIGHT, actionBuffers.ContinuousActions[16],actionBuffers.ContinuousActions[17],actionBuffers.ContinuousActions[18], xSpring:360, xForce:360);
            AdjustJointRotation(Coude_LEFT_JOINT,ref Fatigue_Coude_LEFT, actionBuffers.ContinuousActions[19]);
            AdjustJointRotation(Coude_RIGHT_JOINT,ref Fatigue_Coude_RIGHT, actionBuffers.ContinuousActions[20]);
            AdjustJointRotation(Torse_JOINT,ref Fatigue_Torse, actionBuffers.ContinuousActions[21]);
            AdjustJointRotation(Foot_LEFT_JOINT,ref Fatigue_Foot_LEFT, actionBuffers.ContinuousActions[22]);
            AdjustJointRotation(Foot_RIGHT_JOINT,ref Fatigue_Foot_RIGHT, actionBuffers.ContinuousActions[23]);
            

            UpdateFootPosition();
            //ApplyStepReward();

            float distanceToTargetRestante = Vector3.Distance(((Foot_LEFT.position + Foot_RIGHT.position)/2 + Head.position)/2, Target.position);
            float distanceParcourue = distanceToTargetAtStart - distanceToTargetRestante ;
            float proportionTravel = distanceParcourue / distanceToTargetAtStart ;

            float bonus = 0f;

            //bonus += FlipTrain();
            //Debug.Log("CenterOfGravityReward " + CenterOfGravityReward());
            //Debug.Log(bonus+ " /  " + simpleHeadOnTop(penality:0, baseValue:1, timeExpo:false) + " / " + CenterOfGravityReward());
            float fatiguePenalty =  ( Fatigue_Head + Fatigue_Abdos + Fatigue_Bassin
            + Fatigue_Shoulder_LEFT + Fatigue_Shoulder_RIGHT
            + Fatigue_Coude_LEFT + Fatigue_Coude_RIGHT
            + Fatigue_Cuisse_LEFT + Fatigue_Cuisse_RIGHT
            + Fatigue_Tibias_LEFT + Fatigue_Tibias_RIGHT
            + Fatigue_Foot_LEFT + Fatigue_Foot_RIGHT) /1300;
            //reward(fatiguePenalty);
            //Debug.Log(fatiguePenalty);
            //bonus+= fatiguePenalty;
            //bonus = 1-Vector3.Dot(Torse.forward, Vector3.up);
            //bonus += proportionTravel;
            //Debug.Log(proportionTravel*10);
            //bonus+= isWalkingForward();

            //bonus+= headOnTop(penality:0, baseValue:1, timeExpo:false);
            //Debug.Log("headOnTop:" + headOnTop(penality:0, baseValue:1, timeExpo:false) + " -    " + Vector3.Angle(Cuisse_RIGHT.up, Vector3.up) );
            //bonus+= verticalStability(angle:45f, exposant:1f);
            //Debug.Log("verticalStability:" + verticalStability(45f, 1f));

            //bonus+= DistanceToTarget();
            // //Debug.Log("DistanceToTarget:"+DistanceToTarget());
            //Debug.Log("Bonus: "+ bonus);
            //reward(bonus);
            //Debug.Log(reward + " headHeightProportion:"+headHeightProportion + " proportionTravel:"+proportionTravel);
            //colorChanger.UpdateColorBasedOnReward(reward);

            RecoverFatigue(ref Fatigue_Head);
            RecoverFatigue(ref Fatigue_Tibias_LEFT);
            RecoverFatigue(ref Fatigue_Tibias_RIGHT);
            RecoverFatigue(ref Fatigue_Cuisse_LEFT);
            RecoverFatigue(ref Fatigue_Cuisse_RIGHT);
            RecoverFatigue(ref Fatigue_Bassin);
            RecoverFatigue(ref Fatigue_Abdos);
            RecoverFatigue(ref Fatigue_Shoulder_LEFT);
            RecoverFatigue(ref Fatigue_Shoulder_RIGHT);
            RecoverFatigue(ref Fatigue_Coude_RIGHT);
            RecoverFatigue(ref Fatigue_Coude_RIGHT);
        }
    }

    void IncreaseFatigue(ref float fatigue, float increment)
    {
        fatigue -= increment;
        fatigue = Mathf.Min(Math.Max(fatigue, MinFatigue), MaxFatigue);  // Assurez-vous que la fatigue ne dépasse pas le maximum.
    }

    void RecoverFatigue(ref float fatigue)
    {
        fatigue += RecoveryRate * Time.deltaTime;
        fatigue = Mathf.Max(fatigue, MaxFatigue);
    }

    float CalculateFatigueEffect(float fatigue, float maxFatigue)
    {
        return 1.0f - (fatigue / maxFatigue);  // Réduction linéaire de la capacité basée sur la fatigue.
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

    // Cette fonction ajuste directement la rotation de n'importe quel ConfigurableJoint et configure les drives pour X et YZ.
    public void AdjustJointRotation(
        ConfigurableJoint joint,
        ref float fatigue, 
        float xValue, 
        float? yValue = null, 
        float? zValue = null,
        float xSpring = 100, 
        float xForce = 100, 
        float yzSpring = 75, 
        float yzForce = 75)
    {
        float actionIntensity = CalculateActionIntensity(xValue, yValue == null ? 0:yValue.Value, zValue == null ? 0:zValue.Value);
        IncreaseFatigue(ref fatigue, actionIntensity * FatigueIncrement);
        if (joint == null)
        {
            Debug.LogError("ConfigurableJoint is null.");
            return;
        }

        // Appliquer les rotations cibles
        Vector3 targetRotation = new Vector3(
            ConvertInputToRotation(xValue, xValue < 0 ? -joint.highAngularXLimit.limit:joint.lowAngularXLimit.limit),
            yValue.HasValue ? ConvertInputToRotation(yValue.Value, joint.angularYLimit.limit) : 0,
            zValue.HasValue ? ConvertInputToRotation(zValue.Value, joint.angularZLimit.limit) : 0
        );
        
        joint.targetRotation = Quaternion.Euler(targetRotation);
        //joint.targetAngularVelocity = targetRotation*4000f;

        // Configurer les drives pour les axes X
        JointDrive xDrive = joint.angularXDrive;
        xDrive.positionSpring = xSpring;
        xDrive.maximumForce = xForce;
        joint.angularXDrive = xDrive;

        // Configurer les drives pour les axes Y et Z
        JointDrive yzDrive = joint.angularYZDrive;
        yzDrive.positionSpring = yzSpring;
        yzDrive.maximumForce = yzForce;
        joint.angularYZDrive = yzDrive;
    }

    // Convertit une valeur d'entrée de -1 à 1 en une valeur de rotation en degrés, basée sur la limite maximale spécifiée.
    private float ConvertInputToRotation(float inputValue, float limit)
    {
        return inputValue * limit;
    }


    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        if (Input.GetKey(KeyCode.UpArrow))
        {
            // #Epaule
            // continuousActionsOut[13] = 1.0f;
            continuousActionsOut[16] = 1.0f;
            continuousActionsOut[13] = 1.0f;
            continuousActionsOut[2] = 1.0f;
            continuousActionsOut[5] = 1.0f;
            // continuousActionsOut[0] = 1.0f;
            // continuousActionsOut[1] = 1.0f;
            // continuousActionsOut[0] = 1.0f; //Cuisse G Baissée
            // continuousActionsOut[2] = 1.0f; // Genoux G Délié
            // continuousActionsOut[1] = -1.0f; //Cuisse D Levée
            // continuousActionsOut[5] = -1.0f; // Genoux D Plié
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            // #Epaule
            // continuousActionsOut[13] = -1.0f;
            continuousActionsOut[16] = -1.0f;
            continuousActionsOut[13] = -1.0f;
            continuousActionsOut[2] = -1.0f;
            continuousActionsOut[5] = -1.0f;
            // continuousActionsOut[0] = -1.0f;
            // continuousActionsOut[1] = -1.0f;
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
            // continuousActionsOut[0] = 1.0f;
            // continuousActionsOut[1] = 1.0f;
            // continuousActionsOut[19] = 1.0f;
            // continuousActionsOut[20] = 1.0f;
            continuousActionsOut[17] = 1.0f;

        }
        else if (Input.GetKey(KeyCode.LeftArrow))
        {
            // #Epaule
            // continuousActionsOut[0] = -1.0f;
            // continuousActionsOut[1] = -1.0f;
            // continuousActionsOut[19] = -1.0f;
            // continuousActionsOut[20] = -1.0f;
            continuousActionsOut[17] = -1.0f;
        }


        if (Input.GetKey(KeyCode.C))
        {
            // continuousActionsOut[19] = 1.0f;
            continuousActionsOut[18] = 1.0f;
        }
        else if (Input.GetKey(KeyCode.V))
        {
            // continuousActionsOut[19] = -1.0f;
            continuousActionsOut[18] = -1.0f;
        }
        // if (Input.GetKey(KeyCode.Y))
        // {
        //     continuousActionsOut[15] = 1.0f;
        //     continuousActionsOut[18] = 1.0f;
        // }
        // else if (Input.GetKey(KeyCode.X))
        // {
        //     continuousActionsOut[15] = -1.0f;
        //     continuousActionsOut[18] = -1.0f;
        // }


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
        base.CollectObservations(sensor);
        // sensor.AddObservation(this.transform.localPosition/10f);
        // sensor.AddObservation(this.transform.rotation.eulerAngles/360);
        //Debug.Log(this.transform.localPosition/10f + " " + this.transform.rotation.eulerAngles/360);
        sensor.AddObservation(OrientationTorseFloor());

        sensor.AddObservation(MeasureGroundDistance(transform, 4f));
        sensor.AddObservation(MeasureGroundDistance(Torse, normalTorseHeight));
        sensor.AddObservation(MeasureGroundDistance(Foot_LEFT, 2f));
        sensor.AddObservation(MeasureGroundDistance(Foot_RIGHT, 2f));
        sensor.AddObservation(currentProportionalHeadHeight);

        sensor.AddObservation(CalculateCenterOfGravity());

        // Positions relatives et vitesses des parties du corps importantes
        AddBodyPartObservation(Head, sensor);
        AddBodyPartObservation(Foot_LEFT, sensor);
        AddBodyPartObservation(Foot_RIGHT, sensor);
        AddBodyPartObservation(Tibias_LEFT, sensor);
        AddBodyPartObservation(Tibias_RIGHT, sensor);
        AddBodyPartObservation(Cuisse_LEFT, sensor);
        AddBodyPartObservation(Cuisse_RIGHT, sensor);
        AddBodyPartObservation(Bassin, sensor);
        AddBodyPartObservation(Abdos, sensor);
        AddBodyPartObservation(Arm_LEFT, sensor);
        AddBodyPartObservation(Arm_RIGHT, sensor);
        AddBodyPartObservation(Hand_LEFT, sensor);
        AddBodyPartObservation(Hand_RIGHT, sensor);
        AddBodyPartObservation(Coude_LEFT, sensor);
        AddBodyPartObservation(Coude_RIGHT, sensor);
        AddBodyPartObservation(Shoulder_LEFT, sensor);
        AddBodyPartObservation(Shoulder_RIGHT, sensor);

        // Contact au sol
        sensor.AddObservation(leftFootContact.LeftFootOnFloor ? 1.0f : 0.0f);
        sensor.AddObservation(rightFootContact.RightFootOnFloor ? 1.0f : 0.0f);
        sensor.AddObservation(isLeftFootInFront ? 1.0f : 0.0f);
        sensor.AddObservation(leftFootParallel ? 1.0f : 0.0f);
        sensor.AddObservation(rightFootParallel ? 1.0f : 0.0f);
        sensor.AddObservation(leftTibiasContact.LeftTibiasOnFloor ? 1.0f : 0.0f);
        sensor.AddObservation(rightTibiasContact.RightTibiasOnFloor ? 1.0f : 0.0f);
        sensor.AddObservation(leftHandContact.LeftHandOnFloor ? 1.0f : 0.0f);
        sensor.AddObservation(rightHabdContact.RightHandOnFloor ? 1.0f : 0.0f);
        sensor.AddObservation(headContact.HeadOnFloor ? 1.0f : 0.0f);
        sensor.AddObservation(torseContact.TorseOnFloor ? 1.0f : 0.0f);
        sensor.AddObservation(abdoContact.AbdoOnFloor ? 1.0f : 0.0f);
        sensor.AddObservation(bassinContact.BassinOnFloor ? 1.0f : 0.0f);

        // Distance et direction vers la cible
        Vector3 toTarget = (Target.position - ((Foot_LEFT.position + Foot_RIGHT.position)/2));
        sensor.AddObservation(toTarget.normalized);
        sensor.AddObservation(toTarget.magnitude / 100f);

        Vector3 toPlate = (Pressure_Plate.position - ((Foot_LEFT.position + Foot_RIGHT.position)/2));
        sensor.AddObservation(toPlate.normalized);
        sensor.AddObservation(toPlate.magnitude / 100f);

        // Temps écoulé depuis le début de l'épisode
        sensor.AddObservation(TempsSession / 1000f);

        sensor.AddObservation(Fatigue_Head / MaxFatigue);
        sensor.AddObservation(Fatigue_Tibias_LEFT / MaxFatigue);
        sensor.AddObservation(Fatigue_Tibias_RIGHT / MaxFatigue);
        sensor.AddObservation(Fatigue_Cuisse_LEFT / MaxFatigue);
        sensor.AddObservation(Fatigue_Cuisse_RIGHT / MaxFatigue);
        sensor.AddObservation(Fatigue_Bassin / MaxFatigue);
        sensor.AddObservation(Fatigue_Abdos / MaxFatigue);
        sensor.AddObservation(Fatigue_Torse / MaxFatigue);
        sensor.AddObservation(Fatigue_Shoulder_LEFT / MaxFatigue);
        sensor.AddObservation(Fatigue_Shoulder_RIGHT / MaxFatigue);
        sensor.AddObservation(Fatigue_Coude_LEFT / MaxFatigue);
        sensor.AddObservation(Fatigue_Coude_RIGHT / MaxFatigue);
        sensor.AddObservation(Fatigue_Foot_LEFT / MaxFatigue);
        sensor.AddObservation(Fatigue_Foot_RIGHT / MaxFatigue);

        sensor.AddObservation(Tibias_LEFT_JOINT.targetRotation.x/360);
        sensor.AddObservation(Tibias_RIGHT_JOINT.targetRotation.x/360);
        sensor.AddObservation(Cuisse_LEFT_JOINT.targetRotation.x/360);
        sensor.AddObservation(Cuisse_LEFT_JOINT.targetRotation.y/360);
        sensor.AddObservation(Cuisse_LEFT_JOINT.targetRotation.z/360);
        sensor.AddObservation(Cuisse_RIGHT_JOINT.targetRotation.x/360);
        sensor.AddObservation(Cuisse_RIGHT_JOINT.targetRotation.y/360);
        sensor.AddObservation(Cuisse_RIGHT_JOINT.targetRotation.z/360);
        sensor.AddObservation(Shoulder_LEFT_JOINT.targetRotation.x/360);
        sensor.AddObservation(Shoulder_LEFT_JOINT.targetRotation.y/360);
        sensor.AddObservation(Shoulder_LEFT_JOINT.targetRotation.z/360);
        sensor.AddObservation(Shoulder_RIGHT_JOINT.targetRotation.x/360);
        sensor.AddObservation(Shoulder_RIGHT_JOINT.targetRotation.y/360);
        sensor.AddObservation(Shoulder_RIGHT_JOINT.targetRotation.z/360);
        sensor.AddObservation(Coude_RIGHT_JOINT.targetRotation.x/360);
        sensor.AddObservation(Coude_LEFT_JOINT.targetRotation.x/360);
        sensor.AddObservation(Abdos_JOINT.targetRotation.x/360);
        sensor.AddObservation(Abdos_JOINT.targetRotation.y/360);
        sensor.AddObservation(Torse_JOINT.targetRotation.y/360);
        sensor.AddObservation(Head_JOINT.targetRotation.x/360);
        sensor.AddObservation(Head_JOINT.targetRotation.y/360);
        sensor.AddObservation(Bassin_JOINT.targetRotation.x/360);
        sensor.AddObservation(Bassin_JOINT.targetRotation.y/360);
        sensor.AddObservation(Foot_LEFT_JOINT.targetRotation.x/360);
        sensor.AddObservation(Foot_LEFT_JOINT.targetRotation.y/360);
        sensor.AddObservation(Foot_RIGHT_JOINT.targetRotation.x/360);
        sensor.AddObservation(Foot_RIGHT_JOINT.targetRotation.y/360);
        Vector3 currentCoG = CalculateCenterOfGravity(); // Assume que cela retourne le centre de gravité
        currentCoG = new Vector3(currentCoG.x, 0, currentCoG.z);
        Vector3 midpoint = (Foot_LEFT.position + Foot_RIGHT.position) / 2;
        midpoint = new Vector3(midpoint.x, 0, midpoint.z);
        float distanceToMidpoint = Vector3.Distance(currentCoG, midpoint);
        sensor.AddObservation(distanceToMidpoint);
        sensor.AddObservation(((((Bassin.position.z + midpoint.z)/2) + Head.position.z)/2)-currentCoG.z);
        sensor.AddObservation(Hand_LEFT.position.z - currentCoG.z);
        sensor.AddObservation(Hand_RIGHT.position.z - currentCoG.z);
        sensor.AddObservation(Head.position.z - currentCoG.z);
        sensor.AddObservation(Torse.position.z - currentCoG.z);
        sensor.AddObservation(Bassin.position.z - currentCoG.z);
        sensor.AddObservation(Foot_LEFT.position.z - currentCoG.z);
        sensor.AddObservation(Foot_RIGHT.position.z - currentCoG.z);
    }
    private void AddBodyPartObservation(Transform bodyPart, VectorSensor sensor)
    {
        // Position relative par rapport au torse et la vitesse de la partie du corps
        sensor.AddObservation((bodyPart.localPosition - Torse.localPosition)/ 10f);
        sensor.AddObservation(bodyPart.rotation.eulerAngles/360f);
        sensor.AddObservation(bodyPart.localPosition/ 100f);
        Rigidbody rb = bodyPart.GetComponent<Rigidbody>();
        sensor.AddObservation(Vector3.Angle(bodyPart.up, Vector3.up) / 180.0f);
        if (rb != null)
        {
            sensor.AddObservation(rb.velocity/5f);
            sensor.AddObservation(rb.angularVelocity/5f);
        }
    }

    public void HandleCollision(string bodyPartName, Collider other)
    {
        //Debug.Log(bodyPartName + " a touché " + other.gameObject.name);
        if(other.gameObject.name == "Target")
        {
            reward(10000f);
            Debug.Log("TARGET TOUCHED !");
            AddLog("Target", 1f, Unity.MLAgents.StatAggregationMethod.Sum);
            EndEpisode();
        }
        if(other.gameObject.name == "Plate")
        {
            reward(100f * currentProportionalHeadHeight, "Plate");
            Pressure_Plate.gameObject.SetActive(false);
            //Pressure_Plate.localPosition = new Vector3(UnityEngine.Random.Range(-25,25),-1f,UnityEngine.Random.Range(-14,-64));
        }
        if(other.gameObject.name == "Floor" && bodyPartName != "Pied.R" && bodyPartName != "Pied.L" )
        {

            if(bodyPartName == "Bassin")
            {
                reward(-1f);
                //EndEpisode();
                //reward(-1f);
            }
            if(bodyPartName == "Torse")
            {
                reward(-2f);
            }
            if(bodyPartName == "Abdos")
            {
                reward(-2f);
            }
            if(bodyPartName == "Head")
            {
                reward(-2f);
            }
            if(bodyPartName == "Tibias.R" || bodyPartName == "Tibias.L")
            {
                reward(-1f);
            }
            if(bodyPartName == "Main.R" || bodyPartName == "Main.L")
            {
                reward(-1f);
            }
        }
        else if(other.gameObject.name == "Floor" && (bodyPartName == "Pied.R" || bodyPartName == "Pied.L"))
        {
            reward(1f);
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

    private float hasAFootInAir(bool aFootInAir, bool twoFootOnGround)
    {
        float ret = 0f;
        if(twoFootOnGround)
        {
            if(leftFootContact.LeftFootOnFloor == true && rightFootContact.RightFootOnFloor == true)
            {
                ret = 0.5f;
            }
            else if(leftFootContact.LeftFootOnFloor == false && rightFootContact.RightFootOnFloor == false)
            {
                ret = -0.5f;
            }
        }
        else
        {
            if(leftFootContact.LeftFootOnFloor == false && rightFootContact.RightFootOnFloor == false)
            {
                ret-=2f;
            }
            else if(leftFootContact.LeftFootOnFloor != rightFootContact.RightFootOnFloor)
            {
                ret+=1f;
            }
            else if(leftFootContact.LeftFootOnFloor == true && rightFootContact.RightFootOnFloor == true)
            {
                ret-=0.01f;
            }
        }
        return ret;
    }

    public float headOnTop(int penality = 0, float baseValue = 1f, bool timeExpo = false, float tolerance = 1f)
    {
        float minHeight = -0.2f;  // Hauteur minimale au-dessus de la hauteur normale
        float maxHeight = 0.15f;   // Hauteur maximale au-dessus de la hauteur normale

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
            // La tête est dans la plage idéale
            if(timeExpo)
            {
                reward = baseValue * ((TempsSession*5) + 1);
            }
            else
            {
                reward = baseValue;
            }
            if(leftFootContact.LeftFootOnFloor )
            {
                if(Vector3.Angle(Tibias_LEFT.up, Vector3.up) < 25)
                {
                    if(Vector3.Angle(Cuisse_LEFT.up, Vector3.up) < 60)
                    {
                        if(Vector3.Angle(Bassin.up, Vector3.up) < 25)
                        {
                            reward += reward * 2f;
                        }
                        else
                        {
                            reward += reward * 1.5f;
                        }
                    }
                    else
                    {
                        reward += reward ;
                    }
                }
                else
                {
                    reward += reward;
                }
            }
            else if(rightFootContact.RightFootOnFloor )
            {
                if(Vector3.Angle(Tibias_RIGHT.up, Vector3.up) < 25)
                {
                    if(Vector3.Angle(Cuisse_RIGHT.up, Vector3.up) < 60)
                    {
                        if(Vector3.Angle(Bassin.up, Vector3.up) < 25)
                        {
                            reward += reward * 2f;
                        }
                        else
                        {
                            reward += reward * 1.5f;
                        }
                    }
                    else
                    {
                        reward += reward ;
                    }
                }
                else
                {
                    reward += reward;
                }
            }
        }
        else if (effectiveDistance <= tolerance)
        {
            // La récompense diminue linéairement de 1 à 0 à mesure que la distance augmente jusqu'à la tolérance
            if(timeExpo)
            {
                reward = baseValue* ((TempsSession*2) + 1) - (effectiveDistance / tolerance)*baseValue ;
            }
            else
            {
                reward = baseValue - (effectiveDistance / tolerance)*baseValue ;
            }

            //Debug.Log("baseValue:"+baseValue+ " -"+(effectiveDistance / tolerance)+" * TempsSession:"+TempsSession+"*2+1");
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

        //Debug.Log(leftFootParallel + "  -  "+ rightFootParallel);



        // Positions des pieds en coordonnées locales par rapport au Bassin
        Vector3 leftFootPosition = Bassin.InverseTransformPoint(Foot_LEFT.position);
        Vector3 rightFootPosition = Bassin.InverseTransformPoint(Foot_RIGHT.position);

        // Utilisation de la composante z pour déterminer quel pied est en avant
        //bool isLeftFootInFront = leftFootPosition.z > rightFootPosition.z;
        isLeftFootInFront = leftFootPosition.z - rightFootPosition.z > 0.005;
        //Debug.Log(leftFootPosition.z - rightFootPosition.z);

        if(isLeftFootInFront != lastStepInProgress)
        {
            stepInProgressLeft = true;
            stepInProgressRight = true;
            lastStepInProgress = isLeftFootInFront;
        }

        if (isLeftFootInFront && stepInProgressLeft == true)
        {
            stepInProgressLeft = true;
            stepInProgressRight = false;
            timeSinceLastStepLeft += Time.deltaTime;
            timeSinceLastStepRight = 0;
        }
        else if (!isLeftFootInFront && stepInProgressRight == true)
        {
            stepInProgressLeft = false;
            stepInProgressRight = true;
            timeSinceLastStepRight += Time.deltaTime;
            timeSinceLastStepLeft = 0;  // Réinitialiser le timer si les positions changent
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
        if (timeSinceLastStepLeft > minStepDuration && timeSinceLastStepLeft < maxStepDuration && stepInProgressLeft == true && leftFootContact.LeftFootOnFloor && leftFootParallel && (Vector3.Angle(Tibias_LEFT.up, Vector3.up) < 25) && (Vector3.Angle(Bassin.up, Vector3.up) < 25) && (Vector3.Angle(Cuisse_LEFT.up, Vector3.up) < 25))
        {
            if(firstStep == true)
            {
                firstStep = false;
                bonus += 1f;
            }
            else
            {
                bonus += 1f;  // Récompense pour maintenir un pied devant l'autre
            }
            enchainementStep++;
            stepInProgressLeft = false;
        }
        // Pénaliser l'agent si le pied reste en avant plus de 2 secondes
        else if (timeSinceLastStepLeft >= maxStepDuration)
        {
            //Debug.Log("stagne");
            bonus -= 1f;
            timeSinceLastStepLeft = 0;  // Réinitialiser pour éviter des pénalités continues
            stepInProgressLeft = false;
        }
        else if (timeSinceLastStepRight > minStepDuration && timeSinceLastStepRight < maxStepDuration && stepInProgressRight == true && rightFootContact.RightFootOnFloor && rightFootParallel && (Vector3.Angle(Tibias_RIGHT.up, Vector3.up) < 25) && (Vector3.Angle(Bassin.up, Vector3.up) < 25) && (Vector3.Angle(Cuisse_LEFT.up, Vector3.up) < 25))
        {
            if(firstStep == true)
            {
                firstStep = false;
                bonus += 1f;
            }
            else
            {
                bonus += 1f;  // Récompense pour maintenir un pied devant l'autre
            }
            enchainementStep++;
            stepInProgressRight = false;
        }
        // Pénaliser l'agent si le pied reste en avant plus de 2 secondes
        else if (timeSinceLastStepRight >= maxStepDuration)
        {
            //Debug.Log("stagne");
            bonus -= 1f;
            timeSinceLastStepRight = 0;  // Réinitialiser pour éviter des pénalités continues
            stepInProgressRight = false;
        }
        return bonus;
    }
    // private Vector3 CalculateCenterOfGravity()
    // {
    //     Vector3 totalCenterOfMass = Vector3.zero;
    //     float totalMass = 0f;

    //     foreach (Transform part in bodyParts)
    //     {
    //         Rigidbody rb = part.GetComponent<Rigidbody>();
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
            currentHeadHeight = hit.distance;
        }
        //currentProportionalHeadHeight = MeasureGroundDistance(Head, normalHeadHeight);
        //currentProportionalHeadHeight = (Head.position.y-1.55f) / (normalHeadHeight-1.55f);
        textPlateforme.text = string.Format("{0:N2}",currentProportionalHeadHeight);  
    }

    public float CenterOfGravityReward()
    {
        Vector3 currentCoG = CalculateCenterOfGravity(); // Assume que cela retourne le centre de gravité
        currentCoG = new Vector3(currentCoG.x, 0, currentCoG.z);
        Vector3 midpoint = (Foot_LEFT.position + Foot_RIGHT.position) / 2;
        midpoint = new Vector3(midpoint.x, 0, midpoint.z);
        float reward = 0f;

        if (leftFootContact.LeftFootOnFloor && rightFootContact.RightFootOnFloor)
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
        if (Application.isPlaying) // Assurez-vous que cela est dessiné uniquement si l'application est en cours d'exécution
        {
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
            Vector2 pointA = new Vector2(Foot_LEFT.position.x, Foot_LEFT.position.z-0.06f);
            Vector2 pointB = new Vector2(Foot_RIGHT.position.x, Foot_RIGHT.position.z-0.06f);
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
        }
    }

    private float FlipTrain()
    {
        //Debug.Log(1-Vector3.Dot(Torse.forward, Vector3.up) );
        if(1-Vector3.Dot(Torse.forward, Vector3.up) > 1.9f)
        {
            Debug.Log("RETOURNE !!!!");
            reward(100f);
            EndEpisode();
        }
        return 1-Vector3.Dot(Torse.forward, Vector3.up);
    }
    private bool CheckIfLanded()
    {
        if(leftFootContact.LeftFootOnFloor || rightFootContact.RightFootOnFloor || bassinContact.BassinOnFloor || torseContact.TorseOnFloor || leftHandContact.LeftHandOnFloor || rightHabdContact.LeftHandOnFloor)
        {
            return true;
        }
        return false;
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
}

