using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class AgentWalk : Agent
{
    private Rigidbody rb;
    public Transform cul;
    public Transform footLeft;
    public Transform footRight;
    public Transform tibiasLeft;
    public Transform tibiasRight;
    public Transform cuisseLeft;
    public Transform cuisseRight;
    public Transform fullBody;


    public Vector3 FR_Pos;
    public Vector3 FL_Pos;
    public Vector3 TR_Pos;
    public Vector3 TL_Pos;
    public Vector3 CR_Pos;
    public Vector3 CL_Pos;
    public Vector3 CUL_Pos;
    public Vector3 FULL_Pos;


    public Quaternion CUL_ROTA_POS;
    public Quaternion FR_ROTA_POS;
    public Quaternion FL_ROTA_POS;
    public Quaternion TR_ROTA_POS;
    public Quaternion TL_ROTA_POS;
    public Quaternion CR_ROTA_POS;
    public Quaternion CL_ROTA_POS;
    public Quaternion CU_ROTAL_POS;
    private float bonusReward;


    [SerializeField] private float moveSpeed = 1f;
    public override void Initialize()
    {
        CUL_Pos = cul.localPosition;
        FR_Pos = footRight.localPosition;
        FL_Pos = footLeft.localPosition;
        TL_Pos = tibiasLeft.localPosition;
        TR_Pos = tibiasRight.localPosition;
        CL_Pos = cuisseLeft.localPosition;
        CR_Pos = cuisseRight.localPosition;
        FULL_Pos = fullBody.localPosition;


        CUL_ROTA_POS = cul .transform.rotation;
        FR_ROTA_POS = footRight.transform.rotation;
        FL_ROTA_POS = footLeft.transform.rotation;
        TL_ROTA_POS = tibiasLeft.transform.rotation;
        TR_ROTA_POS = tibiasRight.transform.rotation;
        CL_ROTA_POS = cuisseLeft.transform.rotation;
        CR_ROTA_POS = cuisseRight.transform.rotation;
    }

    public override void OnEpisodeBegin()
    {
        bonusReward = -1f;
        fullBody.localPosition = FULL_Pos;

        Rigidbody rbCul = cul.GetComponent<Rigidbody>();
        cul.transform.rotation = CUL_ROTA_POS;
        resetPosition(cul, 0.2f, CUL_Pos, CUL_ROTA_POS);
        Rigidbody rbfootRight = footRight.GetComponent<Rigidbody>();
        resetPosition(footRight, 0.1f, FR_Pos, FR_ROTA_POS);
        Rigidbody rbfootLeft = footLeft.GetComponent<Rigidbody>();
        resetPosition(footLeft, 0.1f, FL_Pos, FL_ROTA_POS);
        Rigidbody rbtibiasLeft = tibiasLeft.GetComponent<Rigidbody>();
        resetPosition(tibiasLeft, 0.1f, TL_Pos, TL_ROTA_POS);
        Rigidbody rbtibiasRight = tibiasRight.GetComponent<Rigidbody>();
        resetPosition(tibiasRight, 0.1f, TR_Pos, TR_ROTA_POS);
        Rigidbody rbcuisseLeft = cuisseLeft.GetComponent<Rigidbody>();
        resetPosition(cuisseLeft, 0.1f, CL_Pos, CL_ROTA_POS);
        Rigidbody rbcuisseRight = cuisseRight.GetComponent<Rigidbody>();
        resetPosition(cuisseRight, 0.1f, CR_Pos, CR_ROTA_POS);

    }

    public void resetPosition(Transform membre, float timer, Vector3 origin_Pos, Quaternion originRota)
    {
        Rigidbody rbMembre = membre.GetComponent<Rigidbody>();
        membre.transform.rotation = originRota;
        membre.localPosition = origin_Pos;
        membre.localPosition = origin_Pos;
        rbMembre.isKinematic = true;
        StartCoroutine(ReleaseTorse(rbMembre, timer, membre, origin_Pos));
    }
    IEnumerator ReleaseTorse(Rigidbody partie, float time, Transform membre, Vector3 origin_Pos)
    {
        yield return new WaitForSeconds(time); // Attendez une seconde
        membre.localPosition = origin_Pos;
        //Rigidbody rbTorse = cul.GetComponent<Rigidbody>();
        partie.isKinematic = false; // Permet au torse d'être à nouveau affecté par la physique
    }
            
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);

        sensor.AddObservation(footLeft.localPosition);
        // Vitesse de l'articulation1 (si elle a un Rigidbody attaché)
        if (footLeft.GetComponent<Rigidbody>() != null)
        {
            sensor.AddObservation(footLeft.GetComponent<Rigidbody>().velocity);
        }


        sensor.AddObservation(footRight.localPosition);
        // Vitesse de l'articulation1 (si elle a un Rigidbody attaché)
        if (footRight.GetComponent<Rigidbody>() != null)
        {
            sensor.AddObservation(footRight.GetComponent<Rigidbody>().velocity);
        }

        sensor.AddObservation(tibiasLeft.localPosition);
        // Vitesse de l'articulation1 (si elle a un Rigidbody attaché)
        if (tibiasLeft.GetComponent<Rigidbody>() != null)
        {
            sensor.AddObservation(tibiasLeft.GetComponent<Rigidbody>().velocity);
        }
        sensor.AddObservation(tibiasRight.localPosition);
        // Vitesse de l'articulation1 (si elle a un Rigidbody attaché)
        if (tibiasRight.GetComponent<Rigidbody>() != null)
        {
            sensor.AddObservation(tibiasRight.GetComponent<Rigidbody>().velocity);
        }
        sensor.AddObservation(cuisseLeft.localPosition);
        // Vitesse de l'articulation1 (si elle a un Rigidbody attaché)
        if (cuisseLeft.GetComponent<Rigidbody>() != null)
        {
            sensor.AddObservation(cuisseLeft.GetComponent<Rigidbody>().velocity);
        }
        sensor.AddObservation(cuisseRight.localPosition);
        // Vitesse de l'articulation1 (si elle a un Rigidbody attaché)
        if (cuisseRight.GetComponent<Rigidbody>() != null)
        {
            sensor.AddObservation(cuisseRight.GetComponent<Rigidbody>().velocity);
        }        
    }
    public override void OnActionReceived(ActionBuffers actions)
    {















        bonusReward += 0.1f;
        float moveX = actions.ContinuousActions[0];
        float moveZ = actions.ContinuousActions[1];
        float rotateY = actions.ContinuousActions[2];

        // Supposons que action[3] est l'action pour lever le pied droit
        float liftRFootAction = actions.ContinuousActions[3];
        // Supposons que footRightRb est le Rigidbody du pied droit
        Rigidbody footRightRb = footRight.GetComponent<Rigidbody>();
        // Appliquer une force pour lever le pied droit
        Vector3 liftDirection = transform.up * liftRFootAction; // Utilisez transform.up pour lever le pied vers le haut
        footRightRb.AddForce(liftDirection, ForceMode.VelocityChange);


        // Supposons que action[4] est l'action pour lever le pied droit
        float liftLFootAction = actions.ContinuousActions[4];
        // Supposons que footRightRb est le Rigidbody du pied droit
        Rigidbody footLeftRb = footLeft.GetComponent<Rigidbody>();
        // Appliquer une force pour lever le pied droit
        Vector3 liftLDirection = transform.up * liftLFootAction; // Utilisez transform.up pour lever le pied vers le haut
        footLeftRb.AddForce(liftLDirection, ForceMode.VelocityChange);

        // Appliquer le mouvement basé sur les actions
        transform.localPosition += new Vector3(moveX, 0, moveZ) * Time.deltaTime * moveSpeed;
    
        // Appliquer la rotation
        transform.Rotate(new Vector3(0, rotateY, 0), Time.deltaTime * moveSpeed);

        // Exemple de calcul de récompense basé sur la hauteur du torse
        float torsoHeight = cul.position.y;
        float reward = torsoHeight - 1; // Exemple de formule, ajustez selon vos besoins


        if(torsoHeight < 4.4 && bonusReward > 2)
        {
            float distance = Vector3.Distance(FULL_Pos, fullBody.position);
            AddReward(distance * 2 + bonusReward);

            //EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionOut)
    {
        ActionSegment<float> continuousActions = actionOut.ContinuousActions;
        continuousActions[3] = Input.GetAxisRaw("Horizontal");
        continuousActions[4] = Input.GetAxisRaw("Vertical");
        //continuousActions[2] = Input.GetAxisRaw("Axis Up");
    }

}
