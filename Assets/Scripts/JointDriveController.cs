using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using Unity.MLAgents;

namespace Unity.MLAgentsExamples
{
    /// <summary>
    /// Used to store relevant information for acting and learning for each body part in agent.
    /// </summary>
    [System.Serializable]
    public class BodyPart
    {
        [Header("Body Part Info")][Space(10)] public ConfigurableJoint joint;
        public Rigidbody rb;
        [HideInInspector] public Vector3 startingPos;
        [HideInInspector] public Quaternion startingRot;

        [Header("Ground & Target Contact")]
        [Space(10)]
        public GroundContact groundContact;

        public TargetContact targetContact;

        [FormerlySerializedAs("thisJDController")]
        [HideInInspector] public JointDriveController thisJdController;

        [Header("Current Joint Settings")]
        [Space(10)]
        public Vector3 currentEularJointRotation;

        [HideInInspector] public float currentStrength;
        public float currentXNormalizedRot;
        public float currentYNormalizedRot;
        public float currentZNormalizedRot;

        [Header("Other Debug Info")]
        [Space(10)]
        public Vector3 currentJointForce;

        public float currentJointForceSqrMag;
        public Vector3 currentJointTorque;
        public float currentJointTorqueSqrMag;
        public AnimationCurve jointForceCurve = new AnimationCurve();
        public AnimationCurve jointTorqueCurve = new AnimationCurve();

        /// <summary>
        /// Reset body part to initial configuration.
        /// </summary>
        public void Reset(BodyPart bp, Vector3 modification)
        {
            if (bp.rb != null && bp.rb.isKinematic == false)
            {
            }
                bp.rb.velocity = Vector3.zero;
                bp.rb.angularVelocity = Vector3.zero;
                bp.rb.isKinematic = true;
                bp.rb.transform.position = bp.startingPos + modification;
                bp.rb.transform.rotation = bp.startingRot;
                bp.rb.isKinematic = false;
            if (bp.groundContact)
            {
                bp.groundContact.touchingGround = false;
            }

            if (bp.targetContact)
            {
                bp.targetContact.touchingTarget = false;
            }
        }

        /// <summary>
        /// Apply torque according to defined goal `x, y, z` angle and force `strength`.
        /// </summary>
        public void SetJointTargetRotation(float x, float y, float z, bool constrain=true)
        {
            float xRot;
            float yRot;
            float zRot;
            if(constrain)
            {
                x = (x + 1f) * 0.5f;
                y = (y + 1f) * 0.5f;
                z = (z + 1f) * 0.5f;

                xRot = Mathf.Lerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, x);
                yRot = Mathf.Lerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, y);
                zRot = Mathf.Lerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, z);
            }
            else
            {
                xRot = x == 0? 0 : x > 0? joint.highAngularXLimit.limit*x : -joint.lowAngularXLimit.limit*x;
                yRot = y == 0? 0 : y > 0? joint.angularYLimit.limit*y : joint.angularYLimit.limit*y;
                zRot = z == 0? 0 : z > 0? joint.angularZLimit.limit*z : joint.angularZLimit.limit*z;
            }


            currentXNormalizedRot =
                Mathf.InverseLerp(joint.lowAngularXLimit.limit, joint.highAngularXLimit.limit, xRot);
            currentYNormalizedRot = Mathf.InverseLerp(-joint.angularYLimit.limit, joint.angularYLimit.limit, yRot);
            currentZNormalizedRot = Mathf.InverseLerp(-joint.angularZLimit.limit, joint.angularZLimit.limit, zRot);

            joint.targetRotation = Quaternion.Euler(xRot, yRot, zRot);
            currentEularJointRotation = new Vector3(xRot, yRot, zRot);
        }

        public void SetJointStrength(float strength)
        {
            var rawVal = (strength + 1f) * 0.5f * thisJdController.maxJointForceLimit;
            var jd = new JointDrive
            {
                positionSpring = thisJdController.maxJointSpring,
                positionDamper = thisJdController.jointDampen,
                maximumForce = rawVal
            };
            joint.slerpDrive = jd;
            currentStrength = jd.maximumForce;
        }
    }

    public class JointDriveController : MonoBehaviour
    {
        [Header("Joint Drive Settings")]
        [Space(10)]
        public float maxJointSpring;

        public float jointDampen;
        public float maxJointForceLimit;

        [HideInInspector] public Dictionary<Transform, BodyPart> bodyPartsDict = new Dictionary<Transform, BodyPart>();

        [HideInInspector] public List<BodyPart> bodyPartsList = new List<BodyPart>();
        const float k_MaxAngularVelocity = 50.0f;

        /// <summary>
        /// Create BodyPart object and add it to dictionary.
        /// </summary>
        public void SetupBodyPart(Transform t)
        {
            var bp = new BodyPart
            {
                rb = t.GetComponent<Rigidbody>(),
                joint = t.GetComponent<ConfigurableJoint>(),
                startingPos = t.position,
                startingRot = t.rotation
            };
            bp.rb.maxAngularVelocity = k_MaxAngularVelocity;

            // Add & setup the ground contact script
            bp.groundContact = t.GetComponent<GroundContact>();
            if (!bp.groundContact)
            {
                bp.groundContact = t.gameObject.AddComponent<GroundContact>();
                bp.groundContact.agent = gameObject.GetComponent<RobotWalk>();
            }
            else
            {
                bp.groundContact.agent = gameObject.GetComponent<RobotWalk>();
            }

            if (bp.joint)
            {
                var jd = new JointDrive
                {
                    positionSpring = maxJointSpring,
                    positionDamper = jointDampen,
                    maximumForce = maxJointForceLimit
                };
                bp.joint.slerpDrive = jd;
            }

            bp.thisJdController = this;
            bodyPartsDict.Add(t, bp);
            bodyPartsList.Add(bp);
        }

        public void GetCurrentJointForces()
        {
            foreach (var bodyPart in bodyPartsDict.Values)
            {
                if (bodyPart.joint)
                {
                    bodyPart.currentJointForce = bodyPart.joint.currentForce;
                    bodyPart.currentJointForceSqrMag = bodyPart.joint.currentForce.magnitude;
                    bodyPart.currentJointTorque = bodyPart.joint.currentTorque;
                    bodyPart.currentJointTorqueSqrMag = bodyPart.joint.currentTorque.magnitude;
                    if (Application.isEditor)
                    {
                        if (bodyPart.jointForceCurve.length > 1000)
                        {
                            bodyPart.jointForceCurve = new AnimationCurve();
                        }

                        if (bodyPart.jointTorqueCurve.length > 1000)
                        {
                            bodyPart.jointTorqueCurve = new AnimationCurve();
                        }

                        bodyPart.jointForceCurve.AddKey(Time.time, bodyPart.currentJointForceSqrMag);
                        bodyPart.jointTorqueCurve.AddKey(Time.time, bodyPart.currentJointTorqueSqrMag);
                    }
                }
            }
        }
    }
}
