using UnityEngine;

namespace Unity.MLAgentsExamples
{
    /// <summary>
    /// Utility class to allow a stable observation platform.
    /// </summary>
    public class OrientationCubeController : MonoBehaviour
    {
        //Update position and Rotation
        // public void UpdateOrientation(Transform rootBP, Transform target)
        // {
        //     var dirVector = target.position - transform.position;
        //     dirVector.y = 0; //flatten dir on the y. this will only work on level, uneven surfaces
        //     var lookRot =
        //         dirVector == Vector3.zero
        //             ? Quaternion.identity
        //             : Quaternion.LookRotation(dirVector); //get our look rot to the target

        //     //UPDATE ORIENTATION CUBE POS & ROT
        //     transform.SetPositionAndRotation(rootBP.position, lookRot);
        // }
            // Update position and Rotation
        public float rotationSpeed = 90f;
        public void UpdateOrientation(Transform rootBP, Transform target)
        {
            var dirVector = target.position - transform.position;
            dirVector.y = 0; // flatten dir on the y. this will only work on level, uneven surfaces
            var lookRot =
                dirVector == Vector3.zero
                    ? Quaternion.identity
                    : Quaternion.LookRotation(dirVector); // get our look rot to the target

            // Calculate the step size based on the rotation speed and time
            float step = rotationSpeed * Time.deltaTime*25f;

            // Smoothly rotate towards the target rotation
            Quaternion newRotation = Quaternion.RotateTowards(transform.rotation, lookRot, step);
            //Debug.Log(Vector3.Angle(rootBP.position, target.position));

            // Update orientation cube position & rotation
            transform.SetPositionAndRotation(rootBP.position, newRotation);
        }
    }

    
}
