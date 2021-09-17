using UnityEngine;

public class ActuatePosition : MonoBehaviour
{
    public void FixedUpdate()
    {
        limbToBody(GetComponent<HingeJoint>());
    }

    public static Transform limbToBody(HingeJoint limb)
    {
        Transform body = limb.connectedBody.transform;
        HingeJoint nextJoint;
        if (body.TryGetComponent<HingeJoint>(out nextJoint))
        {
            body = limbToBody(nextJoint);
        }

        limb.transform.localRotation = body.localRotation * Quaternion.AngleAxis(
            limb.spring.targetPosition,
            limb.axis
        );
        limb.transform.localPosition = body.localPosition + (
            body.localRotation * Vector3.Scale(limb.connectedAnchor, body.transform.localScale) -
            limb.transform.localRotation * Vector3.Scale(limb.anchor, limb.transform.localScale)
        );
        return limb.transform;
    }
}