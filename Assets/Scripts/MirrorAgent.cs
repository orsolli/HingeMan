using UnityEngine;

public class MirrorAgent : HumanAgent
{
    public HumanAgent agent;
    Matrix4x4 ReflectByX = new Matrix4x4(
        new Vector4(-1, 0, 0, 0),
        new Vector4(0, 1, 0, 0),
        new Vector4(0, 0, 1, 0),
        new Vector4(0, 0, 0, 1)
    );

    protected override void CreateBody()
    {
        body = Instantiate(agent.body, transform);
        actuateMirror(agent.body.transform, body.transform);
        ActuatePosition.limbToBody(body.transform.GetChild((int)HumanAgent.Limbs.Head).GetComponent<HingeJoint>());
        ActuatePosition.limbToBody(body.transform.GetChild((int)HumanAgent.Limbs.LeftElbow).GetComponent<HingeJoint>());
        ActuatePosition.limbToBody(body.transform.GetChild((int)HumanAgent.Limbs.RightElbow).GetComponent<HingeJoint>());
        ActuatePosition.limbToBody(body.transform.GetChild((int)HumanAgent.Limbs.LeftFoot).GetComponent<HingeJoint>());
        ActuatePosition.limbToBody(body.transform.GetChild((int)HumanAgent.Limbs.RightFoot).GetComponent<HingeJoint>());
        head = body.transform.Find("Head");
        focusPoint = blindFocus(head);
        avg_velocity = agent.transform.rotation * (ReflectByX * (Quaternion.Inverse(agent.transform.rotation) * agent.avg_velocity));
        avg_acceleration = new Acceleration()
        {
            acceleration = agent.transform.rotation * (ReflectByX * (Quaternion.Inverse(agent.transform.rotation) * agent.avg_acceleration.acceleration)),
            angular_acceleration = agent.transform.rotation * (ReflectByX * (Quaternion.Inverse(agent.transform.rotation) * agent.avg_acceleration.angular_acceleration))
        };
        previous_velocity = agent.transform.rotation * (ReflectByX * (Quaternion.Inverse(agent.transform.rotation) * agent.previous_velocity));
        angular_velocity = agent.transform.rotation * (ReflectByX * (Quaternion.Inverse(agent.transform.rotation) * agent.angular_velocity));
    }

    public static void actuateMirror(Transform body, Transform mirror)
    {
        var body_rotation = body.GetChild((int)Limbs.Pelvis).localRotation;
        body_rotation = Quaternion.Euler(body_rotation.eulerAngles.x, -body_rotation.eulerAngles.y, -body_rotation.eulerAngles.z);
        mirror.GetChild((int)Limbs.Pelvis).localRotation = body_rotation;
        mirror.GetChild((int)Limbs.Pelvis).localPosition = body.GetChild((int)Limbs.Pelvis).localPosition;
        HingeJoint[] hinges = body.GetComponentsInChildren<HingeJoint>();
        HingeJoint[] mirrorHinges = mirror.GetComponentsInChildren<HingeJoint>();
        var spring = mirrorHinges[(int)Hinges.Head].spring;
        spring.targetPosition = hinges[(int)Hinges.Head].angle;
        mirrorHinges[(int)Hinges.Head].spring = spring;
        spring = mirrorHinges[(int)Hinges.NeckNo].spring;
        spring.targetPosition = -hinges[(int)Hinges.NeckNo].angle;
        mirrorHinges[(int)Hinges.NeckNo].spring = spring;
        spring = mirrorHinges[(int)Hinges.NeckYes].spring;
        spring.targetPosition = hinges[(int)Hinges.NeckYes].angle;
        mirrorHinges[(int)Hinges.NeckYes].spring = spring;
        spring = mirrorHinges[(int)Hinges.Spine].spring;
        spring.targetPosition = hinges[(int)Hinges.Spine].angle;
        mirrorHinges[(int)Hinges.Spine].spring = spring;
        spring = mirrorHinges[(int)Hinges.Tale].spring;
        spring.targetPosition = -hinges[(int)Hinges.Tale].angle;
        mirrorHinges[(int)Hinges.Tale].spring = spring;
        spring = mirrorHinges[(int)Hinges.RightSholder].spring;
        spring.targetPosition = hinges[(int)Hinges.LeftSholder].angle;
        mirrorHinges[(int)Hinges.RightSholder].spring = spring;
        spring = mirrorHinges[(int)Hinges.RightBicep].spring;
        spring.targetPosition = hinges[(int)Hinges.LeftBicep].angle;
        mirrorHinges[(int)Hinges.RightBicep].spring = spring;
        spring = mirrorHinges[(int)Hinges.RightArm].spring;
        spring.targetPosition = hinges[(int)Hinges.LeftArm].angle;
        mirrorHinges[(int)Hinges.RightArm].spring = spring;
        spring = mirrorHinges[(int)Hinges.RightElbow].spring;
        spring.targetPosition = hinges[(int)Hinges.LeftElbow].angle;
        mirrorHinges[(int)Hinges.RightElbow].spring = spring;
        spring = mirrorHinges[(int)Hinges.LeftPelvis].spring;
        spring.targetPosition = hinges[(int)Hinges.RightPelvis].angle;
        mirrorHinges[(int)Hinges.LeftPelvis].spring = spring;
        spring = mirrorHinges[(int)Hinges.LeftThigh].spring;
        spring.targetPosition = hinges[(int)Hinges.RightThigh].angle;
        mirrorHinges[(int)Hinges.LeftThigh].spring = spring;
        spring = mirrorHinges[(int)Hinges.LeftLeg].spring;
        spring.targetPosition = hinges[(int)Hinges.RightLeg].angle;
        mirrorHinges[(int)Hinges.LeftLeg].spring = spring;
        spring = mirrorHinges[(int)Hinges.LeftFoot].spring;
        spring.targetPosition = hinges[(int)Hinges.RightFoot].angle;
        mirrorHinges[(int)Hinges.LeftFoot].spring = spring;
        spring = mirrorHinges[(int)Hinges.LeftSholder].spring;
        spring.targetPosition = hinges[(int)Hinges.RightSholder].angle;
        mirrorHinges[(int)Hinges.LeftSholder].spring = spring;
        spring = mirrorHinges[(int)Hinges.LeftBicep].spring;
        spring.targetPosition = hinges[(int)Hinges.RightBicep].angle;
        mirrorHinges[(int)Hinges.LeftBicep].spring = spring;
        spring = mirrorHinges[(int)Hinges.LeftArm].spring;
        spring.targetPosition = hinges[(int)Hinges.RightArm].angle;
        mirrorHinges[(int)Hinges.LeftArm].spring = spring;
        spring = mirrorHinges[(int)Hinges.LeftElbow].spring;
        spring.targetPosition = hinges[(int)Hinges.RightElbow].angle;
        mirrorHinges[(int)Hinges.LeftElbow].spring = spring;
        spring = mirrorHinges[(int)Hinges.RightPelvis].spring;
        spring.targetPosition = hinges[(int)Hinges.LeftPelvis].angle;
        mirrorHinges[(int)Hinges.RightPelvis].spring = spring;
        spring = mirrorHinges[(int)Hinges.RightThigh].spring;
        spring.targetPosition = hinges[(int)Hinges.LeftThigh].angle;
        mirrorHinges[(int)Hinges.RightThigh].spring = spring;
        spring = mirrorHinges[(int)Hinges.RightLeg].spring;
        spring.targetPosition = hinges[(int)Hinges.LeftLeg].angle;
        mirrorHinges[(int)Hinges.RightLeg].spring = spring;
        spring = mirrorHinges[(int)Hinges.RightFoot].spring;
        spring.targetPosition = hinges[(int)Hinges.LeftFoot].angle;
        mirrorHinges[(int)Hinges.RightFoot].spring = spring;
    }
}
