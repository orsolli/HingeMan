using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

public class MirrorAgent : Agent
{
    public HumanAgent agent;
    float[] mirroredActions;
    enum BodyPart
    {
        Head,
        NeckNo,
        NeckYes,
        Spine,
        Tale,
        Pelvis,
        LeftSholder,
        LeftBicep,
        LeftArm,
        LeftElbow,
        RightPelvis,
        RightThigh,
        RightLeg,
        RightFoot,
        RightSholder,
        RightBicep,
        RightArm,
        RightElbow,
        LeftPelvis,
        LeftThigh,
        LeftLeg,
        LeftFoot
    }

    private void observe(VectorSensor sensor, Transform bodyPart, bool negative = false)
    {
        int factor = negative ? -1 : 1;
        HingeJoint limb = bodyPart.GetComponent<HingeJoint>();
        float range = limb.limits.max - limb.limits.min;
        sensor.AddObservation(factor * 2 * (limb.angle - limb.limits.min) / range - 1);
        sensor.AddObservation(Mathf.Clamp(factor * limb.velocity / 2000f, -1, 1));
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        Transform body = agent.body.transform;
        Transform head = body.GetChild(((int)BodyPart.Head));
        Quaternion head_rotation = Quaternion.Inverse(head.rotation).normalized;
        observe(sensor, head);
        observe(sensor, body.GetChild((int)BodyPart.NeckNo), true);
        observe(sensor, body.GetChild((int)BodyPart.NeckYes));
        observe(sensor, body.GetChild((int)BodyPart.Spine));
        observe(sensor, body.GetChild((int)BodyPart.Tale), true);
        observe(sensor, body.GetChild((int)BodyPart.RightSholder));
        observe(sensor, body.GetChild((int)BodyPart.RightBicep));
        observe(sensor, body.GetChild((int)BodyPart.RightArm));
        observe(sensor, body.GetChild((int)BodyPart.RightElbow));
        observe(sensor, body.GetChild((int)BodyPart.LeftPelvis));
        observe(sensor, body.GetChild((int)BodyPart.LeftThigh));
        observe(sensor, body.GetChild((int)BodyPart.LeftLeg));
        observe(sensor, body.GetChild((int)BodyPart.LeftFoot));
        observe(sensor, body.GetChild((int)BodyPart.LeftSholder));
        observe(sensor, body.GetChild((int)BodyPart.LeftBicep));
        observe(sensor, body.GetChild((int)BodyPart.LeftArm));
        observe(sensor, body.GetChild((int)BodyPart.LeftElbow));
        observe(sensor, body.GetChild((int)BodyPart.RightPelvis));
        observe(sensor, body.GetChild((int)BodyPart.RightThigh));
        observe(sensor, body.GetChild((int)BodyPart.RightLeg));
        observe(sensor, body.GetChild((int)BodyPart.RightFoot));

        Vector3 N = head_rotation * Vector3.left;
        Matrix4x4 ReflectByX = new Matrix4x4(
            new Vector4(-1, 0, 0, 0),
            new Vector4(0, 1, 0, 0),
            new Vector4(0, 0, 1, 0),
            new Vector4(0, 0, 0, 1)
        );
        Vector3 relative_acc = ReflectByX * (head_rotation * agent.desired_acceleration);
        sensor.AddObservation(Vector3.ClampMagnitude(relative_acc / 100f, 1f));
        Rigidbody rigidHead = head.gameObject.GetComponent<Rigidbody>();
        Vector3 total_acceleration = agent.avg_acceleration.acceleration - Physics.gravity;
        total_acceleration = ReflectByX * (head_rotation * total_acceleration);
        sensor.AddObservation(Vector3.ClampMagnitude(total_acceleration / 100f, 1f));
        Vector3 total_angular_acceleration = ReflectByX * (head_rotation * agent.avg_acceleration.angular_acceleration);
        sensor.AddObservation(Vector3.ClampMagnitude(total_angular_acceleration / 100f, 1f));

        Vector3 focusPoint = Quaternion.Inverse(head_rotation) * (ReflectByX * (head_rotation * agent.focusPoint));

        Vector3 absRightEye = head.position + head.rotation * agent.leftEye;
        Vector3 absLeftEye = head.position + head.rotation * agent.rightEye;
        Quaternion lookAtRight = Quaternion.LookRotation(focusPoint - absRightEye, head.rotation * Vector3.up);
        sensor.AddObservation(lookAtRight);
        Quaternion lookAtLeft = Quaternion.LookRotation(focusPoint - absLeftEye, head.rotation * Vector3.up);
        sensor.AddObservation(lookAtLeft);

        Transform rightFoot = body.transform.Find("LeftFoot");
        Quaternion rightFoot_rotation = Quaternion.Inverse(rightFoot.rotation).normalized;
        FootSensor rightFootSensor = rightFoot.GetComponent<FootSensor>();
        sensor.AddObservation(Vector3.ClampMagnitude(rightFoot_rotation * rightFootSensor.impulse / 20f, 1f));

        Transform leftFoot = body.transform.Find("RightFoot");
        Quaternion leftFoot_rotation = Quaternion.Inverse(leftFoot.rotation).normalized;
        FootSensor leftFootSensor = leftFoot.GetComponent<FootSensor>();
        sensor.AddObservation(Vector3.ClampMagnitude(leftFoot_rotation * leftFootSensor.impulse / 20f, 1f));
    }

    public override void OnActionReceived(float[] act)
    {
        mirroredActions = act;
    }

    void FixedUpdate()
    {
        if (agent.StepCount < this.StepCount)
        {
            EndEpisode();
        }
        else if (StepCount % 15 == 1)
        {
            float difference = 0;
            float[] actions = agent.GetAction();
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.Head], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.Head], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.NeckNo], -1, 1) + Mathf.Clamp(actions[(int)BodyPart.NeckNo], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.NeckYes], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.NeckYes], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.Spine], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.Spine], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.Tale], -1, 1) + Mathf.Clamp(actions[(int)BodyPart.Tale], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.LeftSholder], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightSholder], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.LeftBicep], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightBicep], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.LeftArm], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightArm], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.LeftElbow], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightElbow], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.RightPelvis], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftPelvis], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.RightThigh], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftThigh], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.RightLeg], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftLeg], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.RightFoot], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftFoot], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.RightSholder], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftSholder], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.RightBicep], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftBicep], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.RightArm], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftArm], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.RightElbow], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftElbow], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.LeftPelvis], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightPelvis], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.LeftThigh], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightThigh], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.LeftLeg], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightLeg], -1, 1));
            difference += Mathf.Abs(Mathf.Clamp(mirroredActions[(int)BodyPart.LeftFoot], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightFoot], -1, 1));
            difference /= 21;
            float rew = Mathf.Pow(difference, 2);
            SetReward(Mathf.Clamp(agent.reward - rew, -1f, 1f));
            Monitor.Log("Mirror", -rew, MonitorType.slider, agent.transform);
        }
    }
}
