﻿using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

public class MirrorAgent : Agent
{
    public HumanAgent agent;
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
        Vector3 avg_acc = Vector3.zero;
        Vector3 avg_angular_acc = Vector3.zero;
        for (int i = 0; i < HumanAgent.frames; i++)
        {
            avg_acc += agent.acceleration[i][head.gameObject.name] / HumanAgent.frames;
            avg_angular_acc += agent.angular_acceleration[i][head.gameObject.name] / HumanAgent.frames;
        }
        avg_acc /= HumanAgent.frames;
        avg_angular_acc /= HumanAgent.frames;
        Vector3 total_acceleration = avg_acc - Physics.gravity;
        total_acceleration = ReflectByX * (head_rotation * total_acceleration);
        sensor.AddObservation(Vector3.ClampMagnitude(total_acceleration / 100f, 1f));
        Vector3 total_angular_acceleration = ReflectByX * (head_rotation * avg_angular_acc);
        sensor.AddObservation(Vector3.ClampMagnitude(total_angular_acceleration / 100f, 1f));

        Vector3 focusPoint = Quaternion.Inverse(head_rotation) * (ReflectByX * (head_rotation * agent.focusPoint));

        Vector3 absRightEye = head.position + head.rotation * agent.leftEye;
        Vector3 absLeftEye = head.position + head.rotation * agent.rightEye;
        Quaternion lookAtRight = Quaternion.LookRotation(focusPoint - absRightEye, head.rotation * Vector3.up);
        sensor.AddObservation(lookAtRight);
        Quaternion lookAtLeft = Quaternion.LookRotation(focusPoint - absLeftEye, head.rotation * Vector3.up);
        sensor.AddObservation(lookAtLeft);
    }

    public override void OnActionReceived(float[] act)
    {
        float difference = 0;
        float[] actions = agent.GetAction();
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.Head], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.Head], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.NeckNo], -1, 1) + Mathf.Clamp(actions[(int)BodyPart.NeckNo], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.NeckYes], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.NeckYes], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.Spine], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.Spine], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.Tale], -1, 1) + Mathf.Clamp(actions[(int)BodyPart.Tale], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.LeftSholder], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightSholder], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.LeftBicep], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightBicep], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.LeftArm], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightArm], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.LeftElbow], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightElbow], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.RightPelvis], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftPelvis], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.RightThigh], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftThigh], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.RightLeg], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftLeg], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.RightFoot], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftFoot], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.RightSholder], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftSholder], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.RightBicep], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftBicep], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.RightArm], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftArm], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.RightElbow], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.LeftElbow], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.LeftPelvis], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightPelvis], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.LeftThigh], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightThigh], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.LeftLeg], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightLeg], -1, 1));
        difference += Mathf.Abs(Mathf.Clamp(act[(int)BodyPart.LeftFoot], -1, 1) - Mathf.Clamp(actions[(int)BodyPart.RightFoot], -1, 1));
        difference /= 21;
        difference /= 2;
        float rew = 0.18f - Mathf.Pow(difference, 2);
        SetReward(rew * 0.01f);
        Monitor.Log("Mirror", rew, MonitorType.slider, agent.transform);
    }

    void FixedUpdate()
    {
        if (agent.StepCount < this.StepCount)
        {
            EndEpisode();
        }
    }
}
