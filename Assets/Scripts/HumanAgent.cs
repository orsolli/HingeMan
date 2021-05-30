﻿using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

public class HumanAgent : Agent
{
    GameObject body;
    public float saveStateInterval = 1f;
    float saveStateTimer = 0;
    GameObject pendingBody;
    int initialPoseSize;
    public List<GameObject> startPoses = new List<GameObject>();
    int poseIndex = 0;
    float rotation_pct = 0;
    Transform head;
    private static int frames = 30;
    private Dictionary<string, Vector3> velocity;
    private Dictionary<string, Vector3> angular_velocity;
    private Dictionary<string, Vector3>[] acceleration = new Dictionary<string, Vector3>[frames];
    private Dictionary<string, Vector3>[] angular_acceleration = new Dictionary<string, Vector3>[frames];
    public float gracePeriod = 1f;
    public float graceTimer;
    Queue<float> pendingRewards = new Queue<float>();

    // Eyes
    Vector3 rightEye = new Vector3(0.0321f, 0f, 0.08f);
    Vector3 leftEye = new Vector3(-0.0321f, 0f, 0.08f);
    Vector3 focusPoint = Vector3.forward;
    Vector3 desired_acceleration = Vector3.zero;
    public override void Initialize()
    {
        initialPoseSize = startPoses.Count;
        try
        {
            if (transform.childCount > 0)
            {
                Destroy(transform.GetChild(0).gameObject);
            }
        }
        finally
        {

        }
    }
    private void CreateBody()
    {
        poseIndex = (poseIndex + 1) % startPoses.Count;
        body = Instantiate(startPoses[poseIndex], transform);
        body.SetActive(true);
        rotation_pct = ((360 + transform.eulerAngles.y) % 360) / 360;
        head = body.transform.Find("Head");
        focusPoint = blindFocus(head);

        velocity = new Dictionary<string, Vector3>();
        angular_velocity = new Dictionary<string, Vector3>();
        for (int i = 0; i < frames; i++)
        {
            acceleration[i] = new Dictionary<string, Vector3>();
            angular_acceleration[i] = new Dictionary<string, Vector3>();
            foreach (Rigidbody limb in body.GetComponentsInChildren<Rigidbody>())
            {
                acceleration[i][limb.gameObject.name] = Vector3.zero;
                angular_acceleration[i][limb.gameObject.name] = Vector3.zero;
                if (i == 0)
                {
                    velocity[limb.gameObject.name] = Vector3.zero;
                    angular_velocity[limb.gameObject.name] = Vector3.zero;
                }
            }
        }

    }


    private void saveState()
    {
        if (pendingBody != null)
        {
            int replaceIndex = poseIndex;
            if (replaceIndex < initialPoseSize * 2)
            {
                replaceIndex += initialPoseSize;
            }
            if (startPoses.Count > replaceIndex)
            {
                Destroy(startPoses[replaceIndex]);
                startPoses[replaceIndex] = pendingBody;
            }
            else
            {
                startPoses.Add(pendingBody);
            }
        }
        pendingBody = Instantiate(body, transform);
        pendingBody.SetActive(false);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Quaternion head_rotation = Quaternion.Inverse(head.rotation).normalized;
        foreach (HingeJoint limb in body.GetComponentsInChildren<HingeJoint>())
        {
            float range = limb.limits.max - limb.limits.min;
            JointSpring spring = limb.spring;
            sensor.AddObservation(2 * (limb.angle - limb.limits.min) / range - 1);
        }
        Vector3 relative_acc = head_rotation * desired_acceleration;
        sensor.AddObservation(Vector3.ClampMagnitude(relative_acc / 100f, 1f));
        Rigidbody rigidHead = head.gameObject.GetComponent<Rigidbody>();
        Vector3 avg_acc = Vector3.zero;
        Vector3 avg_angular_acc = Vector3.zero;
        for (int i = 0; i < frames; i++)
        {
            avg_acc += acceleration[i][head.gameObject.name] / frames;
            avg_angular_acc += angular_acceleration[i][head.gameObject.name] / frames;
        }
        avg_acc /= frames;
        avg_angular_acc /= frames;
        Vector3 total_acceleration = avg_acc - Physics.gravity;
        total_acceleration = head_rotation * total_acceleration;
        sensor.AddObservation(Vector3.ClampMagnitude(total_acceleration / 100f, 1f));
        Vector3 total_angular_acceleration = head_rotation * avg_angular_acc;
        sensor.AddObservation(Vector3.ClampMagnitude(total_angular_acceleration / 100f, 1f));
        Debug.DrawRay(head.position, head.rotation * total_acceleration, Color.red);
        Debug.DrawRay(head.position, head.rotation * total_angular_acceleration, Color.yellow);

        Vector3 absRightEye = head.position + head.rotation * rightEye;
        Vector3 absLeftEye = head.position + head.rotation * leftEye;
        Quaternion lookAtRight = Quaternion.LookRotation(focusPoint - absRightEye, head.rotation * Vector3.up);
        sensor.AddObservation(lookAtRight);
        Quaternion lookAtLeft = Quaternion.LookRotation(focusPoint - absLeftEye, head.rotation * Vector3.up);
        sensor.AddObservation(lookAtLeft);
    }

    public override void OnActionReceived(float[] act)
    {
        int action = 0;
        foreach (HingeJoint limb in body.GetComponentsInChildren<HingeJoint>())
        {
            float springAction = (Mathf.Clamp(act[action++], -1f, 1f) + 1f) / 2;
            AddReward(-Mathf.Pow(springAction, 2) * 0.02f); // full strength on all 21 gives ca -0.4f. (0.4 * strength / 21)
            float targetAction = (Mathf.Clamp(act[action++], -1f, 1f) + 1f) / 2;
            float range = limb.limits.max - limb.limits.min;
            JointSpring spring = limb.spring;
            spring.spring = springAction * 150;
            spring.targetPosition = targetAction * range + limb.limits.min;
            limb.spring = spring;

        }

        // Find focusPoint
        Vector3 lookAt = head.rotation * Vector3.forward;
        Ray ray = new Ray(head.position, lookAt);
        RaycastHit hit;

        if (Physics.Raycast(ray, out hit))
        {
            focusPoint = head.position + lookAt * hit.distance;
        }
        else
        {
            focusPoint = blindFocus(head);
        }

    }

    float deltaTimeCumulative = 0;
    void FixedUpdate()
    {
        if (body == null)
        {
            deltaTimeCumulative += Time.fixedDeltaTime;
            return;
        }
        float deltaTime = deltaTimeCumulative + Time.fixedDeltaTime;
        deltaTimeCumulative = 0;
        saveStateTimer += deltaTime;
        Vector3 massCenter = Vector3.zero;
        float mass = 0f;
        Vector3 avg_acceleration = Vector3.zero;
        float avg_acceleration_mag = 0;
        //Vector3 avg_ang_acceleration = Vector3.zero;
        Vector3 avg_velocity = Vector3.zero;
        int len = 0;
        int frame = StepCount % frames;
        foreach (Rigidbody part in body.GetComponentsInChildren<Rigidbody>())
        {
            Vector3 acc = (part.velocity - velocity[part.gameObject.name]) / deltaTime;
            Vector3 angular_acc = (part.angularVelocity - angular_velocity[part.gameObject.name]) / deltaTime;
            acceleration[frame][part.gameObject.name] = acc;
            angular_acceleration[frame][part.gameObject.name] = angular_acc;
            velocity[part.gameObject.name] = part.velocity;
            angular_velocity[part.gameObject.name] = part.angularVelocity;

            foreach (Dictionary<string, Vector3> prev_acc in acceleration)
            {
                avg_acceleration += prev_acc[part.gameObject.name] * part.mass / frames;
                avg_acceleration_mag += prev_acc[part.gameObject.name].magnitude * part.mass / frames;
            }
            //avg_ang_acceleration += angular_acc * part.mass;

            len++;
            avg_velocity += part.velocity * part.mass;

            massCenter += part.worldCenterOfMass * part.mass;
            mass += part.mass;
        }
        avg_acceleration /= mass;
        ////avg_ang_acceleration /= len * mass;
        avg_acceleration_mag /= len * mass;
        avg_velocity /= len * mass;
        massCenter /= mass;
        massCenter -= body.transform.position;

        Vector3 direction = transform.rotation * Vector3.forward;
        Vector3 desired_velocity = direction * rotation_pct;
        Vector3 corrected_direction = (desired_velocity - avg_velocity);
        float speed_diff = Mathf.Clamp01(corrected_direction.magnitude);
        float acc_mag = 10f * Mathf.Clamp01((transform.localPosition + head.localPosition).magnitude / 1000f) * speed_diff;
        desired_acceleration = desired_velocity.normalized * acc_mag - Physics.gravity;

        float reward = 0.2f;
        float progress = 0f;

        float moveLoss = -Mathf.Clamp01(avg_acceleration.magnitude - (Vector3.Dot(avg_acceleration, desired_acceleration + Physics.gravity) + 1f) / 2f);
        float directionLoss = -Mathf.Clamp01(-(Vector3.Dot(avg_velocity, desired_velocity) - avg_velocity.magnitude));
        Monitor.Log("Move", moveLoss, MonitorType.slider, head);
        Monitor.Log("Direction", directionLoss, MonitorType.slider, head);
        reward += moveLoss * 0.1f;
        reward += directionLoss * 0.8f;

        Vector3 des_acc_dir = desired_acceleration.normalized;
        Transform footR = body.transform.Find("RightFoot");
        Transform footL = body.transform.Find("LeftFoot");
        float lowerPoint = Mathf.Min(Vector3.Dot(footL.localPosition, des_acc_dir), Vector3.Dot(footR.localPosition, des_acc_dir));
        float footLoss = -1f + Mathf.Clamp01((Vector3.Dot(massCenter, des_acc_dir) - lowerPoint) / 0.8f); // (0.8 - 1.1)
        float headLoss = -1f + Mathf.Clamp01(Vector3.Dot(head.localPosition - massCenter, des_acc_dir) / 0.5f); // Acceptable range: (0.5 - 0.65)
        float heightLoss = (footLoss + headLoss) / 2;
        if (footLoss < -0.1f || headLoss < -0.1f)
        {
            graceTimer -= deltaTime * (1 + rotation_pct) * 20f / (head.localPosition.magnitude + transform.localPosition.magnitude);
            SetReward(-1);
            if (graceTimer < 0)
            {
                EndEpisode();
            }
            Monitor.Log(gameObject.name, -1f, MonitorType.slider, head);
            return;
        }
        else
        {
            if (graceTimer < gracePeriod)
            {
                SetReward(1);
                graceTimer = gracePeriod;
                Monitor.Log(gameObject.name, 1f, MonitorType.slider, head);
                return;
            }
            progress = Mathf.Clamp01(0.1f * Vector3.Dot(transform.localPosition + (footL.localPosition + footR.localPosition) / 2, direction) - 2);
            if (saveStateTimer > saveStateInterval)
            {
                saveStateTimer = 0;
                saveState();
            }
        }
        reward += progress * 0.8f;
        Monitor.Log("Position", progress, MonitorType.slider, head);
        if (rotation_pct < 0.01f)
        {
            reward = 0.2f - avg_acceleration_mag;
        }
        reward = Mathf.Clamp(reward, -1f, 1f);
        if (reward > 0)
        {
            pendingRewards.Enqueue(reward);
        }
        else
        {
            pendingRewards.Enqueue(0);
        }
        if (pendingRewards.Count > 1f / deltaTime)
        {
            pendingRewards.Dequeue();
        }
        AddReward(reward);
        Monitor.Log(gameObject.name, reward, MonitorType.slider, head);
        Debug.DrawRay(head.position, head.rotation * avg_acceleration - Physics.gravity, Color.red);
        Debug.DrawRay(head.position, head.rotation * avg_velocity, Color.green);

    }

    public override void OnEpisodeBegin()
    {
        pendingRewards.Clear();
        Destroy(pendingBody);
        pendingBody = null;
        saveStateTimer = 0;
        Destroy(body);
        CreateBody();
        focusPoint = blindFocus(head);
        graceTimer = gracePeriod;
    }

    public static Vector3 blindFocus(Transform head)
    {
        return head.position + head.rotation * Vector3.forward * 1000;
    }
}
