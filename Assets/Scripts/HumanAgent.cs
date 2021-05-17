﻿using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

public class HumanAgent : Agent
{
    public GameObject bodyPrefab;
    GameObject body;
    float rotation_pct = 0;
    Transform head;
    private Dictionary<GameObject, Vector3> velocity;
    private Dictionary<GameObject, Vector3> angular_velocity;
    private Dictionary<GameObject, Vector3>[] acceleration = new Dictionary<GameObject, Vector3>[5];
    private Dictionary<GameObject, Vector3>[] angular_acceleration = new Dictionary<GameObject, Vector3>[5];
    private int frame = 0;
    public int gracePeriod = 3;
    public float graceTimer;

    // Eyes
    Vector3 rightEye = new Vector3(0.0321f, 0f, 0.08f);
    Vector3 leftEye = new Vector3(-0.0321f, 0f, 0.08f);
    Vector3 focusPoint = Vector3.forward;
    Vector3 desired_acceleration = Vector3.zero;
    public override void Initialize()
    {
        body = Instantiate(bodyPrefab, transform);
        rotation_pct = ((360 + transform.eulerAngles.y) % 360) / 360;
        head = body.transform.Find("Head");
        focusPoint = blindFocus(head);

        velocity = new Dictionary<GameObject, Vector3>();
        angular_velocity = new Dictionary<GameObject, Vector3>();
        for (int i = 0; i < 5; i++)
        {
            acceleration[i] = new Dictionary<GameObject, Vector3>();
            angular_acceleration[i] = new Dictionary<GameObject, Vector3>();
            foreach (Rigidbody limb in body.GetComponentsInChildren<Rigidbody>())
            {
                acceleration[i][limb.gameObject] = Vector3.zero;
                angular_acceleration[i][limb.gameObject] = Vector3.zero;
                if (i == 0)
                {
                    velocity[limb.gameObject] = Vector3.zero;
                    angular_velocity[limb.gameObject] = Vector3.zero;
                }
            }
        }

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Quaternion head_rotation = Quaternion.Inverse(head.rotation).normalized;
        foreach (Collider limb in body.GetComponentsInChildren<Collider>())
        {
            if (limb.gameObject.name != "Head")
            {
                Vector3 limbPos = limb.transform.position - head.position;
                sensor.AddObservation(head_rotation * limbPos);
                sensor.AddObservation(head_rotation * limb.transform.rotation);
            }
        }
        Vector3 relative_acc = head_rotation * desired_acceleration;
        sensor.AddObservation(relative_acc);
        Rigidbody rigidHead = head.gameObject.GetComponent<Rigidbody>();
        Vector3 avg_acc = Vector3.zero;
        Vector3 avg_angular_acc = Vector3.zero;
        for (int i = 0; i < 5; i++)
        {
            avg_acc += acceleration[i][head.gameObject];
            avg_angular_acc += angular_acceleration[i][head.gameObject];
        }
        avg_acc /= 5;
        avg_angular_acc /= 5;
        Vector3 total_acceleration = avg_acc - Physics.gravity;
        total_acceleration = head_rotation * total_acceleration;
        sensor.AddObservation(total_acceleration);
        Vector3 total_angular_acceleration = head_rotation * avg_angular_acc;
        sensor.AddObservation(total_angular_acceleration);

        Vector3 absRightEye = head.position + head.rotation * rightEye;
        Vector3 absLeftEye = head.position + head.rotation * leftEye;
        Quaternion lookAtRight = Quaternion.LookRotation(focusPoint - absRightEye, Vector3.up);
        sensor.AddObservation(lookAtRight);
        Quaternion lookAtLeft = Quaternion.LookRotation(focusPoint - absLeftEye, Vector3.up);
        sensor.AddObservation(lookAtLeft);
    }

    public override void OnActionReceived(float[] act)
    {
        int action = 0;
        foreach (HingeJoint limb in body.GetComponentsInChildren<HingeJoint>())
        {
            act[action] = Mathf.Clamp(act[action], -1f, 1f);

            float range = limb.limits.max - limb.limits.min;
            JointSpring spring = limb.spring;
            //spring.spring = 1000 * limb.GetComponent<Rigidbody>().mass / 60;
            spring.targetPosition = (act[action] + 1f) / 2 * range + limb.limits.min;
            limb.spring = spring;
            action++;

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

    void FixedUpdate()
    {
        Vector3 massCenter = Vector3.zero;
        float mass = 0f;
        Vector3 avg_acceleration = Vector3.zero;
        Vector3 avg_velocity = Vector3.zero;
        int len = 0;
        frame = (frame + 1) % 5;
        foreach (Rigidbody part in body.GetComponentsInChildren<Rigidbody>())
        {
            Vector3 acc = (part.velocity - velocity[part.gameObject]) / Time.fixedDeltaTime;
            Vector3 angular_acc = (part.angularVelocity - angular_velocity[part.gameObject]) / Time.fixedDeltaTime;
            acceleration[frame][part.gameObject] = acc;
            angular_acceleration[frame][part.gameObject] = angular_acc;
            velocity[part.gameObject] = part.velocity;
            angular_velocity[part.gameObject] = part.angularVelocity;

            len++;
            avg_acceleration += acc * part.mass;
            avg_acceleration += angular_acc * part.mass;

            avg_velocity += part.velocity * part.mass;

            massCenter += part.worldCenterOfMass * part.mass;
            mass += part.mass;
        }
        avg_acceleration /= 2f * len * mass;
        avg_velocity /= len * mass;
        massCenter /= mass;

        Vector3 direction = transform.rotation * Vector3.forward;
        Vector3 desired_velocity = direction * rotation_pct;
        Vector3 corrected_direction = (desired_velocity - avg_velocity);
        float speed_diff = Mathf.Clamp01(corrected_direction.magnitude);
        float acc_mag = 10f * Mathf.Clamp01((transform.position + head.position).magnitude / 1000f) * speed_diff;
        desired_acceleration = desired_velocity.normalized * acc_mag - Physics.gravity;

        float reward = 0.2f;

        float moveLoss = -Mathf.Clamp01((avg_acceleration - Physics.gravity - desired_acceleration).magnitude);
        float directionLoss = -Mathf.Clamp01(-(Vector3.Dot(avg_velocity, desired_velocity) - avg_velocity.magnitude) / 2);
        Monitor.Log("Move", moveLoss, MonitorType.slider, head);
        Monitor.Log("Direction", directionLoss, MonitorType.slider, head);
        reward += moveLoss * 0.1f;
        reward += directionLoss * 0.1f;

        Transform footR = body.transform.Find("RightFoot");
        Transform footL = body.transform.Find("LeftFoot");
        float lowerPoint = Mathf.Min(Vector3.Dot(footL.position, desired_acceleration), Vector3.Dot(footR.position, desired_acceleration));
        float footLoss = -1f + Mathf.Clamp01((Vector3.Dot(massCenter, desired_acceleration) - lowerPoint) / 0.8f); // (0.8 - 1.1)
        float headLoss = -1f + Mathf.Clamp01(Vector3.Dot(head.position - massCenter, desired_acceleration) / 0.5f); // Acceptable range: (0.5 - 0.65)
        float heightLoss = (footLoss + headLoss) / 2;
        if (footLoss < -0.1f || headLoss < -0.1f)
        {
            graceTimer -= Time.fixedDeltaTime * 10f / (head.position.magnitude + transform.position.magnitude);
            reward = -0.1f;
            if (graceTimer < 0)
            {
                reward = 1f;
                AddReward(reward);
                EndEpisode();
                return;
            }
        }
        else if (footLoss > 0.1f && headLoss > 0.1f)
        {
            if (++graceTimer == gracePeriod)
            {
                reward = 1f;
            }
        }
        reward += heightLoss * 0.1f;
        Monitor.Log("Height", footLoss, MonitorType.slider, head);
        Monitor.Log("Head", headLoss, MonitorType.slider, head);
        reward = Mathf.Clamp(reward, -1f, 1f);
        AddReward(reward);
        Monitor.Log(gameObject.name, reward, MonitorType.slider, head);

    }

    public override void OnEpisodeBegin()
    {
        DestroyImmediate(body);
        Initialize();
        focusPoint = blindFocus(head);
        graceTimer = gracePeriod;
    }

    public static Vector3 blindFocus(Transform head)
    {
        return head.position + head.rotation * Vector3.forward * 1000;
    }
}
