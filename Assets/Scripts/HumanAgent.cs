using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

public class HumanAgent : Agent
{
    float rotation_pct = 0;
    Transform head;
    private Dictionary<GameObject, Vector3> velocity;
    private Dictionary<GameObject, Vector3> angular_velocity;
    private Dictionary<GameObject, Vector3>[] acceleration = new Dictionary<GameObject, Vector3>[5];
    private Dictionary<GameObject, Vector3>[] angular_acceleration = new Dictionary<GameObject, Vector3>[5];
    private int frame = 0;
    public int gracePeriod = 500;
    public int graceTimer;

    public GameObject clone;

    // Initial positions
    Dictionary<GameObject, Vector3> transformsPosition;
    Dictionary<GameObject, Quaternion> transformsRotation;

    // Eyes
    public Vector3 rightEye = new Vector3(0.0321f, 0f, 0.08f);
    public Vector3 leftEye = new Vector3(-0.0321f, 0f, 0.08f);
    public float height = 1.53f;
    Vector3 focusPoint = Vector3.forward;
    System.Random rand = new System.Random();
    public Vector3 desired_acceleration = Vector3.zero;
    public override void Initialize()
    {
        rotation_pct = ((360 + transform.eulerAngles.y) % 360) / 360;
        head = transform.Find("Head");
        focusPoint = blindFocus(head);

        transformsPosition = new Dictionary<GameObject, Vector3>();
        transformsRotation = new Dictionary<GameObject, Quaternion>();
        foreach (Transform limb in GetComponentsInChildren<Transform>())
        {
            transformsPosition[limb.gameObject] = limb.transform.position;
            transformsRotation[limb.gameObject] = limb.transform.rotation;
        }

        velocity = new Dictionary<GameObject, Vector3>();
        angular_velocity = new Dictionary<GameObject, Vector3>();
        for (int i = 0; i < 5; i++)
        {
            acceleration[i] = new Dictionary<GameObject, Vector3>();
            angular_acceleration[i] = new Dictionary<GameObject, Vector3>();
            foreach (Rigidbody limb in GetComponentsInChildren<Rigidbody>())
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

        /*grounded = new Dictionary<string, bool>();
		foreach (Collider limb in GetComponentsInChildren<Collider>())
		{
			grounded[limb.gameObject.name] = false;
		}*/

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Quaternion head_rotation = Quaternion.Inverse(head.rotation).normalized;
        foreach (Collider limb in GetComponentsInChildren<Collider>())
        {
            if (limb.gameObject.name != "Head")
            {
                Vector3 limbPos = limb.transform.position - head.position;
                sensor.AddObservation(head_rotation * limbPos);
                sensor.AddObservation(head_rotation * limb.transform.rotation);

                if (clone != null)
                {
                    Transform cloneLimb = clone.transform.Find(limb.gameObject.name);
                    cloneLimb.position = head_rotation * limbPos;
                    cloneLimb.rotation = head_rotation * limb.transform.rotation;
                }
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
        Debug.DrawRay(absRightEye, lookAtRight * Vector3.forward * 100, Color.red);
        Debug.DrawRay(absLeftEye, lookAtLeft * Vector3.forward * 100, Color.green);
    }

    public override void OnActionReceived(float[] act)
    {
        int action = 0;
        foreach (HingeJoint limb in GetComponentsInChildren<HingeJoint>())
        {
            act[action] = Mathf.Clamp(act[action], -1f, 1f);
            JointMotor motor = limb.motor;
            motor.targetVelocity = act[action] * 400;
            limb.motor = motor;
            action++;
        }

        // Find focusPoint
        Vector3 lookAt = head.rotation *
            Quaternion.AngleAxis(Mathf.Clamp(act[action++], -1f, 1f), Vector3.right) *
            Quaternion.AngleAxis(Mathf.Clamp(act[action++], -1f, 1f), Vector3.up) *
            Vector3.forward;
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
        foreach (Rigidbody part in GetComponentsInChildren<Rigidbody>())
        {
            Vector3 acc = (part.velocity - velocity[part.gameObject]) / Time.fixedDeltaTime;
            Vector3 angular_acc = (part.angularVelocity - angular_velocity[part.gameObject]) / Time.fixedDeltaTime;
            acceleration[frame][part.gameObject] = acc;
            angular_acceleration[frame][part.gameObject] = angular_acc;
            velocity[part.gameObject] = part.velocity;
            angular_velocity[part.gameObject] = part.angularVelocity;

            len++;
            avg_acceleration += acc * part.mass;

            avg_velocity += part.velocity * part.mass;

            massCenter += part.worldCenterOfMass * part.mass;
            mass += part.mass;
        }
        avg_acceleration /= len * mass;
        avg_velocity /= len * mass;
        massCenter /= mass;

        Vector3 direction = transform.rotation * Vector3.forward;
        Vector3 desired_velocity = direction * rotation_pct;
        Vector3 corrected_direction = (desired_velocity - avg_velocity);
        float speed_diff = Mathf.Clamp01(corrected_direction.magnitude);
        float acc_mag = 10f * Mathf.Clamp01((transform.position + head.position).magnitude / 100f) * speed_diff;
        desired_acceleration = corrected_direction.normalized * acc_mag - Physics.gravity;

        float reward = 0.1f;

        float moveLoss = -Mathf.Clamp01((avg_acceleration - Physics.gravity - desired_acceleration).magnitude);
        float directionLoss = -Mathf.Clamp01(-(Vector3.Dot(avg_velocity, desired_velocity) - avg_velocity.magnitude) / 2);
        Monitor.Log("Move", moveLoss, MonitorType.slider, head);
        Monitor.Log("Direction", directionLoss, MonitorType.slider, head);
        reward += moveLoss * 0.2f;
        reward += directionLoss * 0.1f;

        Transform footR = transform.Find("RightFoot");
        Transform footL = transform.Find("LeftFoot");
        float lowerPoint = Mathf.Min(Vector3.Dot(footL.position, desired_acceleration), Vector3.Dot(footR.position, desired_acceleration));
        float footLoss = -1f + Mathf.Clamp01((Vector3.Dot(massCenter, desired_acceleration) - lowerPoint) / 0.8f); // (1.1 - 0.8)
        float headLoss = -1f + Mathf.Clamp01(Vector3.Dot(head.position - massCenter, desired_acceleration) / 0.5f); // Acceptable range: (0.5 - 0.65)
        float heightLoss = (footLoss + headLoss) / 2;
        reward += heightLoss * 0.1f;
        if (footLoss < -0.1f || headLoss < -0.1f)
        {
            graceTimer -= 1 + (int)(gracePeriod / head.position.magnitude);
            reward += -0.1f;
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
        Monitor.Log("Height", footLoss, MonitorType.slider, head);
        Monitor.Log("Head", headLoss, MonitorType.slider, head);
        reward = Mathf.Clamp(reward, -1f, 1f);
        AddReward(reward);
        Monitor.Log(gameObject.name, reward, MonitorType.slider, head);

        Vector3 absRightEye = head.position + head.rotation * rightEye;
        Vector3 absLeftEye = head.position + head.rotation * leftEye;
        var lookAtRight = Quaternion.LookRotation(focusPoint - absRightEye, Vector3.up);
        var lookAtLeft = Quaternion.LookRotation(focusPoint - absLeftEye, Vector3.up);

        Debug.DrawRay(absRightEye, lookAtRight * Vector3.forward * 100, Color.red);
        Debug.DrawRay(absLeftEye, lookAtLeft * Vector3.forward * 100, Color.green);
    }

    public override void OnEpisodeBegin()
    {
        foreach (Transform limb in GetComponentsInChildren<Transform>())
        {
            limb.position = transformsPosition[limb.gameObject];
            limb.rotation = transformsRotation[limb.gameObject];
        }
        foreach (Rigidbody limb in GetComponentsInChildren<Rigidbody>())
        {
            limb.velocity = Vector3.zero;
        }
        focusPoint = blindFocus(head);
        graceTimer = gracePeriod;
    }

    public static Vector3 blindFocus(Transform head)
    {
        return head.position + head.rotation * Vector3.forward * 1000;
    }
}
