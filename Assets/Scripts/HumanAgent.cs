using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

public class HumanAgent : Agent
{

    Transform head;
    private Dictionary<GameObject, Vector3> velocity;
    private Dictionary<GameObject, Vector3> angular_velocity;
    private Dictionary<GameObject, Vector3>[] acceleration = new Dictionary<GameObject, Vector3>[5];
    private Dictionary<GameObject, Vector3>[] angular_acceleration = new Dictionary<GameObject, Vector3>[5];
    private int frame = 0;
    public int gracePeriod = 500;
    private int graceTimer;

    public GameObject clone;

    // Initial positions
    Dictionary<GameObject, Vector3> transformsPosition;
    Dictionary<GameObject, Quaternion> transformsRotation;

    // Eyes
    Vector3 rightEye = new Vector3(0.1f, 0f, 0.25f);
    Vector3 leftEye = new Vector3(-0.1f, 0f, 0.25f);
    Vector3 focusPoint = Vector3.forward;
    System.Random rand = new System.Random();
    public override void Initialize()
    {

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
        /*
        Debug.DrawRay(Vector3.zero, total_acceleration);
        Debug.DrawRay(Vector3.zero, total_angular_acceleration, Color.red);
        Debug.DrawRay(head.position, head.position + avg_angular_acc, Color.red);
        */
        Vector3 absRightEye = head.position + head.rotation * rightEye;
        Vector3 absLeftEye = head.position + head.rotation * leftEye;
        var lookAtRight = Quaternion.LookRotation(focusPoint - absRightEye, Vector3.up);
        sensor.AddObservation(lookAtRight);
        var lookAtLeft = Quaternion.LookRotation(focusPoint - absLeftEye, Vector3.up);
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
        foreach (Rigidbody part in GetComponentsInChildren<Rigidbody>())
        {
            Vector3 acc = (part.velocity - velocity[part.gameObject]) / Time.fixedDeltaTime;
            Vector3 angular_acc = (part.angularVelocity - angular_velocity[part.gameObject]) / Time.fixedDeltaTime;
            acceleration[frame][part.gameObject] = acc;
            angular_acceleration[frame][part.gameObject] = angular_acc;
            frame = frame % 5;
            velocity[part.gameObject] = part.velocity;
            angular_velocity[part.gameObject] = part.angularVelocity;

            len++;
            len++;
            avg_acceleration += (
                acc +
                angular_acc
            ) * part.mass;
            //avg_velocity += part.velocity * part.mass;

            massCenter += part.worldCenterOfMass * part.mass;
            mass += part.mass;
        }
        avg_acceleration /= len;
        //avg_velocity /= len;
        massCenter /= mass;
        float reward = 0.1f;
        reward -= avg_acceleration.magnitude / (10f * Physics.gravity.magnitude);
        reward *= Mathf.Abs(reward);
        //reward -= 0.01f * avg_velocity.magnitude;
        Monitor.Log("Move", reward, MonitorType.slider, head);

        Transform footR = transform.Find("RightFoot");
        Transform footL = transform.Find("LeftFoot");
        float lowerPoint = Mathf.Min(footL.position.y, footR.position.y);
        float heightReward = (head.position.y - lowerPoint - 3.0f) / 3f;
        float headReward = (head.position.y - massCenter.y - 1.5f) / 1f;
        if (heightReward < -0.1f || headReward < -0.1f)
        {
            headReward /= 10;
            heightReward /= 10;
            reward = 0f;
            if (--graceTimer < 0)
            {
                reward -= 1f;
                EndEpisode();
            }
        }
        else if (graceTimer >= gracePeriod)
        {
            reward = reward * 0.1f + headReward;
        }
        else if (heightReward > 0.1f && headReward > 0.1f)
        {
            float redemption = 1f - (float)graceTimer / gracePeriod;
            reward += redemption;
            gracePeriod = (int)(gracePeriod * (1f + redemption));
            graceTimer = gracePeriod;
        }
        if ((focusPoint - head.position).magnitude > 900f)
        {
            reward -= 0.01f;
        }
        reward += Mathf.Min(heightReward, headReward);
        Monitor.Log("Height", heightReward, MonitorType.slider, head);
        Monitor.Log("Head", headReward, MonitorType.slider, head);
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
