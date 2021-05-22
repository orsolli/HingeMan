using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

public class HumanAgent : Agent
{
    public GameObject bodyPrefab;
    GameObject body;
    float rotation_pct = 0;
    Transform head;
    private static int frames = 100;
    private Dictionary<string, Vector3> velocity;
    private Dictionary<string, Vector3> angular_velocity;
    private Dictionary<string, Vector3>[] acceleration = new Dictionary<string, Vector3>[frames];
    private Dictionary<string, Vector3>[] angular_acceleration = new Dictionary<string, Vector3>[frames];
    public float gracePeriod = 1f;
    public float graceTimer;

    // Eyes
    Vector3 rightEye = new Vector3(0.0321f, 0f, 0.08f);
    Vector3 leftEye = new Vector3(-0.0321f, 0f, 0.08f);
    Vector3 focusPoint = Vector3.forward;
    Vector3 desired_acceleration = Vector3.zero;
    public override void Initialize()
    {
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
        body = Instantiate(bodyPrefab, transform);
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
        for (int i = 0; i < frames; i++)
        {
            avg_acc += acceleration[i][head.gameObject.name] / frames;
            avg_angular_acc += angular_acceleration[i][head.gameObject.name] / frames;
        }
        avg_acc /= frames;
        avg_angular_acc /= frames;
        Vector3 total_acceleration = avg_acc - Physics.gravity;
        total_acceleration = head_rotation * total_acceleration;
        sensor.AddObservation(total_acceleration);
        Vector3 total_angular_acceleration = head_rotation * avg_angular_acc;
        sensor.AddObservation(total_angular_acceleration);
        Debug.DrawRay(head.position, head.rotation * total_acceleration, Color.red);
        Debug.DrawRay(head.position, head.rotation * total_angular_acceleration, Color.yellow);

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
        //Vector3 avg_ang_acceleration = Vector3.zero;
        Vector3 avg_velocity = Vector3.zero;
        int len = 0;
        int frame = StepCount % frames;
        foreach (Rigidbody part in body.GetComponentsInChildren<Rigidbody>())
        {
            Vector3 acc = (part.velocity - velocity[part.gameObject.name]) / Time.fixedDeltaTime;
            Vector3 angular_acc = (part.angularVelocity - angular_velocity[part.gameObject.name]) / Time.fixedDeltaTime;
            acceleration[frame][part.gameObject.name] = acc;
            angular_acceleration[frame][part.gameObject.name] = angular_acc;
            velocity[part.gameObject.name] = part.velocity;
            angular_velocity[part.gameObject.name] = part.angularVelocity;

            foreach (Dictionary<string, Vector3> prev_acc in acceleration)
            {
                avg_acceleration += prev_acc[part.gameObject.name] * part.mass / frames;
            }
            //avg_ang_acceleration += angular_acc * part.mass;

            len++;
            avg_velocity += part.velocity * part.mass;

            massCenter += part.worldCenterOfMass * part.mass;
            mass += part.mass;
        }
        avg_acceleration /= mass;
        ////avg_ang_acceleration /= len * mass;
        avg_velocity /= len * mass;
        massCenter /= mass;

        Vector3 direction = transform.rotation * Vector3.forward;
        Vector3 desired_velocity = direction * rotation_pct;
        Vector3 corrected_direction = (desired_velocity - avg_velocity);
        float speed_diff = Mathf.Clamp01(corrected_direction.magnitude);
        float acc_mag = 10f * Mathf.Clamp01((transform.position + head.position).magnitude / 1000f) * speed_diff;
        desired_acceleration = desired_velocity.normalized * acc_mag - Physics.gravity;

        float reward = 0.2f;
        float progress = 0f;

        float moveLoss = -Mathf.Clamp01((avg_acceleration - Physics.gravity - desired_acceleration).magnitude);
        float directionLoss = -Mathf.Clamp01(-(Vector3.Dot(avg_velocity, desired_velocity) - avg_velocity.magnitude) / 2);
        Monitor.Log("Move", moveLoss, MonitorType.slider, head);
        Monitor.Log("Direction", directionLoss, MonitorType.slider, head);
        reward += moveLoss * 0.1f;
        reward += directionLoss * 0.1f;

        Vector3 des_acc_dir = desired_acceleration.normalized;
        Transform footR = body.transform.Find("RightFoot");
        Transform footL = body.transform.Find("LeftFoot");
        float lowerPoint = Mathf.Min(Vector3.Dot(footL.position, des_acc_dir), Vector3.Dot(footR.position, des_acc_dir));
        float footLoss = -1f + Mathf.Clamp01((Vector3.Dot(massCenter, des_acc_dir) - lowerPoint) / 0.8f); // (0.8 - 1.1)
        float headLoss = -1f + Mathf.Clamp01(Vector3.Dot(head.position - massCenter, des_acc_dir) / 0.5f); // Acceptable range: (0.5 - 0.65)
        float heightLoss = (footLoss + headLoss) / 2;
        if (footLoss < -0.1f || headLoss < -0.1f)
        {
            graceTimer -= Time.fixedDeltaTime * (1 + rotation_pct) * 20f / (head.position.magnitude + transform.position.magnitude);
            reward = -0.1f;
            if (graceTimer < 0)
            {
                reward = 1f;
                AddReward(reward);
                EndEpisode();
                return;
            }
        }
        else
        {
            graceTimer = gracePeriod;
            progress = Mathf.Clamp01(0.1f * Vector3.Dot(body.transform.position + (footL.position + footR.position) / 2, direction) - 2);
        }
        reward += heightLoss * 0.1f;
        reward += progress * 0.8f;
        Monitor.Log("Position", progress, MonitorType.slider, head);
        Monitor.Log("Height", footLoss, MonitorType.slider, head);
        Monitor.Log("Head", headLoss, MonitorType.slider, head);
        reward = Mathf.Clamp(reward, -1f, 1f);
        AddReward(reward);
        Monitor.Log(gameObject.name, reward, MonitorType.slider, head);
        Debug.DrawRay(head.position, head.rotation * avg_acceleration - Physics.gravity, Color.red);
        Debug.DrawRay(head.position, head.rotation * avg_velocity, Color.green);

    }

    public override void OnEpisodeBegin()
    {
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
