using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Policies;

public class HumanAgent : Agent
{
    GameObject body;
    public float maxSpeed = 10f;
    public float saveStateInterval = 1f;
    float saveStateTimer = 0;
    GameObject pendingBody;
    Dictionary<string, Vector3> pendingVelocities;
    Dictionary<string, Vector3> pendingAngularVelocities;
    int initialPoseSize;
    public List<GameObject> startPoses = new List<GameObject>();
    public List<Dictionary<string, Vector3>> startVelocities = new List<Dictionary<string, Vector3>>();
    public List<Dictionary<string, Vector3>> startAngularVelocities = new List<Dictionary<string, Vector3>>();
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
    Vector3 desired_acceleration = -Physics.gravity;
    public Vector3 avg_velocity;
    bool rightStep = true;
    public override void Initialize()
    {
        initialPoseSize = startPoses.Count;
        for (int i = 0; i < initialPoseSize; i++)
        {
            startVelocities.Add(getZeros(startPoses[poseIndex]));
            startAngularVelocities.Add(getZeros(startPoses[poseIndex]));
        }
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
        CreateBody();
    }
    private void CreateBody()
    {
        poseIndex = (poseIndex + 1) % startPoses.Count;
        body = Instantiate(startPoses[poseIndex], transform);
        for (int i = 14; i < 22 && i > 5; i += rightStep ? -1 : 1)
        {
            Vector3 sv = desired_acceleration + Physics.gravity;
            body.transform.GetChild(i).GetComponent<Rigidbody>().velocity = new Vector3(sv.x, sv.y, sv.z);
        }
        rightStep = !rightStep;
        body.SetActive(true);
        rotation_pct = ((360 + transform.eulerAngles.y) % 360) / 360;
        head = body.transform.Find("Head");
        focusPoint = blindFocus(head);

        velocity = getVelocities(body);
        angular_velocity = getAngularVelocities(body);
        for (int i = 0; i < frames; i++)
        {
            acceleration[i] = getZeros(body);
            angular_acceleration[i] = getZeros(body);
        }

    }

    private static Dictionary<string, Vector3> getZeros(GameObject body)
    {
        Dictionary<string, Vector3> dict = new Dictionary<string, Vector3>();
        foreach (Rigidbody limb in body.GetComponentsInChildren<Rigidbody>())
        {
            dict[limb.gameObject.name] = Vector3.zero;
        }
        return dict;
    }
    private static Dictionary<string, Vector3> getVelocities(GameObject body)
    {
        Dictionary<string, Vector3> dict = new Dictionary<string, Vector3>();
        foreach (Rigidbody limb in body.GetComponentsInChildren<Rigidbody>())
        {
            dict[limb.gameObject.name] = limb.velocity;
        }
        return dict;
    }
    private static Dictionary<string, Vector3> getAngularVelocities(GameObject body)
    {
        Dictionary<string, Vector3> dict = new Dictionary<string, Vector3>();
        foreach (Rigidbody limb in body.GetComponentsInChildren<Rigidbody>())
        {
            dict[limb.gameObject.name] = limb.angularVelocity;
        }
        return dict;
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
            //AddReward(-Mathf.Clamp01(-0.1f + Mathf.Pow(springAction, 2) * 0.02f)); // full strength on all 21 gives ca -0.3f.  -0.1 + (strength 0.4 * / 21)
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
            SetReward(-1);
            focusPoint = blindFocus(head);
        }

        // If in bad state, end episode before next action
        if (graceTimer < gracePeriod)
        {
            if (graceTimer > 0.105f)
            {
                graceTimer = 0.1f;
            }
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
        avg_velocity = Vector3.zero;
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


            len++;
            avg_velocity += part.velocity * part.mass;

            massCenter += part.worldCenterOfMass * part.mass;
            mass += part.mass;
        }
        avg_velocity /= len * mass;
        massCenter /= mass;
        massCenter -= body.transform.position;

        Vector3 direction = transform.rotation * Vector3.forward;
        Vector3 desired_velocity = direction * rotation_pct * maxSpeed;
        Vector3 corrected_direction = (desired_velocity - avg_velocity);
        float speed_diff = Mathf.Clamp01(corrected_direction.magnitude);
        desired_acceleration = desired_velocity - Physics.gravity;

        float reward = 0.00166f;
        float velLoss = desired_velocity.sqrMagnitude - (Vector3.Dot(avg_velocity, desired_velocity) - Vector3.Cross(avg_velocity, desired_velocity).magnitude);

        velLoss = Mathf.Clamp01(velLoss / Mathf.Max(desired_velocity.sqrMagnitude, 1f));
        Monitor.Log("Velocity", -velLoss, MonitorType.slider, head);
        reward -= Mathf.Pow(velLoss, 2) * 0.00667f;
        if (velLoss > 0.99f && avg_velocity.magnitude < 0.02f)
        {
            Stop();
        }

        Transform footR = body.transform.Find("RightFoot");
        Transform footL = body.transform.Find("LeftFoot");
        Vector3 position = (footL.position + footR.position) / 2;
        float progress = 1 - Mathf.Clamp01((transform.position + desired_velocity * 0.1f).magnitude - position.magnitude);
        reward += Mathf.Pow(progress, 2) * 0.00166f;
        Monitor.Log("Position", progress, MonitorType.slider, head);
        reward = Mathf.Clamp(reward, -0.06667f, 0.06667f);
        AddReward(reward);
        Monitor.Log(GetComponent<BehaviorParameters>().BehaviorName, reward * 15, MonitorType.slider, head);
        Debug.DrawRay(head.position, avg_velocity, Color.green);

    }

    public void Fall(string name)
    {
        if (rotation_pct > 0.88f)
        {
            Debug.Log($"Fall({name})");
        }
        SetReward(-1);
        EndEpisode();
        Destroy(body);
        CreateBody();
    }
    public void Stop()
    {
        if (rotation_pct > 0.88f)
        {
            Debug.Log("Stop");
        }
        SetReward(-1);
        EndEpisode();
        for (int i = 14; i < 22 && i > 5; i += rightStep ? -1 : 1)
        {
            Vector3 sv = desired_acceleration + Physics.gravity;
            body.transform.GetChild(i).GetComponent<Rigidbody>().velocity = new Vector3(sv.x, sv.y, sv.z);
        }
        rightStep = !rightStep;
    }

    public override void OnEpisodeBegin()
    {
        focusPoint = blindFocus(head);
        graceTimer = gracePeriod;
        saveStateTimer = 0;
    }

    public static Vector3 blindFocus(Transform head)
    {
        return head.position + head.rotation * Vector3.forward * 1000;
    }
}
