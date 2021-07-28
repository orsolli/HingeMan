using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Policies;

public class HumanAgent : Agent
{
    public GameObject body;
    public float maxSpeed = 10f;
    int initialPoseSize;
    public List<GameObject> startPoses = new List<GameObject>();
    public List<Dictionary<string, Vector3>> startVelocities = new List<Dictionary<string, Vector3>>();
    public List<Dictionary<string, Vector3>> startAngularVelocities = new List<Dictionary<string, Vector3>>();
    int poseIndex = 0;
    float rotation_pct = 0;
    Transform head;
    public static int frames = 15;
    private Dictionary<string, Vector3> velocity;
    private Dictionary<string, Vector3> angular_velocity;
    public Dictionary<string, Vector3>[] acceleration = new Dictionary<string, Vector3>[frames];
    public Dictionary<string, Vector3>[] angular_acceleration = new Dictionary<string, Vector3>[frames];

    // Eyes
    public Vector3 rightEye = new Vector3(0.0321f, 0f, 0.08f);
    public Vector3 leftEye = new Vector3(-0.0321f, 0f, 0.08f);
    public Vector3 focusPoint = Vector3.forward;
    public Vector3 desired_acceleration = -Physics.gravity;
    public Vector3 avg_velocity;
    float effort = 0f;
    enum Hinges
    {
        Head,
        NeckNo,
        NeckYes,
        Spine,
        Tale,
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
        body.SetActive(true);
        foreach (var b in body.GetComponentsInChildren<Rigidbody>())
        {
            b.ResetInertiaTensor();
        }
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

    private void observe(VectorSensor sensor, HingeJoint limb)
    {
        float range = limb.limits.max - limb.limits.min;
        sensor.AddObservation(2 * (limb.angle - limb.limits.min) / range - 1);
        sensor.AddObservation(Mathf.Clamp(limb.velocity / 2000f, -1, 1));
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        Quaternion head_rotation = Quaternion.Inverse(head.rotation).normalized;
        var hinges = body.GetComponentsInChildren<HingeJoint>();
        observe(sensor, hinges[(int)Hinges.Head]);
        observe(sensor, hinges[(int)Hinges.NeckNo]);
        observe(sensor, hinges[(int)Hinges.NeckYes]);
        observe(sensor, hinges[(int)Hinges.Spine]);
        observe(sensor, hinges[(int)Hinges.Tale]);
        observe(sensor, hinges[(int)Hinges.LeftSholder]);
        observe(sensor, hinges[(int)Hinges.LeftBicep]);
        observe(sensor, hinges[(int)Hinges.LeftArm]);
        observe(sensor, hinges[(int)Hinges.LeftElbow]);
        observe(sensor, hinges[(int)Hinges.RightPelvis]);
        observe(sensor, hinges[(int)Hinges.RightThigh]);
        observe(sensor, hinges[(int)Hinges.RightLeg]);
        observe(sensor, hinges[(int)Hinges.RightFoot]);
        observe(sensor, hinges[(int)Hinges.RightSholder]);
        observe(sensor, hinges[(int)Hinges.RightBicep]);
        observe(sensor, hinges[(int)Hinges.RightArm]);
        observe(sensor, hinges[(int)Hinges.RightElbow]);
        observe(sensor, hinges[(int)Hinges.LeftPelvis]);
        observe(sensor, hinges[(int)Hinges.LeftThigh]);
        observe(sensor, hinges[(int)Hinges.LeftLeg]);
        observe(sensor, hinges[(int)Hinges.LeftFoot]);

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

        Transform rightFoot = body.transform.Find("RightFoot");
        Quaternion rightFoot_rotation = Quaternion.Inverse(rightFoot.rotation).normalized;
        FootSensor rightFootSensor = rightFoot.GetComponent<FootSensor>();
        sensor.AddObservation(Vector3.ClampMagnitude(rightFoot_rotation * rightFootSensor.impulse / 20f, 1f));

        Transform leftFoot = body.transform.Find("LeftFoot");
        Quaternion leftFoot_rotation = Quaternion.Inverse(leftFoot.rotation).normalized;
        FootSensor leftFootSensor = leftFoot.GetComponent<FootSensor>();
        sensor.AddObservation(Vector3.ClampMagnitude(leftFoot_rotation * leftFootSensor.impulse / 20f, 1f));
    }

    private void act(float force, float angle, HingeJoint limb)
    {
        float springAction = (Mathf.Clamp(force, -1f, 1f) + 1f) / 2;
        effort = -Mathf.Clamp01(Mathf.Pow(springAction, 2));
        AddReward(effort * 0.00166f);
        float targetAction = (Mathf.Clamp(angle, -1f, 1f) + 1f) / 2;
        float range = limb.limits.max - limb.limits.min;
        JointSpring spring = limb.spring;
        spring.spring = springAction * 150;
        spring.targetPosition = targetAction * range + limb.limits.min;
        limb.spring = spring;
    }
    public override void OnActionReceived(float[] actions)
    {
        HingeJoint[] limb = body.GetComponentsInChildren<HingeJoint>();
        act(actions[(int)Hinges.Head * 2], actions[(int)Hinges.Head * 2 + 1], limb[(int)Hinges.Head]);
        act(actions[(int)Hinges.NeckNo * 2], actions[(int)Hinges.NeckNo * 2 + 1], limb[(int)Hinges.NeckNo]);
        act(actions[(int)Hinges.NeckYes * 2], actions[(int)Hinges.NeckYes * 2 + 1], limb[(int)Hinges.NeckYes]);
        act(actions[(int)Hinges.Spine * 2], actions[(int)Hinges.Spine * 2 + 1], limb[(int)Hinges.Spine]);
        act(actions[(int)Hinges.Tale * 2], actions[(int)Hinges.Tale * 2 + 1], limb[(int)Hinges.Tale]);
        act(actions[(int)Hinges.LeftSholder * 2], actions[(int)Hinges.LeftSholder * 2 + 1], limb[(int)Hinges.LeftSholder]);
        act(actions[(int)Hinges.LeftBicep * 2], actions[(int)Hinges.LeftBicep * 2 + 1], limb[(int)Hinges.LeftBicep]);
        act(actions[(int)Hinges.LeftArm * 2], actions[(int)Hinges.LeftArm * 2 + 1], limb[(int)Hinges.LeftArm]);
        act(actions[(int)Hinges.LeftElbow * 2], actions[(int)Hinges.LeftElbow * 2 + 1], limb[(int)Hinges.LeftElbow]);
        act(actions[(int)Hinges.RightPelvis * 2], actions[(int)Hinges.RightPelvis * 2 + 1], limb[(int)Hinges.RightPelvis]);
        act(actions[(int)Hinges.RightThigh * 2], actions[(int)Hinges.RightThigh * 2 + 1], limb[(int)Hinges.RightThigh]);
        act(actions[(int)Hinges.RightLeg * 2], actions[(int)Hinges.RightLeg * 2 + 1], limb[(int)Hinges.RightLeg]);
        act(actions[(int)Hinges.RightFoot * 2], actions[(int)Hinges.RightFoot * 2 + 1], limb[(int)Hinges.RightFoot]);
        act(actions[(int)Hinges.RightSholder * 2], actions[(int)Hinges.RightSholder * 2 + 1], limb[(int)Hinges.RightSholder]);
        act(actions[(int)Hinges.RightBicep * 2], actions[(int)Hinges.RightBicep * 2 + 1], limb[(int)Hinges.RightBicep]);
        act(actions[(int)Hinges.RightArm * 2], actions[(int)Hinges.RightArm * 2 + 1], limb[(int)Hinges.RightArm]);
        act(actions[(int)Hinges.RightElbow * 2], actions[(int)Hinges.RightElbow * 2 + 1], limb[(int)Hinges.RightElbow]);
        act(actions[(int)Hinges.LeftPelvis * 2], actions[(int)Hinges.LeftPelvis * 2 + 1], limb[(int)Hinges.LeftPelvis]);
        act(actions[(int)Hinges.LeftThigh * 2], actions[(int)Hinges.LeftThigh * 2 + 1], limb[(int)Hinges.LeftThigh]);
        act(actions[(int)Hinges.LeftLeg * 2], actions[(int)Hinges.LeftLeg * 2 + 1], limb[(int)Hinges.LeftLeg]);
        act(actions[(int)Hinges.LeftFoot * 2], actions[(int)Hinges.LeftFoot * 2 + 1], limb[(int)Hinges.LeftFoot]);

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
            SetReward(-0.5f);
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

            mass += part.mass;
        }
        avg_velocity /= len * mass;

        Vector3 direction = transform.rotation * Vector3.forward;
        Vector3 desired_position = direction * (1 + body.transform.position.magnitude);
        direction = (desired_position - body.transform.position).normalized;
        Debug.DrawRay(head.position, direction, Color.green);

        Vector3 desired_velocity = direction * Mathf.Pow(rotation_pct, 2) * maxSpeed;
        Vector3 corrected_direction = (desired_velocity - avg_velocity);
        float speed_diff = Mathf.Clamp01(corrected_direction.magnitude);
        desired_acceleration = corrected_direction - Physics.gravity;

        Monitor.RemoveAllValues(head);
        float reward = 0.007f;
        if (rotation_pct < 0.001f)
        {
            reward -= avg_velocity.magnitude * 0.667f;
            Monitor.Log("Velocity", -avg_velocity.magnitude, MonitorType.slider, head);
        }
        else if (corrected_direction.sqrMagnitude > 0.001f)
        {
            float velLoss = 1 - (Vector3.Dot(avg_velocity, corrected_direction) / corrected_direction.magnitude - Vector3.Cross(avg_velocity, corrected_direction).magnitude / corrected_direction.sqrMagnitude);
            velLoss = Mathf.Clamp01(velLoss);
            Monitor.Log("Velocity", -velLoss, MonitorType.slider, head);
            reward -= Mathf.Pow(velLoss, 2) * 0.00667f;
            Vector3 headStart = transform.position.normalized * Mathf.Pow(rotation_pct, 2) * 50;
            Vector3 desired_progress = transform.position + desired_velocity * StepCount * Time.fixedDeltaTime;
            if (desired_progress.magnitude > headStart.magnitude)
            {
                desired_progress = desired_progress - headStart;
                Debug.DrawRay(Vector3.zero, desired_progress, Color.cyan);
                if (head.position.magnitude < desired_progress.magnitude)
                {
                    Fall();
                    return;
                }
            }
            float progress = new Vector2(head.localPosition.x, head.localPosition.z).magnitude;
            if (progress < 0.6
             && body.transform.Find("RightFoot").GetComponent<Rigidbody>().velocity.magnitude < 0.03f
             && body.transform.Find("LeftFoot").GetComponent<Rigidbody>().velocity.magnitude < 0.03f)
            {
                reward -= 0.00067f;
                if (StepCount > 75 && progress < 0.6)
                {
                    Fall();
                }
            }
        }

        Monitor.Log("Effort", effort, MonitorType.slider, head);
        reward = Mathf.Clamp(reward + effort * 0.00166f, -0.06667f, 0.06667f);
        AddReward(reward);
        reward *= 15;
        Monitor.Log(reward.ToString("0.000000"), reward * 10, MonitorType.slider, head);
        Debug.DrawRay(head.position, avg_velocity, Color.green);
    }

    public void Fall()
    {
        SetReward(-1);
        EndEpisode();
        Destroy(body);
        CreateBody();
    }

    public override void OnEpisodeBegin()
    {
        focusPoint = blindFocus(head);
    }

    public static Vector3 blindFocus(Transform head)
    {
        return head.position + head.rotation * Vector3.forward * 1000;
    }
}
