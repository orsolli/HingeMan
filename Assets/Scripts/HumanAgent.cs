using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class HumanAgent : Agent
{
    public GameObject body;
    public float reward;
    public float maxSpeed = 10f;
    public int frameOffset = -1;
    int initialPoseSize;
    public List<GameObject> startPoses = new List<GameObject>();
    public List<Dictionary<string, Vector3>> startVelocities = new List<Dictionary<string, Vector3>>();
    public List<Dictionary<string, Vector3>> startAngularVelocities = new List<Dictionary<string, Vector3>>();
    int poseIndex = 0;
    float speedFactor = 0;
    Transform head;
    // Eyes
    public Vector3 rightEye = new Vector3(0.0321f, 0f, 0.08f);
    public Vector3 leftEye = new Vector3(-0.0321f, 0f, 0.08f);
    public Vector3 focusPoint = Vector3.forward;
    public Vector3 desired_acceleration = -Physics.gravity;
    public Vector3 previous_velocity = Vector3.zero;
    public Vector3 angular_velocity = Vector3.zero;
    public Vector3 avg_velocity;

    public struct Acceleration
    {
        public Vector3 acceleration;
        public Vector3 angular_acceleration;
    };
    public Acceleration avg_acceleration;
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
    enum Limbs
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
            b.velocity = Random.insideUnitSphere * 5f;
            b.angularVelocity = Random.insideUnitSphere * 5f;
        }
        //speedFactor = ((360 + transform.eulerAngles.y) % 360) / 360;
        head = body.transform.Find("Head");
        focusPoint = blindFocus(head);
        avg_velocity = Vector3.zero;
        avg_acceleration = new HumanAgent.Acceleration()
        {
            acceleration = Vector3.zero,
            angular_acceleration = Vector3.zero
        };
        previous_velocity = Vector3.zero;
        angular_velocity = Vector3.zero;
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

    private void observe(VectorSensor sensor, HingeJoint limb)
    {
        float range = limb.limits.max - limb.limits.min;
        sensor.AddObservation(2 * (limb.angle - limb.limits.min) / range - 1);
        sensor.AddObservation(Mathf.Clamp(limb.velocity / 2000f, -1, 1));
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        frameOffset = StepCount % 15;
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
        Vector3 total_acceleration = avg_acceleration.acceleration - Physics.gravity;
        total_acceleration = head_rotation * total_acceleration;
        sensor.AddObservation(Vector3.ClampMagnitude(total_acceleration / 100f, 1f));
        Vector3 total_angular_acceleration = head_rotation * avg_acceleration.angular_acceleration;
        sensor.AddObservation(Vector3.ClampMagnitude(total_angular_acceleration / 100f, 1f));
        Debug.DrawRay(transform.position, head.rotation * total_acceleration / 10f, Color.red, 0.005f);
        Debug.DrawRay(transform.position, head.rotation * total_angular_acceleration, Color.yellow, 0.005f);

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
        effort += Mathf.Clamp01(Mathf.Pow(springAction, 2)) / 2;
        float mass = limb.transform.GetComponent<Rigidbody>().mass +
                     limb.connectedBody.mass;
        effort += Mathf.Clamp01(Mathf.Pow(0.00001f, 1 - Mathf.Pow(angle, 2))) / 2;
        float targetAction = (Mathf.Clamp(angle, -1f, 1f) + 1f) / 2;
        float range = limb.limits.max - limb.limits.min;
        JointSpring spring = limb.spring;
        spring.spring = springAction * 150;
        spring.targetPosition = targetAction * range + limb.limits.min;
        limb.spring = spring;
    }
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        ActionSegment<float> actions = actionBuffers.ContinuousActions;
        effort = 0;
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
            focusPoint = blindFocus(head);
        }
    }

    void FixedUpdate()
    {
        if ((StepCount - frameOffset + 1) % 15 > 0)
        {
            if (StepCount % 15 == 1)
            {
                body.transform.Find("RightFoot").GetComponent<FootSensor>().enabled = false;
                body.transform.Find("LeftFoot").GetComponent<FootSensor>().enabled = false;
            }
            return;
        }
        body.transform.Find("RightFoot").GetComponent<FootSensor>().enabled = true;
        body.transform.Find("LeftFoot").GetComponent<FootSensor>().enabled = true;

        Vector3 new_velocity = Vector3.zero;
        Vector3 new_angular_velocity = Vector3.zero;
        foreach (Rigidbody limb in body.GetComponentsInChildren<Rigidbody>())
        {
            new_velocity += limb.velocity * limb.mass;
            new_angular_velocity += limb.angularVelocity * limb.mass;
        }
        new_velocity /= 22 * 60;
        new_angular_velocity /= 22 * 60;
        avg_acceleration = new Acceleration()
        {
            acceleration = (new_velocity - avg_velocity) / Time.fixedDeltaTime,
            angular_acceleration = (new_angular_velocity - angular_velocity) / Time.fixedDeltaTime
        };
        Debug.DrawRay(body.transform.Find("Head").position, new_velocity * 10f, Color.blue, 0.05f);
        Debug.DrawRay(body.transform.Find("Head").position + new_velocity * 10f, avg_acceleration.acceleration, Color.red, 0.05f);
        avg_velocity = new_velocity;
        angular_velocity = new_angular_velocity;

        Monitor.RemoveAllValues(head);

        reward = 0.25f;

        float accLoss = ((avg_acceleration.acceleration - Physics.gravity) - desired_acceleration).magnitude / Mathf.Max(desired_acceleration.magnitude, 1f);
        accLoss *= 0.5f;
        accLoss = Mathf.Pow(Mathf.Clamp01(accLoss), 2);
        Monitor.Log("Acceleration:" + accLoss.ToString("0.0000"), -accLoss, MonitorType.slider, head);
        reward -= accLoss;
        Vector3 desired_velocity = desired_acceleration + Physics.gravity;
        float velLoss = (Vector3.ClampMagnitude(avg_velocity, 0.1f) - desired_velocity * 0.1f).magnitude / Mathf.Max(desired_velocity.magnitude, 0.1f);
        velLoss *= 0.5f;
        velLoss = Mathf.Pow(Mathf.Clamp01(velLoss), 2);
        Monitor.Log("Velocity:" + velLoss.ToString("0.0000"), -velLoss, MonitorType.slider, head);
        reward -= velLoss;

        effort /= 21;
        Monitor.Log("Effort", -effort, MonitorType.slider, transform);
        reward -= effort * 0.01f;
        reward = Mathf.Clamp(reward, -1, 1);
        AddReward(reward);
        Monitor.Log(reward.ToString("0.0000"), reward, MonitorType.slider, head);
        //Debug.DrawRay(head.position, avg_velocity * 100f, Color.blue, 0.005f);
    }

    public void Fall()
    {
        SetReward(0);
        EndEpisode();
        body.SetActive(false);
        Destroy(body);
        CreateBody();
    }

    bool rightStep = false;
    public void Stop()
    {
        EndEpisode();

        Vector3 sv = desired_acceleration + Physics.gravity * 0.8f;
        var rigidbodies = body.GetComponentsInChildren<Rigidbody>();
        Vector3 velocity = new Vector3(sv.x, sv.y, sv.z);
        rigidbodies[(int)Limbs.Head].velocity = velocity;
        rigidbodies[(int)Limbs.NeckNo].velocity = velocity;
        rigidbodies[(int)Limbs.NeckYes].velocity = velocity;
        rigidbodies[(int)Limbs.Spine].velocity = velocity;
        if (rightStep)
        {
            rigidbodies[(int)Limbs.RightFoot].velocity = Vector3.zero;
            Debug.DrawRay(rigidbodies[(int)Limbs.LeftFoot].position, velocity, Color.magenta, 0.1f);
            rigidbodies[(int)Limbs.LeftSholder].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.LeftBicep].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.LeftArm].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.LeftElbow].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.LeftPelvis].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.LeftThigh].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.LeftLeg].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.LeftFoot].velocity = 2 * velocity;
        }
        else
        {
            rigidbodies[(int)Limbs.LeftFoot].velocity = Vector3.zero;
            Debug.DrawRay(rigidbodies[(int)Limbs.RightFoot].position, velocity, Color.magenta, 0.1f);
            rigidbodies[(int)Limbs.RightPelvis].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.RightThigh].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.RightLeg].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.RightFoot].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.RightSholder].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.RightBicep].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.RightArm].velocity = 2 * velocity;
            rigidbodies[(int)Limbs.RightElbow].velocity = 2 * velocity;
        }
        rightStep = !rightStep;
    }

    public override void OnEpisodeBegin()
    {
        focusPoint = blindFocus(head);
        speedFactor = Academy.Instance.EnvironmentParameters.GetWithDefault("speed", 0f);
        Vector3 direction = transform.rotation * Vector3.forward;
        Vector3 desired_velocity = direction * speedFactor * maxSpeed;
        Debug.DrawRay(head.position, desired_velocity, Color.green, 0.005f);
        desired_acceleration = desired_velocity - Physics.gravity;
    }

    public static Vector3 blindFocus(Transform head)
    {
        return head.position + head.rotation * Vector3.forward * 1000;
    }
}
