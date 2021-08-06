using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using System;
using System.Collections.Generic;

public class FutureAgent : Agent
{
    public HumanAgent agent;
    Transform futureBody;
    Transform futureBodyParent;
    float[] action;
    Vector3 oldHeadPos;
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
    public override void Initialize()
    {
        futureBodyParent = transform.Find("Future");

    }
    private Action<object> AddObservations(VectorSensor sensor)
    {
        return observation => AddObservationCast(observation, sensor);
    }
    private Action<object> SaveObservation(List<float> observations)
    {
        return observation => observations.AddRange(AddObservationCast(observation));
    }
    public void AddObservationCast(object observation, VectorSensor sensor)
    {
        Type t = observation.GetType();
        if (t.Equals(typeof(float)))
            sensor.AddObservation((float)observation);
        else if (t.Equals(typeof(int)))
            sensor.AddObservation((int)observation);
        else if (t.Equals(typeof(Vector3)))
            sensor.AddObservation((Vector3)observation);
        else if (t.Equals(typeof(Vector2)))
            sensor.AddObservation((Vector2)observation);
        else if (t.Equals(typeof(IEnumerable<float>)))
            sensor.AddObservation((IEnumerable<float>)observation);
        else if (t.Equals(typeof(Quaternion)))
            sensor.AddObservation((Quaternion)observation);
        else if (t.Equals(typeof(bool)))
            sensor.AddObservation((bool)observation);
        else throw new Exception("Casting");
    }
    public IEnumerable<float> AddObservationCast(object observation)
    {
        Type t = observation.GetType();
        if (t.Equals(typeof(float)))
            return AddObservation((float)observation);
        if (t.Equals(typeof(int)))
            return AddObservation((int)observation);
        if (t.Equals(typeof(Vector3)))
            return AddObservation((Vector3)observation);
        if (t.Equals(typeof(Vector2)))
            return AddObservation((Vector2)observation);
        if (t.Equals(typeof(IEnumerable<float>)))
            return AddObservation((IEnumerable<float>)observation);
        if (t.Equals(typeof(Quaternion)))
            return AddObservation((Quaternion)observation);
        if (t.Equals(typeof(bool)))
            return AddObservation((bool)observation);
        throw new Exception("Casting");
    }
    public IEnumerable<float> AddObservation(float observation)
    {
        return new float[] { observation };
    }
    public IEnumerable<float> AddObservation(int observation)
    {
        return new float[] { (float)observation };
    }
    public IEnumerable<float> AddObservation(Vector3 observation)
    {
        return new float[] { observation.x, observation.y, observation.z };
    }
    public IEnumerable<float> AddObservation(Vector2 observation)
    {
        return new float[] { observation.x, observation.y };
    }
    public IEnumerable<float> AddObservation(IEnumerable<float> observation)
    {
        return observation;
    }
    public IEnumerable<float> AddObservation(Quaternion observation)
    {
        return new float[] { observation.x, observation.y, observation.z, observation.w };
    }
    public IEnumerable<float> AddObservation(bool observation)
    {
        return new float[] { observation ? 1f : 0f };
    }
    private void observe(Action<object> addObs, Transform bodyPart)
    {
        HingeJoint limb = bodyPart.GetComponent<HingeJoint>();
        float range = limb.limits.max - limb.limits.min;
        addObs(2 * (limb.angle - limb.limits.min) / range - 1);
        addObs(Mathf.Clamp(limb.velocity / 2000f, -1, 1));
    }
    private void observeAction(Action<object> addObs, Transform bodyPart)
    {
        HingeJoint limb = bodyPart.GetComponent<HingeJoint>();
        float range = limb.limits.max - limb.limits.min;
        JointSpring spring = limb.spring;
        addObs(2 * spring.spring / 150 - 1);
        addObs(2 * (spring.targetPosition - limb.limits.min) / range - 1);
    }
    private void observeVelocities(Action<object> addObs, Transform head, Quaternion head_rotation)
    {

        Vector3 total_acceleration = agent.avg_acceleration.acceleration - Physics.gravity;

        total_acceleration = head_rotation * total_acceleration;
        addObs(Vector3.ClampMagnitude(total_acceleration / 100f, 1f));

        Vector3 total_angular_acceleration = head_rotation * agent.avg_acceleration.angular_acceleration;
        addObs(Vector3.ClampMagnitude(total_angular_acceleration / 100f, 1f));

    }
    private void observeEyes(Action<object> addObs, Transform head, Quaternion head_rotation)
    {
        Vector3 focusPoint = Quaternion.Inverse(head_rotation) * (head_rotation * agent.focusPoint);

        Vector3 absRightEye = head.position + head.rotation * agent.rightEye;
        Quaternion lookAtRight = Quaternion.LookRotation(focusPoint - absRightEye, head.rotation * Vector3.up);
        addObs(lookAtRight);

        Vector3 absLeftEye = head.position + head.rotation * agent.leftEye;
        Quaternion lookAtLeft = Quaternion.LookRotation(focusPoint - absLeftEye, head.rotation * Vector3.up);
        addObs(lookAtLeft);
    }
    private void observeFoot(Action<object> addObs, Transform foot)
    {
        Quaternion foot_rotation = Quaternion.Inverse(foot.rotation).normalized;
        FootSensor footSensor = foot.GetComponent<FootSensor>();
        addObs(Vector3.ClampMagnitude(foot_rotation * footSensor.impulse / 20f, 1f));
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        CollectAllObservations(AddObservations(sensor));

        Transform body = agent.body.transform;
        Transform head = body.GetChild(((int)BodyPart.Head));
        observeAction(AddObservations(sensor), head);
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.NeckNo));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.NeckYes));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.Spine));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.Tale));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.LeftSholder));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.LeftBicep));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.LeftArm));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.LeftElbow));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.RightPelvis));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.RightThigh));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.RightLeg));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.RightFoot));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.RightSholder));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.RightBicep));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.RightArm));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.RightElbow));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.LeftPelvis));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.LeftThigh));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.LeftLeg));
        observeAction(AddObservations(sensor), body.GetChild((int)BodyPart.LeftFoot));
    }
    public void CollectAllObservations(Action<object> addObs)
    {
        Transform body = agent.body.transform;
        Transform head = body.GetChild(((int)BodyPart.Head));

        observe(addObs, head);
        observe(addObs, body.GetChild((int)BodyPart.NeckNo));
        observe(addObs, body.GetChild((int)BodyPart.NeckYes));
        observe(addObs, body.GetChild((int)BodyPart.Spine));
        observe(addObs, body.GetChild((int)BodyPart.Tale));
        observe(addObs, body.GetChild((int)BodyPart.LeftSholder));
        observe(addObs, body.GetChild((int)BodyPart.LeftBicep));
        observe(addObs, body.GetChild((int)BodyPart.LeftArm));
        observe(addObs, body.GetChild((int)BodyPart.LeftElbow));
        observe(addObs, body.GetChild((int)BodyPart.RightPelvis));
        observe(addObs, body.GetChild((int)BodyPart.RightThigh));
        observe(addObs, body.GetChild((int)BodyPart.RightLeg));
        observe(addObs, body.GetChild((int)BodyPart.RightFoot));
        observe(addObs, body.GetChild((int)BodyPart.RightSholder));
        observe(addObs, body.GetChild((int)BodyPart.RightBicep));
        observe(addObs, body.GetChild((int)BodyPart.RightArm));
        observe(addObs, body.GetChild((int)BodyPart.RightElbow));
        observe(addObs, body.GetChild((int)BodyPart.LeftPelvis));
        observe(addObs, body.GetChild((int)BodyPart.LeftThigh));
        observe(addObs, body.GetChild((int)BodyPart.LeftLeg));
        observe(addObs, body.GetChild((int)BodyPart.LeftFoot));

        Quaternion head_rotation = Quaternion.Inverse(head.rotation).normalized;

        observeVelocities(addObs, head, head_rotation);
        observeEyes(addObs, head, head_rotation);
        observeFoot(addObs, body.GetChild((int)BodyPart.RightFoot));
        observeFoot(addObs, body.GetChild((int)BodyPart.LeftFoot));
    }

    public override void OnActionReceived(float[] act)
    {
        action = act;
        if (futureBody != null)
            Destroy(futureBody.gameObject);
        if (futureBodyParent != null && futureBodyParent.gameObject.activeSelf)
        {
            futureBody = Instantiate(agent.body.transform, futureBodyParent);
            foreach (var collider in futureBody.GetComponentsInChildren<Collider>())
            {
                collider.enabled = false;
            }
            foreach (var rigidbody in futureBody.GetComponentsInChildren<Rigidbody>())
            {
                rigidbody.useGravity = false;
                rigidbody.mass = rigidbody.name == "Spine" ? 14 : 0.01f;
                rigidbody.drag = 0.01f;
                rigidbody.angularDrag = 0.01f;
                rigidbody.detectCollisions = false;
                rigidbody.solverIterations = 1;
                rigidbody.inertiaTensor = new Vector3(0.001f, 0.0001f, 0.001f);
            }
            int actionIndex = 0;
            foreach (var limb in futureBody.GetComponentsInChildren<HingeJoint>())
            {
                float angle = act[actionIndex++];
                float velocity = act[actionIndex++];
                float targetAction = (Mathf.Clamp(angle, -1f, 1f) + 1f) / 2;
                float range = limb.limits.max - limb.limits.min;
                JointSpring spring = limb.spring;
                spring.spring = 10000;
                spring.damper = 0;
                spring.targetPosition = targetAction * range + limb.limits.min;
                limb.spring = spring;
            }
            Debug.DrawRay(transform.position, transform.position + new Vector3(act[42], act[43], act[44]) * 100f, Color.green, 0.005f);
        }
    }

    void FixedUpdate()
    {
        if (agent.StepCount < StepCount)
        {
            if (futureBody != null)
                Destroy(futureBody.gameObject);
            SetReward(0);
            EndEpisode();
            return;
        }
        else if (agent.StepCount % 15 == 0)
        {
            float difference = 0;
            List<float> observations = new List<float>();
            CollectAllObservations(SaveObservation(observations));
            for (int i = 0; i < observations.Count; i++)
            {
                difference += Mathf.Pow(observations[i] - Mathf.Clamp(action[i], -1, 1), 2);
            }
            difference /= observations.Count;
            SetReward(-difference);
            Monitor.Log("Future", -difference, MonitorType.slider, agent.transform);
        }
        if (futureBody != null)
        {
            var oldSpine = agent.body.transform.GetChild((int)BodyPart.Spine);
            var futureSpine = futureBody.GetChild((int)BodyPart.Spine);
            futureSpine.SetPositionAndRotation(
                oldSpine.position,
                oldSpine.rotation
            );
            futureSpine.GetComponent<Rigidbody>().velocity = oldSpine.GetComponent<Rigidbody>().velocity;
        }
    }
}
