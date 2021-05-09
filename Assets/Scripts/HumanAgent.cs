using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

public class HumanAgent : Agent
{

    //public GameObject tracer;
    Transform head;
    public float strength = 1000;
    //public int maxFailingSteps = 1024;
    //public int failCounter = 0;
    //public Dictionary<string, bool> grounded;
    //private float[] lastAct = new float[18];
    //public Vector3 sumNetForce = Vector3.zero;
    private Dictionary<GameObject, Vector3> velocity;
    private Dictionary<GameObject, Vector3> angular_velocity;
    private Dictionary<GameObject, Vector3>[] acceleration = new Dictionary<GameObject, Vector3>[5];
    private Dictionary<GameObject, Vector3>[] angular_acceleration = new Dictionary<GameObject, Vector3>[5];
    private int frame = 0;

    public float lowestHeight = 1.8f;
    public float highestHeight = 2.8f;

    public GameObject clone;

    // Initial positions
    Dictionary<GameObject, Vector3> transformsPosition;
    Dictionary<GameObject, Quaternion> transformsRotation;

    public override void Initialize()
    {

        head = transform.Find("Head");

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

    }

    public override void OnActionReceived(float[] act)
    {
        float effort = 0;
        int action = 0;
        foreach (HingeJoint limb in GetComponentsInChildren<HingeJoint>())
        {
            act[action] = Mathf.Clamp(act[action], -1f, 1f);
            effort += Mathf.Pow(act[action], 2);
            JointMotor motor = limb.motor;
            motor.targetVelocity = act[action] * strength;
            limb.motor = motor;
            action++;
        }

        // Collect reward
        if (action > 0)
        {
            effort /= action;
        }
        //Debug.Log("effort:" + effort + " action:" + action + "totalEffort:" + effort * action);


    }

    void FixedUpdate()
    {
        Vector3 massCenter = Vector3.zero;
        float mass = 0f;
        float avg_acceleration = 0f;
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
                acc.magnitude +
                angular_acc.magnitude
            ) * part.mass;
            avg_velocity += part.velocity * part.mass;

            massCenter += part.worldCenterOfMass * part.mass;
            mass += part.mass;
        }
        avg_acceleration /= len;
        avg_velocity /= len;
        massCenter /= mass;
        float reward = 1 - avg_acceleration / (100 * Physics.gravity.magnitude) - avg_velocity.magnitude; // Max 10G

        reward = Mathf.Clamp(reward, -1, 1);
        float heightReward = (Vector3.Dot(massCenter, Vector3.up) - lowestHeight) / (highestHeight - lowestHeight);
        if (heightReward < 0)
        {
            AddReward(-1f);
            EndEpisode();
        }
        else
        {
            AddReward(reward);
        }
        Monitor.Log("Reward", reward, MonitorType.slider, head);
    }

    public override void OnEpisodeBegin()
    {
        /*foreach (Collider limb in GetComponentsInChildren<Collider>())
		{
			grounded[limb.gameObject.name] = false;
		}*/
        foreach (Transform limb in GetComponentsInChildren<Transform>())
        {
            limb.position = transformsPosition[limb.gameObject];
            limb.rotation = transformsRotation[limb.gameObject];
        }
        foreach (Rigidbody limb in GetComponentsInChildren<Rigidbody>())
        {
            limb.velocity = Vector3.zero;
        }
    }


    /**
	* Three points are a counter-clockwise turn if ccw > 0, clockwise if
	* ccw < 0, and collinear if ccw = 0 because ccw is a determinant that
	* gives twice the signed  area of the triangle formed by p1, p2 and p3.
	*/
    public static float ccw(Vector2 p1, Vector2 p2, Vector2 p3)
    {
        return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
    }

    public static Vector2[] bound(Vector3[] p)
    {
        int N = p.Length; //number of points
        Vector2[] points = new Vector2[N];
        int first = 0;
        for (int i = 0; i < N; i++)
        {
            int furthest = 0;
            float distance = Vector3.Distance(p[0], p[i]);
            for (int j = 0; j < p.Length; j++)
            {
                if (Vector3.Distance(p[j], p[i]) > distance)
                {
                    furthest = j;
                    distance = Vector3.Distance(p[furthest], p[i]);
                }
            }
            float height = p[i].y;
            float ratio = 1 - Mathf.Clamp(height, 0, distance) / distance;
            Vector2 fromFurthestToCurrent = new Vector2(p[i].x, p[i].z) - new Vector2(p[furthest].x, p[furthest].z);
            Vector2 shortened = fromFurthestToCurrent * ratio;
            Vector2 finishTransform = shortened + new Vector2(p[furthest].x, p[furthest].z);
            points[i] = finishTransform;
            if (points[first].x > points[i].x)
            {
                first = i;
            }
        }

        Vector2[] hull = new Vector2[points.Length];
        Vector2 pointOnHull = points[first]; // which is guaranteed to be part of the CH(S)
        Vector2 endpoint = Vector2.zero;

        // wrapped around to first hull point
        int L = 0;
        for (int i = 0; endpoint != hull[0] || i == 0; i++, pointOnHull = endpoint)
        {
            hull[i] = pointOnHull;
            endpoint = points[0];      // initial endpoint for a candidate edge on the hull
            for (int j = 1; j < hull.Length; j++)
            {
                if (endpoint == pointOnHull || ccw(points[j], hull[i], endpoint) > 0)
                    endpoint = points[j];   // found greater left turn, update endpoint
            }
            L = i;
        }
        Vector2[] result = new Vector2[L];
        System.Array.Copy(hull, 0, result, 0, L);
        return result;
    }

    public static bool within(Vector2[] hull, Vector2 test)
    {
        int N = hull.Length;
        int i, j = 0;
        bool c = false;
        for (i = 0, j = N - 1; i < N; j = i++)
        {
            if (((hull[i].y > test.y) != (hull[j].y > test.y)) &&
            (test.x < (hull[j].x - hull[i].x) * (test.y - hull[i].y) / (hull[j].y - hull[i].y) + hull[i].x))
                c = !c;
        }
        return c;
    }
}
