using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HumanAgent : Agent {

	//public GameObject tracer;
	Transform head;
	public float strength = 1000;
	//public int maxFailingSteps = 1024;
	//public int failCounter = 0;
	//public Dictionary<string, bool> grounded;
	//private float[] lastAct = new float[18];
	//public Vector3 sumNetForce = Vector3.zero;

	public float lowestHeight = 1.8f;
	public float highestHeight = 2.8f;

    // Initial positions
    Dictionary<GameObject, Vector3> transformsPosition;
    Dictionary<GameObject, Quaternion> transformsRotation;

	public override void InitializeAgent()
    {

		head = transform.Find("Head");
		
        transformsPosition = new Dictionary<GameObject, Vector3>();
        transformsRotation = new Dictionary<GameObject, Quaternion>();
		foreach (Transform limb in GetComponentsInChildren<Transform>())
		{
            transformsPosition[limb.gameObject] = limb.transform.position;
            transformsRotation[limb.gameObject] = limb.transform.rotation;
		}
		
		/*grounded = new Dictionary<string, bool>();
		foreach (Collider limb in GetComponentsInChildren<Collider>())
		{
			grounded[limb.gameObject.name] = false;
		}*/

	}

    public override List<float> CollectState()
    {
		List<float> states = new List<float>();

		foreach (Collider limb in GetComponentsInChildren<Collider>())
		{
			Vector3 limbPos = limb.transform.localPosition - head.localPosition;
			limbPos /= limbPos.magnitude != 0 ? limbPos.magnitude : 1;
			if (limb.gameObject.name != "Head") {
				states.Add(limbPos.x);
				states.Add(limbPos.y);
				states.Add(limbPos.z);
			}
			states.Add(limb.transform.localRotation.x);
			states.Add(limb.transform.localRotation.y);
			states.Add(limb.transform.localRotation.z);

			Vector3 floor = new Vector3(limb.transform.position.x, 0, limb.transform.position.z);
			states.Add(Vector3.Distance(limb.ClosestPointOnBounds(floor), floor) / 4.76f);
		}
		return states;
	}

    public override void AgentStep(float[] act)
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
		if (action > 0) {
			effort /= action;
		}
		//Debug.Log("effort:" + effort + " action:" + action + "totalEffort:" + effort * action);
		
        Monitor.Log("Actions", act, MonitorType.hist, head);
		if (!done) {
			/*float[] rewards = new float[6];
			reward = (head.localPosition.y - 3) / 1.76f;
			if (head.localPosition.y < 3)
				failCounter += 1;
			rewards[1] = reward;

			Vector3 dir = head.GetComponent<Rigidbody>().velocity;
			//reward -= Vector3.Dot(dir, Vector3.up) < 0 ? -0.025f : 0.1f;
			//rewards[2] = Vector3.Dot(dir, Vector3.up) < 0 ? -0.25f : 1f;

			reward -= effort;
			rewards[3] = -effort;

			float rewardDiff = 0;
			for (int i=0; i<18; i++) {
				rewardDiff += (Mathf.Max(Mathf.Pow((lastAct[i]+1)/2 - (act[i]+1)/2, 2), 0.5f) - 0.5f) / 18;
			}
			//reward -= 0.1f * rewardDiff;
			//rewards[4] = -rewardDiff;
			lastAct = act;
			
			float fallReward = 0;
			foreach (string entry in grounded.Keys) {
				fallReward -= grounded[entry] && !entry.Contains("Foot") ? 0.05f : 0;
				fallReward -= grounded[entry] && entry == "Head" ? 0.05f : 0;
				if (grounded[entry] && !entry.Contains("Foot")) {
					failCounter += 1;
					fallReward -= 0.4f * Mathf.Pow(failCounter / maxFailingSteps != 0 ? maxFailingSteps : 1 , 2);
				}
			}
			if (maxFailingSteps < failCounter) {
				done = true;
				failCounter = 0;
			}
			if (fallReward < 0.05f)
				failCounter = Mathf.Max(0, failCounter - 1);
			//reward += 0.075f * fallReward;
			//rewards[5] = fallReward;
			rewards[0] = reward;*/


			Vector3 massCenter = Vector3.zero;
				float mass = 0f;
				foreach (Rigidbody part in GetComponentsInChildren<Rigidbody>())
				{
					massCenter += part.worldCenterOfMass * part.mass;
					mass += part.mass;
				}
				massCenter /= mass;
			Vector2 projectedMassCenter = new Vector2(massCenter.x, massCenter.z);

			Vector3 touchCenter = Vector3.zero;
				Transform rFoot = transform.Find("RightFoot");
				Transform lFoot = transform.Find("LeftFoot");
				Vector3[] rFootVerts = rFoot.GetComponent<MeshFilter>().mesh.vertices;
				Vector3[] lFootVerts = lFoot.GetComponent<MeshFilter>().mesh.vertices;
				for (int i = 0; i < rFootVerts.Length; i++) {
					rFootVerts[i] = rFoot.transform.localToWorldMatrix.MultiplyPoint(rFootVerts[i]);
					//rFootVerts[i] += new Vector3(rFoot.transform.position.x + transform.position.x, -rFootVerts[i].y, rFoot.transform.position.z + transform.position.z);
				}
				for (int i = 0; i < lFootVerts.Length; i++) {
					lFootVerts[i] = lFoot.transform.localToWorldMatrix.MultiplyPoint(lFootVerts[i]);
					//lFootVerts[i] += new Vector3(lFoot.transform.position.x + transform.position.x, -rFootVerts[i].y, lFoot.transform.position.z + transform.position.z);
				}
				Vector3[] vertices = new Vector3[rFootVerts.Length + lFootVerts.Length];
				rFootVerts.CopyTo(vertices, 0);
				lFootVerts.CopyTo(vertices, rFootVerts.Length);
				//int L = head.GetComponent<MeshFilter>().mesh.vertices.Length;
				Vector2[] hull = bound(vertices);//new Vector2[vertices.Length];//
				#if false
					int[] triangles = new int[hull.Length*3];
					for (int i = 3; i < hull.Length; i++) {
						int j = (i-3)*3;
						triangles[j] = i-1;
						triangles[j+1] = 0;
						triangles[j+2] = i-2;

					}
					Vector3[] newVertices = new Vector3[hull.Length];
					for (int i = 0; i < hull.Length; i++) {
						newVertices[i] = head.transform.worldToLocalMatrix.MultiplyPoint(new Vector3(hull[i].x, 0, hull[i].y));
						
					}
					Mesh m = head.GetComponent<MeshFilter>().mesh;
					m.Clear();
					m.vertices = newVertices;
					m.triangles = triangles;
					m.uv = hull;
				#endif
				bool innafor = within(hull, projectedMassCenter);
				float distance = 0;
				if (!innafor) {
					int closest = 0;
					distance = Vector2.Distance(hull[0], projectedMassCenter);
					for (int i = 0; i < hull.Length; i++) {
						if (Vector2.Distance(hull[i], projectedMassCenter) < distance) {
							closest = i;
							distance = Vector2.Distance(hull[closest], projectedMassCenter);
						}
					}
				}
/*
				if (rFootTouch && lFootTouch) {
					touchCenter = (
						rFoot.position +
						lFoot.position
						) / 2;
				} else if (rFootTouch != lFootTouch) {
					touchCenter = rFootTouch ? rFoot.position : lFoot.position;
				}

			float feetRadius = Vector3.Distance(
				rFoot.position,
				lFoot.position); */
			float balanceLoss = Mathf.Pow(distance, 0.5f) * 2;
			float heightReward = ( Vector3.Dot(massCenter, Vector3.up) - lowestHeight ) / ( highestHeight - lowestHeight );
			reward = Mathf.Clamp(heightReward, -1, 1) - Mathf.Clamp(balanceLoss, 0, 2);
			if (heightReward < 0 || balanceLoss > 2) {
				done = true;
			}

        	//Monitor.Log("massCenter", heightReward, MonitorType.slider, head);

			//Monitor.Log("Rewards", rewards, MonitorType.hist, head);
        	//Debug.Log("Reward: " + reward + "heightReward: " + heightReward + "balanceLoss: " + balanceLoss);
			reward = Mathf.Clamp(reward, -1, 1);
		}
        Monitor.Log("Reward", reward, MonitorType.slider, head);
	}

    public override void AgentReset()
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

    public override void AgentOnDone()
    {
    }


	/**
	* Three points are a counter-clockwise turn if ccw > 0, clockwise if
	* ccw < 0, and collinear if ccw = 0 because ccw is a determinant that
	* gives twice the signed  area of the triangle formed by p1, p2 and p3.
	*/
	public static float ccw(Vector2 p1, Vector2 p2, Vector2 p3){
		return (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
	}

	public static Vector2[] bound(Vector3[] p) {
		int N = p.Length; //number of points
		Vector2[] points = new Vector2[N];
		int first = 0;
		for(int i = 0; i < N; i++) {
			int furthest = 0;
			float distance = Vector3.Distance(p[0], p[i]);
			for (int j = 0; j < p.Length; j++) {
				if (Vector3.Distance(p[j], p[i]) > distance) {
					furthest = j;
					distance = Vector3.Distance(p[furthest], p[i]);
				}
			}
			float height = p[i].y;
			float ratio = 1 - Mathf.Clamp(height, 0, distance)/distance;
			Vector2 fromFurthestToCurrent = new Vector2(p[i].x, p[i].z) - new Vector2(p[furthest].x, p[furthest].z);
			Vector2 shortened = fromFurthestToCurrent * ratio;
			Vector2 finishTransform = shortened + new Vector2(p[furthest].x, p[furthest].z);
			points[i] = finishTransform;
			if (points[first].x > points[i].x) {
				first = i;
			}
		}

		Vector2[] hull = new Vector2[points.Length];
		Vector2 pointOnHull = points[first]; // which is guaranteed to be part of the CH(S)
		Vector2 endpoint = Vector2.zero;

		// wrapped around to first hull point
		int L = 0;
		for(int i = 0; endpoint != hull[0] || i == 0; i++, pointOnHull = endpoint) {
			hull[i] = pointOnHull;
			endpoint = points[0];      // initial endpoint for a candidate edge on the hull
			for(int j = 1; j < hull.Length; j++) {
				if (endpoint == pointOnHull || ccw(points[j], hull[i], endpoint) > 0)
					endpoint = points[j];   // found greater left turn, update endpoint
			}
			L = i;
		}
		Vector2[] result = new Vector2[L];
		System.Array.Copy(hull, 0, result, 0, L);
		return result;
	}

	public static bool within(Vector2[] hull, Vector2 test) {
		int N = hull.Length;
		int i, j = 0;
		bool c = false;
		for (i = 0, j = N-1; i < N; j = i++) {
			if ( ((hull[i].y>test.y) != (hull[j].y>test.y)) &&
			(test.x < (hull[j].x-hull[i].x) * (test.y-hull[i].y) / (hull[j].y-hull[i].y) + hull[i].x) )
			c = !c;
		}
		return c;
	}
}
