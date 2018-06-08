using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HumanAgent : Agent {

	public GameObject tracer;
	Transform head;
	public float strength = 1000;
	public int maxFailingSteps = 1024;
	public int failCounter = 0;
	public Dictionary<string, bool> grounded;
	//private float[] lastAct = new float[18];
	public Vector3 sumNetForce = Vector3.zero;

	public float lowestHeight = 2;
	public float highestHeight = 3;

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
		
		grounded = new Dictionary<string, bool>();
		foreach (Collider limb in GetComponentsInChildren<Collider>())
		{
			grounded[limb.gameObject.name] = false;
		}

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

			Vector3 touchCenter = Vector3.zero;
				Transform rFoot = transform.Find("RightFoot");
				Transform lFoot = transform.Find("LeftFoot");
				bool rFootTouch = isTouchingFloor(rFoot.GetComponent<Collider>());
				bool lFootTouch = isTouchingFloor(lFoot.GetComponent<Collider>());
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
				lFoot.position);
			float balanceLoss = 1;
			if (touchCenter != Vector3.zero) {
				balanceLoss = Vector3.Distance(touchCenter, new Vector3(massCenter.x, 0, massCenter.z)) / feetRadius;
			}
			float heightReward = ( Vector3.Dot(massCenter, Vector3.up) - lowestHeight ) / ( highestHeight - lowestHeight );
			reward = Mathf.Clamp(heightReward, -1, 1) - balanceLoss;
			if (heightReward < 0) {
				done = true;
			}

        	//Monitor.Log("massCenter", heightReward, MonitorType.slider, head);

			//Monitor.Log("Rewards", rewards, MonitorType.hist, head);
        	Debug.Log("Reward: " + reward + "heightReward: " + heightReward + "balanceLoss: " + balanceLoss);
			reward = Mathf.Clamp(reward, -1, 1);
		}
        Monitor.Log("Reward", reward, MonitorType.slider, head);
	}

    public override void AgentReset()
    {
		foreach (Collider limb in GetComponentsInChildren<Collider>())
		{
			grounded[limb.gameObject.name] = false;
		}
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

	public static bool isTouchingFloor(Collider a, float floorHeight = 0) {
		Vector3 floor = new Vector3(a.transform.position.x, floorHeight, a.transform.position.z);
		Vector3 closest = a.ClosestPointOnBounds(floor);
		return Vector3.Distance(closest, floor) < 0.01f;
	}
}
