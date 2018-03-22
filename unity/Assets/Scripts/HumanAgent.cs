using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HumanAgent : Agent {

	public GameObject tracer;

	public float strength = 20;
	public int maxFailingSteps = 1024;
	public int failCounter = 0;
	public bool fell = false;
	public bool fallen = false;
	private float[] lastAct = new float[18];

    // Initial positions
    Dictionary<GameObject, Vector3> transformsPosition;
    Dictionary<GameObject, Quaternion> transformsRotation;

	public override void InitializeAgent()
    {
        transformsPosition = new Dictionary<GameObject, Vector3>();
        transformsRotation = new Dictionary<GameObject, Quaternion>();
		foreach (Transform limb in GetComponentsInChildren<Transform>())
		{
            transformsPosition[limb.gameObject] = limb.position;
            transformsRotation[limb.gameObject] = limb.rotation;
		}

	}

    public override List<float> CollectState()
    {
		List<float> states = new List<float>();

		foreach (Transform limb in GetComponentsInChildren<Transform>())
		{
			var characterJoint = limb.gameObject.GetComponent<CharacterJoint>();
			if (characterJoint != null) {
				var connectedLimb = characterJoint.connectedBody;
				var limbPos = limb.position - connectedLimb.position;
				limbPos /= limbPos.magnitude;
				states.Add(limbPos.x);
				states.Add(connectedLimb.velocity.x);
				states.Add(limbPos.y);
				states.Add(connectedLimb.velocity.y);
				states.Add(limbPos.z);
				states.Add(connectedLimb.velocity.z);
				//if (tracer != null)
				//Instantiate(tracer, limbPos + transform.position, Quaternion.Euler(connectedLimb.velocity.normalized));
			}
		}
		
        //Monitor.Log("States", states, MonitorType.hist, transform.Find("Head"));
		return states;
	}

    public override void AgentStep(float[] act)
    {
        Monitor.Log("Actions", act, MonitorType.hist, transform.Find("Head"));
		float effort = 0;
		int action = 0;
		foreach (Transform limb in GetComponentsInChildren<Transform>())
		{
			var characterJoint = limb.gameObject.GetComponent<CharacterJoint>();
			if (characterJoint != null) {
				var connectedLimb = characterJoint.connectedBody;

				var highTwistLimit = characterJoint.highTwistLimit.limit;	//	The upper limit around the primary axis of the character joint.
				var lowTwistLimit = characterJoint.lowTwistLimit.limit;	//	The lower limit around the primary axis of the character joint.
				var swing1Limit = characterJoint.swing1Limit.limit;	 	//	The angular limit of rotation (in degrees) around the primary axis of the character joint.
				var swing2Limit = characterJoint.swing2Limit.limit;	 	//	The angular limit of rotation (in degrees) around the primary axis of the character joint.
				if(highTwistLimit - lowTwistLimit > 1) {
            		act[action] = Mathf.Max(Mathf.Min(act[action], 1), -1);
					effort += Mathf.Pow(act[action], 2) > 0.5f ? Mathf.Pow(act[action], 4) : 0;

					var limbDir = Vector3.Cross(limb.worldToLocalMatrix.inverse * characterJoint.axis, limb.transform.up);
					var connectedLimbDir = Vector3.Cross(limb.worldToLocalMatrix.inverse * characterJoint.axis, connectedLimb.transform.up);

					limb.GetComponent<Rigidbody>().AddForceAtPosition(act[action] * strength *
						(limbDir),
						limb.worldToLocalMatrix.inverse.MultiplyPoint(characterJoint.anchor.normalized));
					
					limb.GetComponent<Rigidbody>().AddForceAtPosition(-act[action] * strength *
						(limbDir),
						limb.worldToLocalMatrix.inverse.MultiplyPoint(-characterJoint.anchor.normalized));

					connectedLimb.AddForceAtPosition(-act[action] * strength *
						(connectedLimbDir),
						connectedLimb.transform.worldToLocalMatrix.inverse.MultiplyPoint(characterJoint.anchor.normalized));

					connectedLimb.AddForceAtPosition(act[action] * strength *
						(connectedLimbDir),
						connectedLimb.transform.worldToLocalMatrix.inverse.MultiplyPoint(-characterJoint.anchor.normalized));
					/*
					if ((act[action] > 0.1f || act[action] < -0.1f) && tracer != null){
						Vector3 vec4 = (limbDir - connectedLimbDir);
						Instantiate(tracer, vec4 + limb.position, Quaternion.Euler(characterJoint.connectedBody.velocity.normalized));
					}*/

					action++;
				}
				if (swing1Limit > 1) {
            		act[action] = Mathf.Max(Mathf.Min(act[action], 1), -1);
					effort += Mathf.Pow(act[action], 2) > 0.5f ? Mathf.Pow(act[action], 4) : 0;

					var limbDir = Vector3.Cross(limb.worldToLocalMatrix.inverse * characterJoint.swingAxis, limb.transform.up);
					var connectedLimbDir = Vector3.Cross(limb.worldToLocalMatrix.inverse * characterJoint.swingAxis, connectedLimb.transform.up);

					limb.GetComponent<Rigidbody>().AddForceAtPosition(act[action] * strength *
						(limbDir),
						limb.worldToLocalMatrix.inverse.MultiplyPoint(characterJoint.anchor.normalized));
					
					limb.GetComponent<Rigidbody>().AddForceAtPosition(-act[action] * strength *
						(limbDir),
						limb.worldToLocalMatrix.inverse.MultiplyPoint(-characterJoint.anchor.normalized));

					connectedLimb.AddForceAtPosition(-act[action] * strength *
						(connectedLimbDir),
						connectedLimb.transform.worldToLocalMatrix.inverse.MultiplyPoint(characterJoint.anchor.normalized));

					connectedLimb.AddForceAtPosition(act[action] * strength *
						(connectedLimbDir),
						connectedLimb.transform.worldToLocalMatrix.inverse.MultiplyPoint(-characterJoint.anchor.normalized));


					action++;
				}
			}
		}
		effort /= action;
		//Debug.Log("effort:" + effort + " action:" + action + "totalEffort:" + effort * action);
		
		if (!done) {
			float[] rewards = new float[6];
			reward = 0.1f * Mathf.Pow(Mathf.Min(transform.Find("Head").position.y / 4.76f, 1), 2);
			rewards[1] = Mathf.Pow(Mathf.Min(transform.Find("Head").position.y / 4.76f, 1), 2);
			Vector3 dir = transform.Find("Head").GetComponent<Rigidbody>().velocity;
			reward += Vector3.Dot(dir, Vector3.up) < 0 ? -0.025f : 0.1f;
			rewards[2] = Vector3.Dot(dir, Vector3.up) < 0 ? -0.25f : 1f;
			reward -= 0.05f * effort;
			rewards[3] = -effort;
			float rewardDiff = 0;
			for (int i=0; i<18; i++) {
				rewardDiff += Mathf.Pow((lastAct[i]+1)/2 - (act[i]+1)/2, 2) / 18;
			}
			reward -= 0.05f * rewardDiff;
			rewards[4] = -rewardDiff;
			lastAct = act;
			
			float fallReward = 0;
			if (fell) {
				fallReward -= 0.025f;
				if (maxFailingSteps < failCounter) {
					done = true;
					failCounter = 0;
				} else {
					failCounter++;
					fallReward -= 0.975f * Mathf.Pow(failCounter / maxFailingSteps, 2);
				}
				fell = false; // Maybe you'll get up.
			} else {
				fallen = false; // Eey!! You got up!
				failCounter = 0;
			}
			reward += fallReward;
			rewards[5] = fallReward;
			rewards[0] = reward;
			Monitor.Log("Rewards", rewards, MonitorType.hist, transform.Find("Head"));
			reward = Mathf.Clamp(reward, -1, 1);
		}
        Monitor.Log("Reward", reward, MonitorType.slider, transform.Find("Head"));
	}

    public override void AgentReset()
    {
		foreach (Transform limb in GetComponentsInChildren<Transform>())
		{
            limb.position = transformsPosition[limb.gameObject];
            limb.rotation = transformsRotation[limb.gameObject];
			Rigidbody rigidbody = limb.gameObject.GetComponent<Rigidbody>();
			if(rigidbody) {
				rigidbody.velocity = Vector3.zero;
			}
		}
		fell = false;
		fallen = false;
	}

    public override void AgentOnDone()
    {
    }
}
