using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HumanAgent : Agent {

	//public GameObject tracer;

	public float strength = 20;
	public int maxFailingSteps = 1024;

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
				//Instantiate(tracer, limbPos + connectedLimb.position, Quaternion.Euler(connectedLimb.velocity.normalized));
			}
		}
		
		return states;
	}

    public override void AgentStep(float[] act)
    {
		int i = 0;
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
					limb.GetComponent<Rigidbody>().AddRelativeTorque(act[i++] * strength * characterJoint.axis);
				}
				if (swing1Limit > 1) {
					limb.GetComponent<Rigidbody>().AddRelativeTorque(act[i++] * strength * characterJoint.swingAxis);
				}
			}
		}

		if (!done) {
			reward = Mathf.Min(transform.Find("Head").position.y / 2f - 1.5f, 1);

			
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
	}

    public override void AgentOnDone()
    {
    }
}
