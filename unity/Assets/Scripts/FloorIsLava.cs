using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FloorIsLava : MonoBehaviour {

	void OnCollisionEnter(Collision collisionInfo)
	{
		if (!(collisionInfo.gameObject.name == "LeftFoot" || collisionInfo.gameObject.name == "RightFoot")) {
			collisionInfo.contacts[0].otherCollider.gameObject.GetComponentsInParent<HumanAgent>()[0].done = true;
		}
	}
}
