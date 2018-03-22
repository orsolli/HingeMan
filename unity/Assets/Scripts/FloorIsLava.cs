using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FloorIsLava : MonoBehaviour {

	void OnCollisionStay(Collision collisionInfo)
	{
		if (!(collisionInfo.gameObject.name == "LeftFoot" || collisionInfo.gameObject.name == "RightFoot")) {
			collisionInfo.contacts[0].otherCollider.gameObject.GetComponentsInParent<HumanAgent>()[0].fell = true;
			collisionInfo.contacts[0].otherCollider.gameObject.GetComponentsInParent<HumanAgent>()[0].fallen = true;
			if (collisionInfo.gameObject.name == "Head") {
				collisionInfo.contacts[0].otherCollider.gameObject.GetComponentsInParent<HumanAgent>()[0].reward = -1;
				collisionInfo.contacts[0].otherCollider.gameObject.GetComponentsInParent<HumanAgent>()[0].done = true;
			}
		}
	}
}
