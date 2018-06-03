using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FloorIsLava : MonoBehaviour {

	void OnCollisionStay(Collision collisionInfo)
	{
		//if(!GameObject.Find(collisionInfo.gameObject.name).GetComponentsInParent<HumanAgent>()[0].grounded[collisionInfo.gameObject.name])
		//Debug.Log("Stay: " + collisionInfo.gameObject.name);
		foreach (var item in GameObject.Find(collisionInfo.transform.parent.name).GetComponentsInParent<HumanAgent>()) {
			item.grounded[collisionInfo.gameObject.name] = true;
		}
	}
	void OnCollisionExit(Collision collisionInfo)
	{
		//if(GameObject.Find(collisionInfo.gameObject.name).GetComponentsInParent<HumanAgent>()[0].grounded[collisionInfo.gameObject.name])
		//Debug.Log("Exit: " + collisionInfo.gameObject.name);
		foreach (var item in GameObject.Find(collisionInfo.transform.parent.name).GetComponentsInParent<HumanAgent>()) {
			item.grounded[collisionInfo.gameObject.name] = false;
		}
	}
}
