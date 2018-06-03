using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SelfDestruct : MonoBehaviour {

	private int i = 1;
	// Use this for initialization
	void Start () {
		i = 1;
	}
	void Update() {
		if (i < 0)
			Destroy(gameObject);
		i--;
	}
}
