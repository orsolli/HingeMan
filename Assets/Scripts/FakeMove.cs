using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FakeMove : MonoBehaviour
{
    float rotation_pct = 0;
    Transform head;
    // Start is called before the first frame update
    void Start()
    {
        head = transform.Find("Head");
        rotation_pct = ((360f + transform.eulerAngles.y) % 360f) / 360f;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        foreach (Rigidbody part in GetComponentsInChildren<Rigidbody>())
        {
            Vector3 direction = transform.rotation * Vector3.forward;
            Vector3 desired_velocity = direction * rotation_pct;
            Vector3 corrected_direction = (desired_velocity - part.velocity);
            float speed_diff = Mathf.Clamp01(corrected_direction.magnitude);
            float acc_mag = 10f * Mathf.Clamp01((transform.position + head.position).magnitude / 100f) * speed_diff;
            Vector3 desired_acceleration = corrected_direction.normalized * acc_mag - Physics.gravity;

            part.AddForce(desired_acceleration * part.mass);
        }
    }
}
