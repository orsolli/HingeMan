using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FootSensor : MonoBehaviour
{
    public Vector3 impulse = Vector3.zero;
    Vector3 prevVel = Vector3.zero;
    public void OnCollisionEnter(Collision other)
    {
        OnCollisionStay(other);
    }
    public void OnCollisionStay(Collision other)
    {
        if (other.transform.name.Equals("LeftLeg") || other.transform.name.Equals("RightLeg"))
            return;
        int sign = Vector3.Dot(other.GetContact(0).point - transform.position, other.impulse) > 0 ? -1 : 1;
        impulse = sign * other.impulse / Time.fixedDeltaTime / 60 + other.relativeVelocity - prevVel;
        prevVel = other.relativeVelocity;
        Debug.DrawRay(transform.position, other.relativeVelocity - prevVel, Color.blue);
        Debug.DrawRay(transform.position, impulse / 20f, Color.red);
        Debug.DrawRay(transform.position, Vector3.ClampMagnitude(impulse / 20f, 1f), Color.green);
    }
    public void OnCollisionExit(Collision other)
    {
        impulse = Vector3.zero;
        prevVel = Vector3.zero;
    }
}


