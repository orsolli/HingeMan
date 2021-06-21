using UnityEngine;
using Unity.MLAgents;

public class FloorIsLava : MonoBehaviour
{
    public void OnCollisionEnter(Collision other)
    {
        if (!other.gameObject.name.Contains("Foot") && !other.gameObject.name.Contains("Leg"))
        {
            other.gameObject.GetComponentInParent<HumanAgent>().Fall(other.gameObject.name);
        }
    }
}
