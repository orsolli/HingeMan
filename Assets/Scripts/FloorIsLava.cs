using UnityEngine;
using Unity.MLAgents;

public class FloorIsLava : MonoBehaviour
{
    public void OnCollisionStay(Collision other)
    {
        if (!other.gameObject.name.Contains("Foot"))
        {
            other.gameObject.GetComponentInParent<HumanAgent>().Fall(other.gameObject.name);
        }
    }
}
