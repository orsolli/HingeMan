using UnityEngine;
using Unity.MLAgents;

public class IncreaseGravity : MonoBehaviour
{
    public int timeToFullGravity = 1000000;
    // Update is called once per frame
    void Start()
    {
        Academy.Instance.AgentPreStep += UpdateGravity;
    }
    public void UpdateGravity(int stepCount)
    {
        Physics.gravity = new Vector3(0, -9.81f, 0) * Mathf.Clamp01(0.16f + 1000f * (stepCount / 1000) / timeToFullGravity);
    }
}
