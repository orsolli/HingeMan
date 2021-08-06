using UnityEngine;
using System.Linq;

public class RewardManager : MonoBehaviour
{
    public Transform agents_parent;
    HumanAgent[] agents;
    ComputeShader rewardShader;
    int kernelIndex;
    ComputeBuffer massBuffer;
    ComputeBuffer velocitiesBuffer;
    ComputeBuffer previous_velocitiesBuffer;
    ComputeBuffer resultsBuffer;

    struct Limb
    {
        public Vector3 Velocity;
        public Vector3 Angular_velocity;
        public float Mass;
    }

    void Start()
    {
        agents = agents_parent.GetComponentsInChildren<HumanAgent>();
        massBuffer = new ComputeBuffer(32, 4);
        velocitiesBuffer = new ComputeBuffer(agents.Length * 32, 4 * 3 * 2 * 2);
        previous_velocitiesBuffer = new ComputeBuffer(agents.Length * 32, 4 * 3 * 2 * 2);
        resultsBuffer = new ComputeBuffer(agents.Length, 4 * 3 * 2 * 2);
        if (rewardShader == null)
            rewardShader = Resources.Load<ComputeShader>("RewardShader");
        kernelIndex = rewardShader.FindKernel("HumanReward");
        rewardShader.SetBuffer(kernelIndex, "mass", massBuffer);
        rewardShader.SetBuffer(kernelIndex, "velocities", velocitiesBuffer);
        rewardShader.SetBuffer(kernelIndex, "previous_velocities", previous_velocitiesBuffer);
        rewardShader.SetBuffer(kernelIndex, "results", resultsBuffer);
        previous_velocitiesBuffer.SetData(Enumerable.Repeat(Vector3.zero, 64 * agents.Length).ToArray());
        float[] masses = new float[32];
        int index = 0;
        foreach (Rigidbody limb in agents[0].body.GetComponentsInChildren<Rigidbody>())
        {
            masses[index++] = limb.mass;
        }
        while (index < 32)
        {
            masses[index++] = 0f;
        }
        massBuffer.SetData(masses);
    }
    void OnDestroy()
    {
        velocitiesBuffer.Dispose();
        previous_velocitiesBuffer.Dispose();
        massBuffer.Dispose();
        resultsBuffer.Dispose();
    }
    float deltaTimeCumulative = 0;
    void FixedUpdate()
    {
        if (agents[0].StepCount % 15 == 0)
        {
            float deltaTime = deltaTimeCumulative + Time.fixedDeltaTime;
            deltaTimeCumulative = 0;
            Vector3[] vels = new Vector3[agents.Length * 64];
            int index = 0;
            foreach (HumanAgent agent in agents)
            {
                foreach (Rigidbody limb in agent.body.GetComponentsInChildren<Rigidbody>())
                {
                    vels[index++] = limb.velocity;
                    vels[index++] = limb.angularVelocity;
                }
                while (index % 64 > 0)
                {
                    vels[index++] = Vector3.zero;
                }
            }
            velocitiesBuffer.SetData(vels);
            rewardShader.Dispatch(kernelIndex, 1, 1, agents.Length);
            Vector3[,] results = new Vector3[agents.Length, 4];
            resultsBuffer.GetData(results);
            for (int i = 0; i < agents.Length; i++)
            {
                agents[i].avg_velocity = results[i, 0] / (22f * 60f);
                agents[i].avg_acceleration = new HumanAgent.Acceleration()
                {
                    acceleration = results[i, 1] / (22f * 60f * deltaTime),
                    angular_acceleration = results[i, 3] / (22f * 60f * deltaTime)
                };
            }
        }
        else
        {
            deltaTimeCumulative += Time.fixedDeltaTime;
        }
    }
}
