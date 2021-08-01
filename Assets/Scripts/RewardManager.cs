using UnityEngine;
using System.Linq;

public class RewardManager : MonoBehaviour
{
    public Transform agents_parent;
    HumanAgent[] agents;
    public ComputeShader rewardShader;
    int kernelIndex;
    ComputeBuffer velocitiesBuffer;
    ComputeBuffer previous_velocitiesBuffer;
    ComputeBuffer accelerationsBuffer;
    ComputeBuffer avg_velocitiesBuffer;
    ComputeBuffer avg_accelerationsBuffer;

    struct Limb
    {
        public Vector3 Velocity;
        public Vector3 Angular_velocity;
        public float Mass;
    }

    void Start()
    {
        agents = agents_parent.GetComponentsInChildren<HumanAgent>();
        velocitiesBuffer = new ComputeBuffer(agents.Length * 22, 4 * 3 * 2 + 4);
        previous_velocitiesBuffer = new ComputeBuffer(agents.Length * 22, 4 * 3 * 2 + 4);
        accelerationsBuffer = new ComputeBuffer(agents.Length * 22, 4 * 3 * 2);
        avg_velocitiesBuffer = new ComputeBuffer(agents.Length * 22, 4 * 3);
        avg_accelerationsBuffer = new ComputeBuffer(agents.Length * 22, 4 * 3 * 2);
        if (rewardShader == null)
            rewardShader = Resources.Load<ComputeShader>("RewardShader");
        kernelIndex = rewardShader.FindKernel("HumanReward");
        rewardShader.SetBuffer(kernelIndex, "velocities", velocitiesBuffer);
        rewardShader.SetBuffer(kernelIndex, "previous_velocities", previous_velocitiesBuffer);
        rewardShader.SetBuffer(kernelIndex, "accelerations", accelerationsBuffer);
        rewardShader.SetBuffer(kernelIndex, "avg_velocities", avg_velocitiesBuffer);
        rewardShader.SetBuffer(kernelIndex, "avg_accelerations", avg_accelerationsBuffer);
        previous_velocitiesBuffer.SetData(Enumerable.Repeat(new Limb()
        {
            Velocity = Vector3.zero,
            Angular_velocity = Vector3.zero,
            Mass = 1
        }, 22 * agents.Length).ToArray());
    }
    void OnDestroy()
    {
        velocitiesBuffer.Dispose();
        previous_velocitiesBuffer.Dispose();
        accelerationsBuffer.Dispose();
        avg_velocitiesBuffer.Dispose();
        avg_accelerationsBuffer.Dispose();
    }
    float deltaTimeCumulative = 0;
    void FixedUpdate()
    {
        if (agents[0].StepCount % 15 == 0)
        {
            float deltaTime = deltaTimeCumulative + Time.fixedDeltaTime;
            deltaTimeCumulative = 0;
            Limb[] vels = new Limb[agents.Length * 22];
            int index = 0;
            foreach (HumanAgent agent in agents)
            {
                foreach (Rigidbody limb in agent.body.GetComponentsInChildren<Rigidbody>())
                {
                    vels[index++] = new Limb()
                    {
                        Velocity = limb.velocity,
                        Angular_velocity = limb.angularVelocity,
                        Mass = limb.mass
                    };
                }
            }
            velocitiesBuffer.SetData(vels);
            rewardShader.SetFloat("deltatime", deltaTime);
            rewardShader.Dispatch(kernelIndex, agents.Length, 1, 1);
            Vector3[] avg_velocities = new Vector3[agents.Length];
            HumanAgent.Acceleration[] avg_accelerations = new HumanAgent.Acceleration[agents.Length];
            avg_velocitiesBuffer.GetData(avg_velocities);
            avg_accelerationsBuffer.GetData(avg_accelerations);
            for (int i = 0; i < agents.Length; i++)
            {
                agents[i].avg_velocity = avg_velocities[i];
                agents[i].avg_acceleration = avg_accelerations[i];
            }
        }
        else
        {
            deltaTimeCumulative += Time.fixedDeltaTime;
        }
    }
}
