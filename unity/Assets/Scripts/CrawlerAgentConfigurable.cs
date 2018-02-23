using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CrawlerAgentConfigurable: Agent
{

    public float strength;

    public bool[] leg_touching;

    public bool fell = false;

    Transform body;
    Transform jesus;


    public Transform[] limbs;



    // Initial positions
    Dictionary<GameObject, Vector3> transformsPosition;
    Dictionary<GameObject, Quaternion> transformsRotation;

    //public static Dictionary<string, float> maxValues = new Dictionary<string, float>();
    //public static Dictionary<string, float> minValues = new Dictionary<string, float>();


    public override void InitializeAgent()
    {

        body = transform.Find("Sphere");
        jesus = transform.Find("Jesus");

        transformsPosition = new Dictionary<GameObject, Vector3>();
        transformsRotation = new Dictionary<GameObject, Quaternion>();

        Transform[] allChildren = GetComponentsInChildren<Transform>();
        foreach (Transform child in allChildren)
        {
            transformsPosition[child.gameObject] = child.position;
            transformsRotation[child.gameObject] = child.rotation;
        }


        leg_touching = new bool[8];
        for (int i = 0; i < leg_touching.Length; i++)
        {
            leg_touching[i] = true;
        }
    }

    public override List<float> CollectState()
    {
        List<float> state = new List<float>();
        state.Add(body.transform.rotation.eulerAngles.x/180.0f-1.0f);
        state.Add(body.transform.rotation.eulerAngles.y/180.0f-1.0f);
        state.Add(body.transform.rotation.eulerAngles.z/180.0f-1.0f);

        state.Add(body.gameObject.GetComponent<Rigidbody>().velocity.x / 10f);
        state.Add(body.gameObject.GetComponent<Rigidbody>().velocity.y / 10f);
        state.Add(body.gameObject.GetComponent<Rigidbody>().velocity.z / 10f);


        Vector3 dir = jesus.transform.position;
        // Joystick
        state.Add(dir.x / dir.magnitude);//X
        state.Add(dir.y / dir.magnitude);//Y
        state.Add(dir.z / dir.magnitude);//Z

        foreach (Transform t in limbs)
        {
            state.Add(t.localPosition.x / 10f);
            state.Add(t.localPosition.y / 10f);
            state.Add(t.localPosition.z / 10f);
            state.Add(t.localRotation.x);
            state.Add(t.localRotation.y);
            state.Add(t.localRotation.z);
            state.Add(t.localRotation.w);
            Rigidbody rb = t.gameObject.GetComponent < Rigidbody >();
            state.Add(rb.velocity.x);
            state.Add(rb.velocity.y);
            state.Add(rb.velocity.z);
            state.Add(rb.angularVelocity.x / 30f);
            state.Add(rb.angularVelocity.y / 30f);
            state.Add(rb.angularVelocity.z / 30f);
        }

        for (int index = 0; index < leg_touching.Length; index++)
        {
            if (leg_touching[index])
            {
                state.Add(1.0f);
            }
            else
            {
                state.Add(0.0f);
            }
        }

        return state;
    }

    public override void AgentStep(float[] act)
    {
        float onSteroids = 0;
        for (int k = 0; k < act.Length; k++)
        {
            act[k] = Mathf.Max(Mathf.Min(act[k], 1), -1);
            onSteroids += act[k] * act[k];
        }
        onSteroids /= act.Length;

        for (int i = 0; i < limbs.Length; i++) {
            limbs[i].gameObject.GetComponent<Rigidbody>().AddTorque(-limbs[i].transform.right * strength * act[i]);
        }

        limbs[0].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[16]);
        limbs[1].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[17]);
        limbs[2].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[18]);
        limbs[3].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[19]);
        limbs[4].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[20]);
        limbs[5].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[21]);
        limbs[6].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[22]);
        limbs[7].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[23]);


        if (!done)
        {
            Vector3 velocity = body.gameObject.GetComponent<Rigidbody>().velocity;
            Vector3 dir = jesus.transform.position / jesus.transform.position.magnitude;
            float dot = (0
                + velocity.x * dir.x
                + velocity.y * dir.y
                + velocity.z * dir.z
            );
            float speed = body.gameObject.GetComponent<Rigidbody>().velocity.magnitude;
            float onMyWay = dot / speed;

            float onMyFeet = 0;
            Vector3 lastPoint = new Vector3(0,0,0);
            Vector3 firstPoint = lastPoint;
            bool init = false;
            for (int i = 0; i < leg_touching.Length; i++)
            {
                if (leg_touching[i]) {
                    if (init)  {
                        onMyFeet += calculateVolume(
                            limbs[i+8].position,
                            lastPoint,
                            body.transform.position,
                            new Vector3(body.transform.position.x, lastPoint.y, body.transform.position.z));
                    } else {
                        firstPoint = limbs[i+8].position;
                        init = true;
                    }
                    lastPoint = limbs[i+8].position;
                }
            }
            if (init)  {
                onMyFeet += calculateVolume(
                    firstPoint,
                    lastPoint,
                    body.transform.position,
                    new Vector3(body.transform.position.x, lastPoint.y, body.transform.position.z));
            }
            
            onMyFeet /= 12f;
            if (!(onMyFeet > -1 && onMyFeet < 2)) {
                onMyFeet = 1;
            }

            reward = 0;
            if (onMyWay > 0.8f) {
                reward += dot > 10 ? 0.1f : 0;
            } else if (onMyWay < -0.5f) {
                reward -= 0.1f;
            } else {
                reward -= 0.05f;                
            }

            //reward -= 0.005f * (1.0f - onMyFeet);
            //reward -= 0.0001f * onSteroids;
            //reward += Mathf.Pow(onMyWay,3) * Mathf.Pow(0.1f * Mathf.Clamp(speed, 0f, 10f), 3);
            
            if (fell || onMyFeet < 0.1f && onMyWay < -0.8f) {
                reward = -1f;
                done = true;
                fell = false;
            }
        }

        Monitor.Log("Reward", reward, MonitorType.slider, body.gameObject.transform);

    }

    public override void AgentReset()
    {
        
        for (int i = 0; i < leg_touching.Length; i++){
            leg_touching[i] = true;
        }
        Transform[] allChildren = GetComponentsInChildren<Transform>();
        foreach (Transform child in allChildren)
        {
            if ((child.gameObject.name.Contains("Spider"))
                || (child.gameObject.name.Contains("parent")))
            {
                continue;
            }
            child.position = transformsPosition[child.gameObject];
            child.rotation = transformsRotation[child.gameObject];
            Rigidbody rigBod = child.gameObject.GetComponent<Rigidbody>();
            if (rigBod){
                rigBod.velocity = default(Vector3);
                rigBod.angularVelocity = default(Vector3);
            }
        }
        transform.rotation = Quaternion.Euler(new Vector3(0, Random.value * 180, 0));
        jesus.transform.position = transform.position + new Vector3(Random.value * 10 - 5, 0, Random.value * 10 - 5);
    }

    public override void AgentOnDone()
    {

    }

    private float calculateVolume(Vector3 a, Vector3 b, Vector3 c, Vector3 d) {
        float s = ((a-b).magnitude + (a-c).magnitude + (c-b).magnitude) / 2;
        float area = Mathf.Pow(s
            *(s-(a-b).magnitude)
            *(s-(a-c).magnitude)
            *(s-(c-b).magnitude), 0.5f);
        Vector3 areaNorm = Vector3.Normalize(Vector3.Cross(b - a, c - a));
        float distance = Vector3.Dot(areaNorm, d - a) / areaNorm.magnitude;
        return area * distance / 3f;
    }

}
