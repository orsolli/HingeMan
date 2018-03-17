using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CrawlerAgentConfigurable: Agent
{

    public float strength;

    public bool[] leg_touching;
    public bool flirting = false;
    bool[] leg_waslifted;
    Vector3[] leg_contrib;

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
        leg_waslifted = new bool[8];
        leg_contrib = new Vector3[8];
        for (int i = 0; i < leg_touching.Length; i++)
        {
            leg_touching[i] = true;
            leg_contrib[i] = body.transform.position;
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


        Vector3 dir = jesus.transform.localPosition / jesus.transform.localPosition.magnitude;
        // Joystick
        state.Add(dir.x);//X
        state.Add(dir.y);//Y
        state.Add(dir.z);//Z

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
            if (i < 8) {
                body.gameObject.GetComponent<Rigidbody>().AddTorque(limbs[i].transform.right * strength * act[i]);
            } else {
                limbs[i-8].gameObject.GetComponent<Rigidbody>().AddTorque(limbs[i].transform.right * strength * act[i]);
            }
        }

        limbs[0].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[16]);
        limbs[1].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[17]);
        limbs[2].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[18]);
        limbs[3].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[19]);
        limbs[4].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[20]);
        limbs[5].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[21]);
        limbs[6].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[22]);
        limbs[7].gameObject.GetComponent<Rigidbody>().AddTorque(-body.transform.up * strength * act[23]);

        body.gameObject.GetComponent<Rigidbody>().AddTorque(body.transform.up * strength * act[16]);
        body.gameObject.GetComponent<Rigidbody>().AddTorque(body.transform.up * strength * act[17]);
        body.gameObject.GetComponent<Rigidbody>().AddTorque(body.transform.up * strength * act[18]);
        body.gameObject.GetComponent<Rigidbody>().AddTorque(body.transform.up * strength * act[19]);
        body.gameObject.GetComponent<Rigidbody>().AddTorque(body.transform.up * strength * act[20]);
        body.gameObject.GetComponent<Rigidbody>().AddTorque(body.transform.up * strength * act[21]);
        body.gameObject.GetComponent<Rigidbody>().AddTorque(body.transform.up * strength * act[22]);
        body.gameObject.GetComponent<Rigidbody>().AddTorque(body.transform.up * strength * act[23]);


        if (!done)
        {
            Vector3 velocity = body.gameObject.GetComponent<Rigidbody>().velocity;
            //Vector3 dir = jesus.transform.localPosition / jesus.transform.localPosition.magnitude;
            float dis = /*Vector3.Dot(*/body.gameObject.transform.localPosition.magnitude;//, dir);
/*          float dot = (0
                + velocity.x * dir.x
                + velocity.y * dir.y
                + velocity.z * dir.z
            );
            float speed = velocity.magnitude;
            float onMyWay = dot / Mathf.Max(speed,1);
/*
            float onMyFeet = 0;
            Vector3 lastPoint = new Vector3(0,0,0);
            Vector3 firstPoint = lastPoint;
            bool init = false;
            for (int i = 0; i < leg_touching.Length; i++)
            {
                if (leg_touching[i]) {
                    if (init)  {
                        onMyFeet += calculateVolume(
                            limbs[i+8].localPosition,
                            lastPoint,
                            body.transform.localPosition,
                            new Vector3(body.transform.localPosition.x, lastPoint.y, body.transform.localPosition.z));
                    } else {
                        firstPoint = limbs[i+8].localPosition;
                        init = true;
                    }
                    lastPoint = limbs[i+8].localPosition;
                }
            }
            if (init)  {
                onMyFeet += calculateVolume(
                    firstPoint,
                    lastPoint,
                    body.transform.localPosition,
                    new Vector3(body.transform.localPosition.x, lastPoint.y, body.transform.localPosition.z));
            }
            
            onMyFeet /= 12f;
            if (!(onMyFeet > -1 && onMyFeet < 2)) {
                onMyFeet = 1;
            }
*/
            int legsTouch = 0;
            float contrib = 0;
            for (int i = 0; i < leg_touching.Length; i++) {
                if (leg_touching[i]) {
                    if (leg_waslifted[i]) {
                        leg_contrib[i] = body.transform.position;
                    } else {
                        Vector3 travers = body.transform.position - leg_contrib[i];
                        if ( travers.magnitude != 0) {
                            contrib += Vector3.Dot(travers, velocity) / Mathf.Max(travers.magnitude, 1);
                        }
                    }
                    leg_waslifted[i] = false;
                    legsTouch++;
                } else {
                    if (!leg_waslifted[i]) {
                        Vector3 travers = body.transform.position - leg_contrib[i];
                        if ( travers.magnitude != 0) {
                            contrib += Vector3.Dot(travers, velocity) / Mathf.Max(travers.magnitude, 1);
                        }
                        legsTouch++;
                    }
                    leg_waslifted[i] = true;
                }
            }

            reward = 0;
            if (legsTouch > 0) {
                reward += contrib / legsTouch;
            }
            //if (onMyWay > 0.9f) {
            //reward += 0.25f * onMyWay;//(dot > 1 ? onMyWay : 0.1f * onMyWay) * 0.2f;// + 0.05f * Mathf.Clamp(dis, 0, 10);              
            //}
            bool overHead = false;
            for (int i = 8; i<16; i++) {
                float diff = limbs[i].transform.localPosition.y - body.transform.localPosition.y;
                if (diff > 2) {
                    overHead = true;
                } else if (diff > 0) {
                    reward -= 0.05f * diff / 8;
                }
            }

            float balance = Vector3.Dot(body.transform.up, Vector3.up);
            //if (balance < 0.7f) {
            reward -= 0.5f - 0.5f * balance*balance;
            //}
            //reward -= 0.005f * (1.0f - onMyFeet);
            reward -= 0.001f * onSteroids;
            //reward += Mathf.Pow(onMyWay,3) * Mathf.Pow(0.1f * Mathf.Clamp(speed, 0f, 10f), 3);
            //reward += Mathf.Clamp(dis, 0, 10) * 0.01f;
            reward -= flirting ? 0.1f : 0;
            if (fell || balance < 0.2f) {
                reward = -1 + Mathf.Pow(Mathf.Clamp(dis, 0, 100) * 0.01f, 2);
                done = true;
                fell = false;
                //Debug.Log(fell ? "fell" : balance < 0.25f ? "tipped" : flirting ? "lowfive" : "highfive");
            }
        }
        Monitor.Log("Reward", reward, MonitorType.slider, body.gameObject.transform);

    }

    public override void AgentReset()
    {
        flirting = false;
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
        //transform.rotation = Quaternion.Euler(new Vector3(0, Random.value * 180, 0));
        //jesus.transform.position = transform.position + new Vector3(Random.value * 10 - 5, 0, Random.value * 10 - 5);
    }

    public override void AgentOnDone()
    {

    }
/*
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
*/
}
