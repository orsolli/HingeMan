using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Vector3 offset = Vector3.up;
    public int timeAtEach = 10;
    int timer = 0;
    public Transform currentTarget;
    public List<GameObject> subjects;
    public int[] sortedSubjects = new int[36];
    Transform lastTarget;
    int index = 0;
    // Update is called once per frame
    float frameTimer = 1;
    // Start is called before the first frame update
    void Start()
    {
        int i = 0;
        foreach (GameObject subject in subjects)
        {
            float rotation = (360f + subject.transform.eulerAngles.y) % 360f;
            sortedSubjects[(int)(rotation / 10f + 0.5f)] = i++;
        }
        currentTarget = subjects[sortedSubjects[35]].GetComponentInChildren<Rigidbody>().transform;
    }
    void FixedUpdate()
    {
        frameTimer += Time.fixedDeltaTime;
        if (frameTimer > 1)
        {
            frameTimer -= 1;
            if (timer++ == 0)
            {
                GameObject subject = subjects[sortedSubjects[index++]];
                lastTarget = currentTarget;
                currentTarget = subject.GetComponentInChildren<Rigidbody>().transform;
            }
            timer = timer % timeAtEach;
            index = index % 36;
        }
        transform.position = lastTarget.position + offset;
        transform.LookAt(currentTarget.position, Vector3.up);
    }
}
