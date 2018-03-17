using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CrawlerLegContact : MonoBehaviour {

    public int index;
    public CrawlerAgentConfigurable agent;

    void Start(){
//        agent = gameObject.transform.parent.gameObject.GetComponent<SpiderAgent>();
    }

    void OnCollisionEnter(Collision other){
        if (other.gameObject.name == "Platform")
        {
            agent.leg_touching[index] = true;
        } else {
            agent.flirting = true;
        }
    }

    void OnCollisionExit(Collision other){
        if (other.gameObject.name == "Platform")
        {
            agent.leg_touching[index] = false;
        }
    }

}
