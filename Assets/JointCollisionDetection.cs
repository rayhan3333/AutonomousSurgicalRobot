using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JointCollisionDetection : MonoBehaviour
{
    public PSMAgent PSMAgent;
    public CollisionHandler handler;
    private List<string> collisionList;

    // Start is called before the first frame update
    void Start()
    {
        collisionList = new List<string> { "outer_roll_joint", "IC", "jaw_joint" };
    }

    // Update is called once per frame
    void Update()
    {

    }

    void OnCollisionEnter(Collision collision)
    {
        if (collisionList.Contains(collision.gameObject.name))
        {
            PSMAgent.jointCollision = true;
            Debug.Log("Collision with: " + collision.gameObject.name);
            handler.jointCollision = true;
        }
    }

    void OnCollisionStay(Collision collision)
    {
         if (collisionList.Contains(collision.gameObject.name))
        {
            PSMAgent.jointCollision = true;
            Debug.Log("Collision with: " + collision.gameObject.name);
            handler.jointCollision = true;
        }
    }
}
