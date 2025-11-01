using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using Obi;
using RosSharp.Urdf;
using UnityEditor.ShaderGraph;
using Unity.VisualScripting;
using DVRK;
using UnityEditor;
using System;
using UnityEditor.PackageManager.Requests;
//using System.Numerics;

public enum JawState
{
    closedNoContact, //default
    closedContact, // ignore Collisions because jaw entered tumor closed so has to retry

    openNoContact, // listenCollisions

    openContact, // Ideal wait for Close

    picking  // create attachment
}
public class CollisionHandler : MonoBehaviour
{
    ObiSolver solver;
    public GameObject jawObj;

    private Transform jawTransform;
    public GameObject rollObj;
    public GameObject tumorObj;
    public bool jawCol;
    public bool rollCol;

    public DVRK.URDFJoint jaw;

    public JawState jawState;
    public bool picking;
    public GameObject surfacePatchPrefab;



    
    public struct ActorPair
    {
        public readonly ObiActor actorA;
        public readonly ObiActor actorB;
        public int particleA;
        public int particleB;

        public ActorPair(ObiActor actorA, ObiActor actorB, int particleA, int particleB)
        {
            this.actorA = actorA;
            this.actorB = actorB;
            this.particleA = particleA;
            this.particleB = particleB;
        }
    }

    public struct ParticleContact
    {
        public Vector3 normal;
        public Vector3 tangent;
        public Vector3 point;
    }
    public List<ParticleContact> contactList = new List<ParticleContact>();
    public UnityEvent<ActorPair> callback;

    public MeshFilter meshCol;
    Mesh mesh;

    public Vector3[] vertices;
    public int[] triangles;
    public List<int> observedTris = new List<int>();
    public List<int> successfulTris = new List<int>();


    public bool tumorContact;
    public bool tumorAligned;

    public PSM psmScript;

    public Transform tumorPosition;
    public int j = 69;
    public float[] previousjoints;

    public JointPositionRecorder posRecord;

    public void SpawnTriangle(Vector3 p0, Vector3 p1, Vector3 p2)
    {
        GameObject triangleGO = new GameObject("Triangle");

        // Don’t parent unless necessary
        // triangleGO.transform.parent = this.transform;

        MeshFilter mf = triangleGO.AddComponent<MeshFilter>();
        MeshRenderer mr = triangleGO.AddComponent<MeshRenderer>();
        mr.material.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);

        Mesh mesh = new Mesh();
        mesh.name = "TriangleMesh";

        // Make vertices relative to triangleGO (so translate to local space)
        Vector3 center = (p0 + p1 + p2) / 3f;
        triangleGO.transform.position = center;

        Vector3[] vertices = new Vector3[]
        {
            p0 - center,
            p1 - center,
            p2 - center
        };

        int[] triangles = new int[] { 0, 1, 2 };

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();

        mf.mesh = mesh;

        Material mat = new Material(Shader.Find("Unlit/Color"));
        mat.color = Color.cyan;
        mr.material = mat;

    }
    void Awake()
    {
        solver = GetComponent<ObiSolver>();
    }
    void Start()
    {
        previousjoints = new float[6];
        jawTransform = jawObj.transform;
        jawState = JawState.openNoContact;


        mesh = meshCol.sharedMesh;

        vertices = mesh.vertices;
        triangles = mesh.triangles;
        j = ((int)(UnityEngine.Random.Range(0f, (float)triangles.Length - 1) * 3)) / 3;
        //Debug.Log(j);
    }
    //bool IKSuccess = true;
    public int tries = 0;
    public bool complete = false;
    public bool resetToDefault = true;
    int timer = 0;
    public Transform PSMreturn;

    public Vector3 currentGoalDistance;
    public Quaternion currentDeltaAngle;

    
    public int previousJ;

    public bool jointCollision;

    void Update()
    {
        //UNCOMMENT FOR ALGORITHM
        if (successfulTris.Count > 150)
        {

            complete = true;
        }
        else
        {
            //psmScript.jawPos = -1;

        }
        if (jawState == JawState.closedNoContact || jawState == JawState.openNoContact)
        {
            if (jaw.currentJointValue >= 70f)
            {
                jawState = JawState.openNoContact;
            }
            else
            {
                jawState = JawState.closedNoContact;
            }
        }
        else if (jawState == JawState.openContact)
        {
            if (jaw.currentJointValue <= 30f)
            {
                jawState = JawState.picking;
                picking = true;
                Debug.Log("Picking just set");
            }
        }
        else if (jawState == JawState.picking)
        {
            //Debug.Log("picking in update");
            if (jaw.currentJointValue >= 70f)
            {
                jawState = JawState.openNoContact;
                tumorObj.GetComponents<ObiParticleAttachment>()[1].enabled = false;
            }
        }

        //Imitation Learning - Algorithmic Pick Up
        if (complete)
        {
            
     


            //Debug.Log(jawState);
            if (resetToDefault)
            {
                Vector3 _v0 = vertices[triangles[j]];
                Vector3 _v1 = vertices[triangles[j + 1]];
                Vector3 _v2 = vertices[triangles[j + 2]];

                Debug.Log("Starting pickup");
                Vector3 pickupPos = (_v0 + _v1 + _v2) / 3f + Vector3.Cross(_v1 - _v0, _v2 - _v0).normalized * -0.4f; // Add Little off face
                psmScript.targetPosition = meshCol.transform.TransformPoint(pickupPos);
                psmScript.gizmo.position = meshCol.transform.TransformPoint(pickupPos);
                Vector3 pickupDir = Vector3.Cross(_v1 - _v0, _v2 - _v0).normalized;
                psmScript.targetOrientation = Quaternion.LookRotation(pickupDir);
                psmScript.gizmo.rotation = Quaternion.LookRotation(pickupDir);
                psmScript.SendAction(7, 90);
                //psmScript.jawPos = 1;

                timer++;
                if (timer > 3000)
                {
                    resetToDefault = false;
                    timer = 0;
                }

                return;

            }
            Debug.Log("Returning");
            psmScript.gizmo.position = PSMreturn.position;
            psmScript.targetPosition = PSMreturn.position;
            
            return;
        }

        //Imitation Learning Cover Tumor Surfacew Area
        Transform t = meshCol.transform;

        Vector3 v0 = vertices[triangles[j]];
        Vector3 v1 = vertices[triangles[j + 1]];
        Vector3 v2 = vertices[triangles[j + 2]];
        Vector3 faceCenter;
        if ((t.TransformPoint(v0)).y < -0.83)
        {
            faceCenter = (v0 + v1 + v2) / 3f + 0.1f * Vector3.ProjectOnPlane(Vector3.Cross(v1 - v0, v2 - v0), Vector3.up).normalized;
        }
        else
        {
            faceCenter = (v0 + v1 + v2) / 3f + Vector3.Cross(v1 - v0, v2 - v0).normalized * 0.03f; // Add Little off face

        }
        Vector3 localPoint = t.InverseTransformPoint(jawTransform.position);
        float distSqr = (localPoint - faceCenter).sqrMagnitude;
        psmScript.targetPosition = t.TransformPoint(faceCenter);
        psmScript.gizmo.position = t.TransformPoint(faceCenter);
        Vector3 closestNormal = Vector3.Cross(v1 - v0, v2 - v0).normalized;
        psmScript.targetOrientation = Quaternion.LookRotation(Vector3.ProjectOnPlane(Vector3.down, closestNormal));
        psmScript.gizmo.rotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(Vector3.down, closestNormal));
        float angle = Vector3.Angle(t.InverseTransformDirection(
        Vector3.Cross(vertices[triangles[j + 1]] - vertices[triangles[j]], vertices[triangles[j + 2]] - vertices[triangles[j]]).normalized),
            jawTransform.forward);
        if (tries > 2000 || jointCollision)
        {
            //Debug.Log("j:" + j);
            //Debug.Log(psmScript.targetPosition);
            //Debug.Log(psmScript.targetOrientation);
            //Debug.Log("FAILLLLLLLLLEDDDDDDDDDDDDDDDDDDDDD");
            posRecord.storedJointPositions.Clear();
            if (tries > 2000)
            {
                observedTris.Add(j);

            }


            for (int i = 0; i < 6; i++)
            {
                //Debug.Log(previousjoints[i]);
                psmScript.independentJoints[i].SetJointValue(previousjoints[i]);
            }

            tries = 0;
            j = previousJ;
            jointCollision = false;
            
            bool foundNear = false;
            // Debug.Log("prev J " + j);
            int startK = UnityEngine.Random.Range(0, triangles.Length);

            for (int offset = 0; offset < triangles.Length; offset++)
            {
                int k = (startK + offset) % triangles.Length;
                if ((triangles[k] == triangles[j] || triangles[k] == triangles[j + 1] || triangles[k] == triangles[j + 2]) && !observedTris.Contains((k / 3) * 3))
                {
                    //Debug.Log(j);
                    j = (k / 3) * 3;
                    //Debug.Log(j);
                    foundNear = true;
                    break;
                }
            }
            if (!foundNear)
            {
                Debug.Log("CONNECTION BREAK---------------------------------------------");
                psmScript.GetComponent<PSMAgent>().noNextTriangle = true;
                complete = true;

                j += 3;
            }
            if (j > triangles.Length - 1)
            {
                Debug.Log("CONNECTION BREAK---------------------------------------------");

                j = findNewJ();
            }
        }
        tries++;
        //Debug.Log(j);

        if (distSqr < 0.02f && !observedTris.Contains(j))
        {
            //Debug.Log("foujd close triangle");

            if (angle > 85f && angle < 95f)
            {
                tries = 0;
                tumorAligned = true;
                observedTris.Add(j);
                successfulTris.Add(j);
                posRecord.success = true;
                // Debug.Log("jaw satisifies plane test");
                SpawnTriangle(t.TransformPoint(vertices[triangles[j]]),
                    t.TransformPoint(vertices[triangles[j + 1]]),
                    t.TransformPoint(vertices[triangles[j + 2]]));

                for (int i = 0; i < 6; i++) {
                    // Debug.Log(i);
                    // Debug.Log(psmScript.independentJoints[i].currentJointValue);
                    previousjoints[i] = psmScript.independentJoints[i].currentJointValue;
                }
                previousJ = j;
                //Debug.Log(j);

                bool foundNear = false;
                // Debug.Log("prev J " + j);
                int startK = UnityEngine.Random.Range(0, triangles.Length);

                for (int offset = 0; offset < triangles.Length; offset++)
                {
                    int k = (startK + offset) % triangles.Length;
                    if ((triangles[k] == triangles[j] || triangles[k] == triangles[j + 1] || triangles[k] == triangles[j + 2]) && !observedTris.Contains((k / 3) * 3))
                    {
                        //Debug.Log(j);
                        j = (k / 3) * 3;
                        //Debug.Log(j);
                        foundNear = true;
                        break;
                    }
                }
                if (!foundNear)
                {
                    Debug.Log("CONNECTION BREAK---------------------------------------------");
                    psmScript.GetComponent<PSMAgent>().noNextTriangle = true;
                    complete = true;

                    j += 3;
                }
                if (j > triangles.Length - 1)
                {
                    Debug.Log("CONNECTION BREAK---------------------------------------------");

                    j = findNewJ();
                }
                // Debug.Log("new J " + j);
                // Debug.DrawLine(t.TransformPoint(vertices[triangles[closestTriIndex]]), t.TransformPoint(vertices[triangles[closestTriIndex+1]]), Color.red, 5f);
                // Debug.DrawLine(t.TransformPoint(vertices[triangles[closestTriIndex+1]]), t.TransformPoint(vertices[triangles[closestTriIndex+2]]), Color.red, 5f);
                // Debug.DrawLine(t.TransformPoint(vertices[triangles[closestTriIndex+2]]), t.TransformPoint(vertices[triangles[closestTriIndex]]), Color.red, 5f);

            }
            else
            {
                tumorAligned = false;
            }
        }
        else
        {
            //Debug.Log("Not satisfied - Distance: " + distSqr + "Angle: " + angle);

        }
        if (psmScript.gizmo.position.y > -0.35f)
        {
            observedTris.Add(j);
            bool foundNear = false;

            for (int k = 0; k < triangles.Length; k++)
            {
                if ((triangles[k] == triangles[j] || triangles[k] == triangles[j + 1] || triangles[k] == triangles[j + 2]) && !observedTris.Contains((k / 3) * 3))
                {
                    //  Debug.Log("found new triangle");
                    j = (k / 3) * 3;
                    foundNear = true;
                    break;
                }
            }
            if (!foundNear)
            {
                Debug.Log("no connected triangle");
                psmScript.GetComponent<PSMAgent>().noNextTriangle = true;
                complete = true;
                j += 3;
            }
            if (j > triangles.Length - 1)
            {
                j = findNewJ();
            }
        }

        //Create collision surface DONT UNCOMMENT
        // if (contactList != null && contactList.Count > 800)
        // {
        //     //Debug.Log(contactList.Count);
        //     foreach (ParticleContact c in contactList)
        //     {
        //         Vector3 point = c.point;     // contact point
        //         Vector3 normal = c.normal;    // contact normal
        //         Vector3 tangent = c.tangent;   // contact tangent
        //         Vector3 bitangent = Vector3.Cross(normal, tangent);

        //         GameObject patch = Instantiate(surfacePatchPrefab, point, Quaternion.identity);
        //         patch.transform.localScale = new Vector3(0.05f, 0.05f, 0.05f);

        //         // Create rotation from tangent and bitangent
        //         patch.transform.rotation = Quaternion.LookRotation(normal, bitangent); // Align Z to normal, Y to bitangent

        //         Destroy(patch, 300f); // Auto-remove after delay
        //     }
        //     contactList.Clear();
        // }


        //UNCOMMENT FOR IL TRAINING / RL FINETUNE

        float closestDistSqr = Mathf.Infinity;
        int closestTriIndex = -1;
        Vector3 closestNormalRL = Vector3.zero;
        Vector3 closestFace = Vector3.zero;


        //Transform t = meshCol.transform;
        //Vector3 localPoint = t.InverseTransformPoint(jawTransform.position); // Convert to local space

        for (int i = 0; i < triangles.Length; i += 3)
        {
            Vector3 v0RL = vertices[triangles[i]];
            Vector3 v1RL = vertices[triangles[i + 1]];
            Vector3 v2RL = vertices[triangles[i + 2]];
            // if (i % 9 == 0)
            // {
            //     Debug.DrawLine(v0, v1, Color.red);
            //     Debug.DrawLine(v1, v2, Color.red);
            //     Debug.DrawLine(v2, v0, Color.red);
            // }

            Vector3 faceCenterRL = (v0RL + v1RL + v2RL) / 3f + Vector3.Cross(v1RL - v0RL, v2RL - v0RL).normalized * 0.03f;
            float distSqrRL = (localPoint - faceCenterRL).sqrMagnitude;

            if (distSqrRL < closestDistSqr)
            {
                closestDistSqr = distSqrRL;
                closestTriIndex = i;
                closestFace = faceCenterRL;

                // Save normal of closest triangle
                closestNormalRL = Vector3.Cross(v1RL - v0RL, v2RL - v0RL).normalized;
            }
        }
        currentGoalDistance = t.TransformDirection(localPoint - closestFace);
        currentDeltaAngle = Quaternion.LookRotation(Vector3.ProjectOnPlane(Vector3.down, closestNormalRL)) * Quaternion.Inverse(jawTransform.rotation);
        //Debug.Log("Goal distance: " + currentGoalDistance + "Angle: " + currentDeltaAngle);
        // if (closestDistSqr < 0.02f && !observedTris.Contains(closestTriIndex))
        // {
        //     tumorContact = true;
        //     //Debug.Log("foujd close triangle");
        //     float angle = Vector3.Angle(t.InverseTransformDirection(
        //         Vector3.Cross(vertices[triangles[closestTriIndex + 1]] - vertices[triangles[closestTriIndex]], vertices[triangles[closestTriIndex + 2]] - vertices[triangles[closestTriIndex]]).normalized),
        //         jawTransform.forward);
        //     Debug.Log(angle);
        //     if (angle > 85f && angle < 95f)
        //     {
        //         tumorAligned = true;
        //         observedTris.Add(closestTriIndex);
        //         successfulTris.Add(closestTriIndex);
        //         Debug.Log("jaw satisifies plane test");
        //         // SpawnTriangle(t.TransformPoint(vertices[triangles[closestTriIndex]]),
        //         //     t.TransformPoint(vertices[triangles[closestTriIndex + 1]]),
        //         //     t.TransformPoint(vertices[triangles[closestTriIndex + 2]]));

        //         // Debug.DrawLine(t.TransformPoint(vertices[triangles[closestTriIndex]]), t.TransformPoint(vertices[triangles[closestTriIndex+1]]), Color.red, 5f);
        //         // Debug.DrawLine(t.TransformPoint(vertices[triangles[closestTriIndex+1]]), t.TransformPoint(vertices[triangles[closestTriIndex+2]]), Color.red, 5f);
        //         // Debug.DrawLine(t.TransformPoint(vertices[triangles[closestTriIndex+2]]), t.TransformPoint(vertices[triangles[closestTriIndex]]), Color.red, 5f);

        //     }
        //     else
        //     {
        //         tumorAligned = false;
        //     }
        // }
        // else
        // {
        //     tumorContact = false;
        // }

    }

    int findNewJ()
    {
        observedTris.Sort();
        int j = 0;
        foreach (int element in observedTris)
        {
            if (element == j)
            {
                j += 3;
            }
            else
            {
                return j;
            }
        }
        Debug.Log("no new J found: done");
        complete = true;
        return -1;
    }
    void OnEnable()
    {
        solver.OnCollision += Solver_OnCollision;
        solver.OnParticleCollision += Solver_OnParticleCollision;
    }

    void OnDisable()
    {
        solver.OnCollision -= Solver_OnCollision;
        solver.OnParticleCollision -= Solver_OnParticleCollision;

    }

    void Solver_OnCollision(object sender, ObiNativeContactList e)
    {

        //Debug.Log("check");
        //first contact (jaw and tumor) jaw has to be exposed
        var world = ObiColliderWorld.GetInstance();

        // just iterate over all contacts in the current frame:
        if (jawState != JawState.closedContact && complete)
        {
            bool tempRollCol = false;
            bool tempJawCol = false;
            foreach (Oni.Contact contact in e)
            {
                // if this one is an actual collision:
                if (contact.distance < 0.01)
                {

                    ObiSolver.ParticleInActor pa = solver.particleToActor[solver.simplices[contact.bodyA]];
                    if (pa.actor.gameObject == tumorObj)
                    {
                        ObiColliderBase col = world.colliderHandles[contact.bodyB].owner;
                        if (col != null)
                        {


                            //Debug.Log(col.name);
                            if (col.gameObject == jawObj)
                            {
                                if (picking) //only run once
                                {

                                    tumorObj.GetComponent<ObiParticleAttachment>().enabled = false;

                                    ObiParticleAttachment attachment = tumorObj.AddComponent<ObiParticleAttachment>();
                                    attachment.target = jaw.transform;

                                    var group = ScriptableObject.CreateInstance<ObiParticleGroup>();
                                    group.particleIndices.Add(pa.indexInActor); // index of the particle in the actor
                                    attachment.particleGroup = group;
                                    picking = false;


                                }
                                //first time check if jaw is closed
                                if (jaw.currentJointValue < 70f && !jawCol)
                                {
                                    Debug.Log("ENTERED CLOSED CONTACT!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                                    jawState = JawState.closedContact;
                                    jawCol = true;
                                    return;
                                }
                                jawCol = true;
                                tempJawCol = true;

                            }

                            if (col.gameObject == rollObj)
                            {
                                rollCol = true;
                                tempRollCol = true;
                            }

                            if (rollCol && jawCol && jawState != JawState.closedContact && jawState != JawState.picking)
                            {

                                jawState = JawState.openContact;
                            }
                        }
                    }


                }
            }
            if (!tempRollCol)
            {
                rollCol = false;
            }
            if (!tempJawCol)
            {
                jawCol = false;
            }
            if ((!rollCol || !jawCol) && (jawState == JawState.openContact || jawState == JawState.closedContact))
            {
                jawState = JawState.closedNoContact; //reset to default
            }
        }
        else if (complete)
        {
            foreach (Oni.Contact contact in e)
            {
                // if this one is an actual collision:
                if (contact.distance < 0.01)
                {
                    ObiColliderBase col = world.colliderHandles[contact.bodyB].owner;
                    if (col != null)
                    {
                        //Debug.Log(col.name);
                        return;
                    }
                }


            }
            //if no contacts
            jawState = JawState.closedNoContact;

        }
    }
    bool runOnce = true;

    void Solver_OnParticleCollision(object sender, ObiNativeContactList e)
    {
        if (!solver.initialized || callback == null) return;

        // just iterate over all contacts in the current frame:
        foreach (Oni.Contact contact in e)
        {
            // if this one is an actual collision:
            if (contact.distance < 0.01)
            {

                // get the index of the first entry in the simplices array for both bodies:
                int startA = solver.simplexCounts.GetSimplexStartAndSize(contact.bodyA, out _);
                int startB = solver.simplexCounts.GetSimplexStartAndSize(contact.bodyB, out _);

                // retrieve the index of both particles from the simplices array:
                int particleA = solver.simplices[startA];
                int particleB = solver.simplices[startB];

                Vector3 point = solver.positions[particleA];

                // retrieve info about both actors involved in the collision:
                var particleInActorA = solver.particleToActor[particleA];
                var particleInActorB = solver.particleToActor[particleB];

                // if they're not the same actor, trigger a callback:
                if (particleInActorA != null && particleInActorB != null && particleInActorA.actor != particleInActorB.actor)
                {
                    //callback.Invoke(new ActorPair(particleInActorA.actor, particleInActorB.actor, particleA, particleB));
                    //Debug.Log("Normal: " + contact.normal + "   Tangent: " + contact.tangent + "    Bitangent: " + contact.bitangentImpulse);
                    if (runOnce && contactList != null)
                    {
                        contactList.Add(new ParticleContact { normal = contact.normal, tangent = contact.tangent, point = point });
                    }
                }

            }
        }
        runOnce = false;

	}
}

