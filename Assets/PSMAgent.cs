using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using DVRK;
using Unity.VisualScripting;
using Obi;
using System.Linq;
public class PSMAgent : Agent
{
    [Header("Specific to Ball3D")]
    //public GameObject block;
    [Tooltip("Whether to use vector observation. This option should be checked " +
        "in 3DBall scene, and unchecked in Visual3DBall scene. ")]
    public bool useVecObs;
    //Rigidbody m_BallRb;
    EnvironmentParameters m_ResetParams;
    public List<URDFJoint> independentJoints = new List<URDFJoint>();
    public URDFJoint jaw = null;

    public static List<URDFRobot> instances = new List<URDFRobot>();
    private int instanceID = -1;
    //private Vector3 baseBlockPos;

    public CollisionHandler collisionHandler;

    public GameObject tumor;

    private ObiSoftbody softbody;

    private Vector3 defaultPos;
    private Quaternion defaultRot;

    PSM psmScript;

    public bool noNextTriangle = false;

    public bool jointCollision = false;

    private Vector3[] initialParticlePositions;

    public Transform RCM;

    public override void Initialize()
    {
        foreach (URDFJoint joint in GetComponentsInChildren<URDFJoint>())
        {
            joint.SetupRobotJoint();
        }
        foreach (URDFJoint joint in independentJoints)
        {
            joint.SetJointValueDefault();
        }
        if (jaw != null)
        {
            jaw.SetJointValueDefault();
        }
        psmScript = GetComponent<PSM>();


        // instances.Add(this);
        // instanceID = instances.Count - 1;
        // Debug.Log(name + ": Current URDFRobot instanceID: " + instanceID);
        //baseBlockPos = block.transform.position;

        softbody = tumor.GetComponent<ObiSoftbody>();

        StartCoroutine(WaitForSoftbodyReady());

        RCM = GetComponent<PSM>().RCM;


    }
    private IEnumerator WaitForSoftbodyReady()
    {
        while (!softbody.isLoaded)
            yield return null;

        initialParticlePositions = new Vector3[softbody.particleCount];

        for (int i = 0; i < softbody.particleCount; i++)
        {
            int solverIndex = softbody.solverIndices[i];
            initialParticlePositions[i] = softbody.solver.positions[solverIndex];
        }

        defaultPos = tumor.transform.position;
        defaultRot = tumor.transform.rotation;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (useVecObs)
        {
            //Joint Observation data
            foreach (URDFJoint joint in independentJoints)
            {
                sensor.AddObservation(joint.currentJointValue);
            }
            sensor.AddObservation(jaw.currentJointValue);
            sensor.AddObservation(jaw.transform.position - gameObject.transform.position);
            sensor.AddObservation(jaw.transform.rotation); //direction ee pointing

            //Block pick up Observation
            //sensor.AddObservation(block.transform.position - gameObject.transform.position);
            //sensor.AddObservation(block.transform.forward);

            //Tumor Position Observation
            //sensor.AddObservation(tumor.transform.position - gameObject.transform.position);

            //Tumor Surface Area Observation


            // sensor.AddObservation(collisionHandler.currentGoalDistance);
            // sensor.AddObservation(collisionHandler.currentDeltaAngle);


        }
    }


    //Tumor Reward Variables
    int observedTrisCount;
    public Transform tumorAnchor;
    
    public float learningRate = 1f;
    public int maxIterations = 20;
    public float angleThreshold = 0.5f;
    Vector3 jointOperator = new Vector3(0f, -1f, 0f);

    void ApplyStep(URDFJoint joint, Vector3 errorVec, int i)
    {
        // Get world-space axis from jointOperator
        Vector3 localAxis = jointOperator.normalized;
        Vector3 axisWorld = joint.jointObject.transform.parent.TransformDirection(localAxis);

        // Project error onto axis
        // float deltaAngle = Vector3.Dot(axisWorld, errorVec) * learningRate;

        // float newVal = joint.currentJointValue + deltaAngle;
        // independentJoints[i].SetJointValue(newVal);
        //joint.SetJointValue(newVal);

        // Debug.Log($"[IK] Joint {i} | Step: {deltaAngle:F3} | New Val: {newVal:F3} | Axis: {axisWorld}");

        float deltaAngleA = Vector3.Dot(axisWorld, errorVec) * learningRate;
        float newValA = joint.currentJointValue + deltaAngleA;

        float deltaAngleB = -deltaAngleA;
        float newValB = joint.currentJointValue + deltaAngleB;

        bool useA = (newValA >= joint.jointLimit.x && newValA <= joint.jointLimit.y);
        bool useB = (newValB >= joint.jointLimit.x && newValB <= joint.jointLimit.y);

        float chosenDelta = 0f;
        if (useA && (!useB || Mathf.Abs(deltaAngleA) >= Mathf.Abs(deltaAngleB)))
            chosenDelta = deltaAngleA;
        else if (useB)
            chosenDelta = deltaAngleB;

        joint.SetJointValue(joint.currentJointValue + chosenDelta);
    }


    public override void OnActionReceived(ActionBuffers actionBuffers)
    {

        //Delta Jaw Position
        float maxDelta = 0.04f;
        float reward = 0f;
        var jawPos = 2f * Mathf.Clamp(actionBuffers.ContinuousActions[6], -maxDelta * 10, maxDelta * 10);
        jaw.SetJointValue(jaw.currentJointValue + jawPos);
        for (int i = 0; i < independentJoints.Count; i++)
        {
            float jointPos;
            if (i == 2) // Insertion Joint
            {
                //Debug.Log("Insertion action buffers: " + actionBuffers.ContinuousActions[i]);
                jointPos = 2f * Mathf.Clamp(actionBuffers.ContinuousActions[i], -maxDelta / 120, maxDelta / 120);

            }
            else if (new List<int> { 3, 4, 5 }.Contains(i))
            {
                jointPos = 2f * Mathf.Clamp(actionBuffers.ContinuousActions[i], -maxDelta * 5, maxDelta * 5);

            }
            else
            {
                jointPos = 2f * Mathf.Clamp(actionBuffers.ContinuousActions[i], -maxDelta, maxDelta);

            }
            float newVal = independentJoints[i].currentJointValue + jointPos;
            if (independentJoints[i].jointLimit.x < newVal && independentJoints[i].jointLimit.y > newVal)
            {
                independentJoints[i].SetJointValue(newVal);
            }
            else
            {
                //Debug.Log("overshoot");
                reward -= 0.01f;
            }

            //deviation Penalty
            if (i != 2)
            {
                float deviation = Mathf.Abs(newVal - independentJoints[i].defaultJointValue);
                reward -= deviation * 0.0003f;
            }
        }

        //World Space with IK Control
        // Vector3 targetPosition = new Vector3(actionBuffers.ContinuousActions[0],
        //                                      actionBuffers.ContinuousActions[1],
        //                                      actionBuffers.ContinuousActions[2]) + gameObject.transform.position + new Vector3(0f, -1f, 0f);
        // Vector4 tempRotation = new Vector4(actionBuffers.ContinuousActions[3],
        //                                    actionBuffers.ContinuousActions[4],
        //                                    actionBuffers.ContinuousActions[5],
        //                                    actionBuffers.ContinuousActions[6]);
        // tempRotation.Normalize();
        // Quaternion targetRotation = new Quaternion(tempRotation.x, tempRotation.y, tempRotation.z, tempRotation.w);
        // float yaw = -1f * Mathf.Atan2(RCM.position.x - targetPosition.x, RCM.position.y - targetPosition.y) * Mathf.Rad2Deg;
        // float pitch = -1f * Mathf.Atan2(targetPosition.z - RCM.position.z, RCM.position.y - targetPosition.y) * Mathf.Rad2Deg;
        // float insertion = (RCM.position - targetPosition).magnitude / 17f + 0.01f * Mathf.Pow(independentJoints[4].currentJointValue / 80f, 2);

        // independentJoints[0].SetJointValue(yaw);
        // independentJoints[1].SetJointValue(pitch);
        // independentJoints[2].SetJointValue(insertion);

        // for (int iter = 0; iter < 1; iter++)
        // {
        //     Quaternion qCurrent = jaw.transform.rotation;
        //     Quaternion qTarget = targetRotation;
        //     Quaternion delta = qTarget * Quaternion.Inverse(qCurrent);

        //     if (delta.w < 0f)
        //         delta = new Quaternion(-delta.x, -delta.y, -delta.z, -delta.w);

        //     delta.ToAngleAxis(out float angle, out Vector3 axis);
        //     if (angle > 180f) angle -= 360f;
        //     if (float.IsNaN(axis.x)) break;
        //     if (Mathf.Abs(angle) < angleThreshold)
        //     {
        //         Debug.Log($"[IK] Converged at iteration {iter} | Error angle: {angle:F2}°");
        //         break;
        //     }

        //     Vector3 errorVec = axis.normalized * angle;

        //     ApplyStep(independentJoints[5], errorVec, 5);
        //     ApplyStep(independentJoints[4], errorVec, 4);
        //     ApplyStep(independentJoints[3], errorVec, 3);

        // }
        // jaw.SetJointValue(actionBuffers.ContinuousActions[7] * 45f + 45f);



        // //Tumor Removal Reward

        // //Reward for getting closer to tumor
        // float maxReward = 0.02f;

        // float distance = Vector3.Distance(jaw.transform.position, tumorAnchor.position);
        // reward += Mathf.Clamp(maxReward / (distance + 0.001f), 0f, maxReward);

        // //Covering surface area final reward

        // // 1. Incremental reward for newly observed tumor triangles
        // int deltaTris = collisionHandler.successfulTris.Count - observedTrisCount;
        // reward += 1f * deltaTris; // scale this as needed
        // observedTrisCount = collisionHandler.successfulTris.Count;

        // // 2. Minor positive reward for any tumor contact (encourages active interaction)
        // if (collisionHandler.tumorContact)
        //     reward += 0.01f;
        // else
        //     reward -= 0.005f; // gentle penalty for idling

        // // 3. Success condition — enough surface covered
        // if (collisionHandler.successfulTris.Count > 25)
        // {
        //     collisionHandler.complete = true;
        //     reward += 20f;  // final bonus

        // }
        if (jointCollision)
        {
            //Debug.Log("JOINT COLLISION!!!!!!!!!!!!!!!!!!!");
            reward -= 3f;
            jointCollision = false;
        }

        // if (collisionHandler.jawState == JawState.picking && jaw.transform.position.y > 1.8f)
        // {
        //     reward += 50f;
        //     AddReward(reward);
        //     EndEpisode();
        // }
        
        
        // //Debug.Log(reward);
        // AddReward(reward);

        //Block pick up reward
        // if (block.transform.position.y > 0.75f)
        // {
        //     SetReward(10f);
        //     EndEpisode();
        // }
        // else
        // {
        //     SetReward(10 * (block.transform.position.y - baseBlockPos.y));
        // }
        // if (Vector3.Distance(jaw.transform.position, block.transform.position) < 0.01f)
        // {
        //     SetReward(0.1f);
        // }
        // else
        // {
        //     SetReward(-0.05f);
        // }
    }
    int currentDemonstration = 0;
    public override void OnEpisodeBegin()
    {
        Debug.Log("Current Demonstration Number: " + currentDemonstration);
        currentDemonstration++;
        foreach (URDFJoint joint in independentJoints)
        {
            joint.SetJointValueDefault();
        }
        if (jaw != null)
        {
            jaw.SetJointValueDefault();
        }


        // block.transform.position = //new Vector3(Random.Range(-0.1f, 0.1f), 0f, Random.Range(-0.1f, 0.1f))
        //     baseBlockPos;
        //Reset the parameters when the Agent is reset.

        // collisionHandler.observedTris.Clear();
        // collisionHandler.successfulTris.Clear();
        // collisionHandler.picking = false;
        // collisionHandler.tumorAligned = false;
        // collisionHandler.tumorContact = false;
        // collisionHandler.complete = false;
        // collisionHandler.resetToDefault = true;
        foreach (GameObject obj in FindObjectsByType<GameObject>(FindObjectsSortMode.None))
        {
            if (obj.name == "Triangle")
            {
                Destroy(obj);
            }
        }
        //tumor.transform.position = defaultPos;
        //tumor.transform.rotation = defaultRot;
        var particleAttachments = tumor.GetComponents<ObiParticleAttachment>();
        if (particleAttachments.Length > 1)
        {
            Destroy(particleAttachments[1]);
        }
        if (softbody.isLoaded)
        {
            for (int i = 0; i < softbody.particleCount; i++)
            {
                int solverIndex = softbody.solverIndices[i];
                Debug.Log(initialParticlePositions[i]);
                softbody.solver.positions[solverIndex] = initialParticlePositions[i];
                softbody.solver.velocities[solverIndex] = Vector4.zero;
            }

            //softbody.Teleport(defaultPos, defaultRot);
        }
        particleAttachments[0].enabled = true;
        noNextTriangle = false;

        jointCollision = false;
        for (int i = 0; i < 6; i++)
        {
            psmScript.jointPos[i] = 0;
        }
        psmScript.jawPos = 0;
        // collisionHandler.jawState = JawState.closedNoContact;
        // collisionHandler.j = ((int)(Random.Range(0f, (float)collisionHandler.triangles.Length - 1) * 3)) / 3;
        // Debug.Log("From episode: "+collisionHandler.j);

        
        
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
    
        if (psmScript != null)
        {
            if (continuousActionsOut.Length == 8)
            {
                //Debug.Log("entering heuristic");
                // continuousActionsOut[0] = psmScript.targetPosition.x - gameObject.transform.position.x;;
                // continuousActionsOut[1] = psmScript.targetPosition.y - gameObject.transform.position.y + 1f;
                // continuousActionsOut[2] = psmScript.targetPosition.z - gameObject.transform.position.z;
                // continuousActionsOut[3] = psmScript.targetOrientation.x;
                // continuousActionsOut[4] = psmScript.targetOrientation.y;
                // continuousActionsOut[5] = psmScript.targetOrientation.z;
                // continuousActionsOut[6] = psmScript.targetOrientation.w;


                // continuousActionsOut[7] = psmScript.jawPos;

                for (int i = 0; i < 6; i++)
                {
                    continuousActionsOut[i] = psmScript.jointPos[i];
                }
                continuousActionsOut[6] = psmScript.jawPos;

            }
        }






    }

   
}
