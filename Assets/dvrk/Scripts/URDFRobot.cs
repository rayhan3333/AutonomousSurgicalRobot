/*
    Author(s):  Long Qian
    Created on: 2019-03-29
    (C) Copyright 2015-2018 Johns Hopkins University (JHU), All Rights Reserved.

    --- begin cisst license - do not edit ---
    This software is provided "as is" under an open source license, with
    no warranty.  The complete license can be found in license.txt and
    http://www.cisst.org/cisst/license.txt.
    --- end cisst license ---
*/
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace DVRK {

    public class URDFRobot : MonoBehaviour {

        public List<URDFJoint> independentJoints = new List<URDFJoint>();
        public URDFJoint jaw = null;

        public Transform ee;

        public static List<URDFRobot> instances = new List<URDFRobot>();
        private int instanceID = -1;

        private UDPClient udpClient;
        private UdpJsonSender udpSender;

        public Transform gizmo;
        private Vector3 lastPos;
        public Transform RCM;

        public Mode mode = Mode.write;
        private Transform RCMstart;

        public virtual void HandleMessage(string message)
        {
            Debug.Log("DVRK::URDFRobot base class not implementing HandleMessage");
        }
        public virtual void SendJointStates(UdpJsonSender udpsend, bool moving)
        {
            Debug.Log("DVRK::URDFRobot base class not implementing SendJointStates");

        }

        float val = 0;
        // Use this for initialization
        void Start()
        {

            foreach (URDFJoint joint in GetComponentsInChildren<URDFJoint>())
            {
                joint.SetupRobotJoint();
            }
            int current = 0;
            foreach (URDFJoint joint in independentJoints)
            {
                joint.SetJointValueDefault();
                jointFrames[current] = joint.transform;
                current+= 1;
            }
            if (jaw != null)
            {
                jaw.SetJointValueDefault();
            }
            
            

            instances.Add(this);
            instanceID = instances.Count - 1;
            Debug.Log(name + ": Current URDFRobot instanceID: " + instanceID);
        }

        // void SolveIK()
        // {
        //     int maxIterations = 10;
        //     float positionThreshold = 0.01f;
        //     float rotationWeight = 0.5f;

        //     Transform endEffector = jaw.transform;

        //     for (int iter = 0; iter < maxIterations; iter++)
        //     {
        //         // Check if the end effector is close enough to the target
        //         float distanceToTarget = Vector3.Distance(endEffector.position, target.position);
        //         if (distanceToTarget < positionThreshold)
        //             break;

        //         // Start from end effector and move backward
        //         for (int i = independentJoints.Count - 2; i >= 0; i--)
        //         {
        //             URDFJoint joint = independentJoints.ElementAt<URDFJoint>(i);

        //             // --- POSITION ---
        //             Vector3 toEffector = endEffector.position - joint.position;
        //             Vector3 toTarget = target.position - joint.position;

        //             Quaternion rotToTarget = Quaternion.FromToRotation(toEffector, toTarget);
        //             joint.rotation = rotToTarget * joint.rotation;

        //             // --- ORIENTATION (only apply a fraction per iteration) ---
        //             if (rotationWeight > 0 && i == joints.Length - 2)
        //             {
        //                 Quaternion desiredRotation = target.rotation;
        //                 Quaternion currentRotation = endEffector.rotation;
        //                 Quaternion rotDelta = desiredRotation * Quaternion.Inverse(currentRotation);
        //                 joint.rotation = Quaternion.Slerp(joint.rotation, rotDelta * joint.rotation, rotationWeight);
        //             }
        //         }
        //     }
        // }

        // LateUpdate is called once per frame

        public Vector3 targetPosition;
        public Quaternion targetOrientation;
        bool insertionOffset = false;
        void LateUpdate()
        {
            //foreach (URDFJoint joint in independentJoints)
            //{
                //Debug.Log(joint.currentJointValue);
            //}
            //Debug.Log(independentJoints[2].currentJointValue);

            //Debug.Log(independentJoints.Count);
            targetOrientation = gizmo.transform.rotation;
            targetPosition = gizmo.transform.position;
            if (targetPosition != null || targetOrientation != null)
            {

              //  SolvePosition();
                //SolveOrientationIK();
            }

            // independentJoints[1].SetJointValue(val);
                // val += 0.05f;

        }
        //public PSMAgent psmAgent;
        public float[] jointPos = new float[6];
        public float jawPos;
        public void SendAction(int id, float val)
        {
            independentJoints[id].SetJointValue(val);
            if (new List<int> { 0, 1, 2 }.Contains(id))
            {
                //Debug.Log("Moving");
                
                //Debug.Log("send action: " + (val - independentJoints[id].currentJointValue));
                //jointPos[id] = 0.05f * (val - independentJoints[id].currentJointValue);
                //Debug.Log("New Position: " + id + " is " + jointPos[id]);
            }
            else if (id == 7)
            {
                jawPos = 30f * (val - jaw.currentJointValue);
            }
            else
            {
                //jointPos[id] = 10f * (val - independentJoints[id].currentJointValue);
                //Debug.Log("New Position: " + id + " is " + jointPos[id]);
            }
        }


        void SolvePosition()
        {
            //Debug.Log(targetPosition);
            //Vector3 localDir = RCM.InverseTransformDirection(gizmo.position - RCM.position);
            float yaw = -1f * Mathf.Atan2(RCM.position.x - gizmo.position.x, RCM.position.y - gizmo.position.y) * Mathf.Rad2Deg;
            float pitch = -1f * Mathf.Atan2(gizmo.position.z - RCM.position.z, RCM.position.y - gizmo.position.y) * Mathf.Rad2Deg;
            SendAction(0, yaw);
            SendAction(1, pitch);
            //SendAction(2, (RCM.position - gizmo.position).magnitude / 17f+0.01f*Mathf.Pow(independentJoints[4].currentJointValue / 80f, 2));

            // SendAction(2, (RCM.position - gizmo.position).magnitude / 17.95f);

            // return;

            //float desiredDistance = (gizmo.position - RCM.position).magnitude;

            //Vector3 insertionAxis = RCM.forward;
            Vector3 insertionAxis = (gizmo.position - RCM.position).normalized;

            // Compute current joint insertion
            float currentInsertion = independentJoints[2].currentJointValue;

            // Compute current tool tip position (after joint insertion applied)
            //Vector3 toolTip = RCM.position + insertionAxis.normalized * currentInsertion;
            Vector3 toolTip = independentJoints[3].transform.position;

            // Actual distance from tip to target
             float actualError = Vector3.Dot(gizmo.position - toolTip, insertionAxis.normalized);
            //Debug.Log("Error: " + actualError);
            
            // // Proportional gain (tune this)
            float kP = 1.0f; // smaller = slower, larger = more responsive (but can overshoot)

            // // Compute adjustment
             float deltaInsertion = kP * actualError;

            // // Apply adjustment
            float newInsertion = currentInsertion + deltaInsertion;
            //Debug.Log(newInsertion);

            // // Clamp if needed
            float minInsertion = independentJoints[2].jointLimit.x;
            float maxInsertion = independentJoints[2].jointLimit.y;
            newInsertion = Mathf.Clamp(newInsertion, minInsertion, maxInsertion) - 0.03f;
            //Debug.Log("new insertion: " + newInsertion);
            // // Send to prismatic joint
            //SendAction(2, newInsertion);
            // independentJoints[0].SetJointValue(yaw);
            // independentJoints[1].SetJointValue(pitch);
            independentJoints[2].SetJointValue((RCM.position - gizmo.position).magnitude / 17f); //hardcode scale not accurate
            // float insertion = Mathf.Clamp(localDir.magnitude, independentJoints[2].jointLimit.x, independentJoints[2].jointLimit.y);
            // independentJoints[2].SetJointValue(insertion);

            // Vector3 axisOrigin = RCM.position;

            // // Vector from base to tip and to gizmo
            // Vector3 toTip = toolTip - axisOrigin;
            // Vector3 toGizmo = gizmo.position - axisOrigin;

            // // Project both onto axis
            // float tipProj = Vector3.Dot(toTip, insertionAxis);
            // float gizmoProj = Vector3.Dot(toGizmo, insertionAxis);

            // // Compute insertion error
            // float error = gizmoProj - tipProj;

            // // Proportional adjustment
            // float kP = 0.25f;
            // float delta = kP * error;

            // float currentInsertion = independentJoints[2].currentJointValue;
            // float desiredInsertion = Mathf.Clamp(
            //     currentInsertion + delta,
            //     independentJoints[2].jointLimit.x,
            //     independentJoints[2].jointLimit.y
            // );

            // SendAction(2, desiredInsertion);

        }

        // ------- config -------
        [SerializeField] Transform[] jointFramesParent = new Transform[6]; // parent/static side; local +Y is axis
        [SerializeField] Transform tip;         // real tool tip
        //[SerializeField] bool revoluteUsesDegrees = true;      // what your joint API expects for 0 & 1
        [SerializeField] float insertionUnitsPerMeter = 17.0f; // convert meters -> your prismatic units
        [SerializeField] float damping = 1e-3f;                // λ (stability)
        [SerializeField] int posIters = 3;                     // a few steps are enough
        [SerializeField] float maxYawStepDeg = 2f, maxPitchStepDeg = 2f;
        [SerializeField] float maxInsStepUnits = 0.004f;       // prismatic clamp in joint units

        // Call this right AFTER your wrist/orientation solve:
        public void CorrectPositionAfterOrientation()
        {
            tip = jaw.transform;
            for (int i =0; i < jointFramesParent.Length; i++)
            {
                jointFramesParent[i] = independentJoints[i].transform;
            }
            for (int iter = 0; iter < posIters; iter++)
            {
                Vector3 e = gizmo.position - tip.position;      // position error (meters)
                if (e.sqrMagnitude < 1e-8f) break;

                // --- Build J_pos (3x3) for [yaw, pitch, insertion] ---
                Vector3 p = tip.position;

                // Revolute columns: w × (p - r)
                Vector3 w0 = jointFramesParent[0].TransformDirection(Vector3.up).normalized;
                Vector3 r0 = jointFramesParent[0].position;
                Vector3 c0 = Vector3.Cross(w0, p - r0);

                Vector3 w1 = jointFramesParent[1].TransformDirection(Vector3.up).normalized;
                Vector3 r1 = jointFramesParent[1].position;
                Vector3 c1 = Vector3.Cross(w1, p - r1);

                // Prismatic column: axis direction (in meters). If your joint units aren’t meters,
                // we’ll convert the delta later; keep this as meters here.
                Vector3 a2 = jointFramesParent[2].TransformDirection(Vector3.up).normalized;
                Vector3 c2 = a2;

                // A = J^T J + λ² I, b = J^T e
                float lam2 = damping * damping;

                // columns as vectors
                Vector3 C0 = c0, C1 = c1, C2 = c2;
                // A (symmetric)
                float A00 = Vector3.Dot(C0, C0) + lam2;
                float A01 = Vector3.Dot(C0, C1);
                float A02 = Vector3.Dot(C0, C2);
                float A11 = Vector3.Dot(C1, C1) + lam2;
                float A12 = Vector3.Dot(C1, C2);
                float A22 = Vector3.Dot(C2, C2) + lam2;
                // b
                Vector3 JT_e = new Vector3(Vector3.Dot(C0, e), Vector3.Dot(C1, e), Vector3.Dot(C2, e));

                // Solve 3x3 symmetric system
                Vector3 dq = SolveSymmetric3x3(A00, A01, A02, A11, A12, A22, JT_e);

                // Clamp and apply:
                // yaw (0) & pitch (1) are angles (rad) → convert to API units if needed
                float dYaw = Mathf.Clamp(dq.x, -maxYawStepDeg * Mathf.Deg2Rad, maxYawStepDeg * Mathf.Deg2Rad);
                float dPitch = Mathf.Clamp(dq.y, -maxPitchStepDeg * Mathf.Deg2Rad, maxPitchStepDeg * Mathf.Deg2Rad);
                float dIns_m = dq.z; // meters
                float dIns_units = Mathf.Clamp(dIns_m * insertionUnitsPerMeter, -maxInsStepUnits, maxInsStepUnits);

                ApplyDeltaAngle(0, dYaw);
                ApplyDeltaAngle(1, dPitch);
                ApplyDeltaPrismatic(2, dIns_units);

                Physics.SyncTransforms(); // ensure tip moved before next iteration
                if ((gizmo.position - tip.position).magnitude < 1e-3f) break; // ~1 mm
            }
        }

        // --- helpers ---
        void ApplyDeltaAngle(int idx, float deltaRad)
        {
            float cur = independentJoints[idx].currentJointValue;
            float next = revoluteUsesDegrees ? cur + deltaRad * Mathf.Rad2Deg : cur + deltaRad;

            // clamp to joint limits if you have them
            next = Mathf.Clamp(next, independentJoints[idx].jointLimit.x, independentJoints[idx].jointLimit.y);
            SetJoint(idx, next);
        }

        void ApplyDeltaPrismatic(int idx, float deltaUnits)
        {
            float next = independentJoints[idx].currentJointValue + deltaUnits;
            next = Mathf.Clamp(next, independentJoints[idx].jointLimit.x, independentJoints[idx].jointLimit.y);
            SetJoint(idx, next);
        }

        void SetJoint(int idx, float val)
        {
            // Use your existing sender; fallback to SetJointValue if needed
            // SendAction(idx, val);
            independentJoints[idx].SetJointValue(val);
        }

        // Solve symmetric 3x3 (A00 A01 A02; A01 A11 A12; A02 A12 A22)
        static Vector3 SolveSymmetric3x3(float A00, float A01, float A02,
                                         float A11, float A12,
                                         float A22, Vector3 b)
        {
            // Gaussian elimination tailored for 3x3 SPD-ish
            float a00 = A00, a01 = A01, a02 = A02, a11 = A11, a12 = A12, a22 = A22;
            float bx = b.x, by = b.y, bz = b.z;

            if (Mathf.Abs(a00) < 1e-9f) a00 = 1e-9f;
            float f10 = a01 / a00; a11 -= f10 * a01; a12 -= f10 * a02; by -= f10 * bx;
            float f20 = a02 / a00; a22 -= f20 * a02; bz -= f20 * bx;

            if (Mathf.Abs(a11) < 1e-9f) a11 = 1e-9f;
            float f21 = a12 / a11; a22 -= f21 * a12; bz -= f21 * by;

            if (Mathf.Abs(a22) < 1e-9f) a22 = 1e-9f;
            float z = bz / a22;
            float y = (by - a12 * z) / a11;
            float x = (bx - a01 * y - a02 * z) / a00;
            return new Vector3(x, y, z);
        }

        void SolveAgentPosition()
        {
            //Vector3 localDir = RCM.InverseTransformDirection(gizmo.position - RCM.position);
            float yaw = -1f * Mathf.Atan2(RCM.position.x - gizmo.position.x, RCM.position.y - gizmo.position.y) * Mathf.Rad2Deg;
            float pitch = -1f * Mathf.Atan2(gizmo.position.z - RCM.position.z, RCM.position.y - gizmo.position.y) * Mathf.Rad2Deg;
            SendAction(0, yaw);
            SendAction(1, pitch);
            SendAction(2, (RCM.position - gizmo.position).magnitude / 17f+0.01f*Mathf.Pow(independentJoints[4].currentJointValue / 80f, 2));
            // independentJoints[0].SetJointValue(yaw);
            // independentJoints[1].SetJointValue(pitch);
            // independentJoints[2].SetJointValue((RCM.position - gizmo.position).magnitude / 17f); //hardcode scale not accurate
            // float insertion = Mathf.Clamp(localDir.magnitude, independentJoints[2].jointLimit.x, independentJoints[2].jointLimit.y);
            // independentJoints[2].SetJointValue(insertion);

        }

        
      
        //public float learningRate = 1f;
        public int maxIterations = 20;
        public float angleThreshold = 0.5f;

        // void SolveOrientationIK()
        // {
        //     for (int iter = 0; iter < 1; iter++)
        //     {
        //         Quaternion qCurrent = jaw.transform.rotation;
        //         Quaternion qTarget = gizmo.rotation;
        //         Quaternion delta = qTarget * Quaternion.Inverse(qCurrent);

        //         if (delta.w < 0f)
        //             delta = new Quaternion(-delta.x, -delta.y, -delta.z, -delta.w);

        //         delta.ToAngleAxis(out float angle, out Vector3 axis);
        //         if (angle > 180f) angle -= 360f;
        //         if (float.IsNaN(axis.x)) break;
        //         if (Mathf.Abs(angle) < angleThreshold)
        //         {
        //             //Debug.Log($"[IK] Converged at iteration {iter} | Error angle: {angle:F2}°");
        //             break;
        //         }
        //         float angleEps = 1e-2f;   // ~0.57 degrees
        //         float axisEps = 1e-4f;    // magnitude of the axis

        //         if (float.IsNaN(axis.x) || axis == Vector3.zero || axis.magnitude < axisEps || Mathf.Abs(angle) < angleEps)
        //             break; // IK converged or direction unreliable

        //         Vector3 errorVec = axis.normalized * angle;

        //         ApplyStep(independentJoints[5], errorVec, 5);
        //         ApplyStep(independentJoints[4], errorVec, 4);
        //         ApplyStep(independentJoints[3], errorVec, 3);

        //     }
        //     //independentJoints[2].SetJointValue(independentJoints[2].currentJointValue + 0.01f*Mathf.Pow(independentJoints[4].currentJointValue / 80f, 2));

        // }

        [Header("Wrist IK")]
        public Transform[] jointFrames = new Transform[6]; // 0..5 parent-side locators; axis = local Y
        public float learningRate = 0.5f;                  // 0.2–0.7 good
        public int orientationIters = 6;
        public float maxStepDeg = 3f;                      // per-joint per-iter clamp (deg)
        public bool revoluteUsesDegrees = true;            // matches your API
        public bool jointsUseDegrees = true;
        void SolveOrientationIK()
        {
            const float ANG_EPS = 0.05f;      // deg
            const float AXIS_EPS = 1e-6f;

            for (int iter = 0; iter < 3; iter++) // a few passes is plenty
            {
                // Desired: align tool forward with target forward (translation doesn't affect this)
                Vector3 fCur = jaw.transform.forward;
                Vector3 fTgt = gizmo.transform.forward;

                float errDeg = Vector3.Angle(fCur, fTgt);
                if (errDeg < ANG_EPS) break;

                // Stable axis for the minimal rotation fCur -> fTgt
                Vector3 axis = Vector3.Cross(fCur, fTgt);
                if (axis.sqrMagnitude < AXIS_EPS)
                {
                    // parallel (0°) or anti-parallel (180°): pick a consistent fallback axis ⟂ fCur
                    axis = Vector3.Cross(fCur, Vector3.up);
                    if (axis.sqrMagnitude < AXIS_EPS) axis = Vector3.Cross(fCur, Vector3.right);
                }
                axis.Normalize();

                // Rotation vector in RADIANS (world)
                Vector3 rotVecRad = axis * (errDeg * Mathf.Deg2Rad);

                // Distal -> proximal: 5,4,3. Recompute error after each step.
                StepJointTowardAxis(5, ref rotVecRad);
                StepJointTowardAxis(4, ref rotVecRad);
                StepJointTowardAxis(3, ref rotVecRad);

                // If tiny after these three, bail early
                float leftDeg = Quaternion.Angle(ee.rotation, Quaternion.FromToRotation(fCur, fTgt) * ee.rotation);
                if (leftDeg < ANG_EPS) break;
            }
            Debug.Log("Ran Orientation IK");
            //CorrectPositionAfterOrientation();
        }

        void StepJointTowardAxis(int i, ref Vector3 rotVecRad)
        {
            // Joint world axis = its OWN local +Y (not the parent)
            Vector3 axisWorld = independentJoints[i].jointObject.transform.TransformDirection(Vector3.up).normalized;

            // Project the rotation vector onto this axis
            float deltaRad = learningRate * Vector3.Dot(axisWorld, rotVecRad);
            float maxStepRad = maxStepDeg * Mathf.Deg2Rad;
            deltaRad = Mathf.Clamp(deltaRad, -maxStepRad, maxStepRad);

            // Respect limits and units
            float cur = independentJoints[i].currentJointValue;
            float next = jointsUseDegrees ? cur + deltaRad * Mathf.Rad2Deg : cur + deltaRad;

            float lo = independentJoints[i].jointLimit.x;
            float hi = independentJoints[i].jointLimit.y;
            if (lo < hi) next = Mathf.Clamp(next, lo, hi);

            // Apply
            SendAction(i, next);
            Physics.SyncTransforms();

            // Recompute the residual rotation vector for the remaining joints
            Vector3 fCur = jaw.transform.forward;
            Vector3 fTgt = gizmo.transform.forward;
            float errDeg = Vector3.Angle(fCur, fTgt);
            if (errDeg < 0.05f) { rotVecRad = Vector3.zero; return; }

            Vector3 axis = Vector3.Cross(fCur, fTgt);
            if (axis.sqrMagnitude < 1e-6f)
            {
                axis = Vector3.Cross(fCur, Vector3.up);
                if (axis.sqrMagnitude < 1e-6f) axis = Vector3.Cross(fCur, Vector3.right);
            }
            rotVecRad = axis.normalized * (errDeg * Mathf.Deg2Rad);
        }

        [Range(1, 30)] public int wristIters = 8;
        [Range(0.05f, 1.0f)] public float learn = 0.4f;   // step gain
        public Vector3 toolForwardLocal = Vector3.forward; // forward axis on 'jaw'
        public Vector3 scissorUpLocal = Vector3.up;      // “up” axis on 'jaw' for roll
        public Vector3 verticalWorld = Vector3.up;      // what “vertical” means
        public float[] jointDir = new float[6] { 1, 1, 1, 1, 1, 1 }; // set -1 to flip any joint

        // --- Call this after position is set ---
        public void SolveOrientationForwardThenVertical()
        {
            // Build the desired orientation: forward = gizmo.forward; up = world-up projected onto plane ⟂ forward
            Vector3 f_des = gizmo.TransformDirection(toolForwardLocal).normalized;
            Vector3 up_proj = Vector3.ProjectOnPlane(verticalWorld.normalized, f_des);
            if (up_proj.sqrMagnitude < 1e-8f) // forward almost vertical -> pick a stable fallback
                up_proj = Vector3.ProjectOnPlane(Vector3.right, f_des);
            up_proj.Normalize();
            Quaternion R_des = Quaternion.LookRotation(f_des, up_proj);

            for (int iter = 0; iter < wristIters; iter++)
            {
                // Rotation bringing current jaw -> desired
                Quaternion qCur = ee.rotation;
                Quaternion qErr = Quaternion.Inverse(qCur) * R_des;

                qErr.ToAngleAxis(out float angDeg, out Vector3 axisWorld);
                if (float.IsNaN(axisWorld.x)) break;
                if (angDeg > 180f) angDeg -= 360f;
                if (Mathf.Abs(angDeg) < 0.05f) break; // close enough

                Vector3 rotVecRad = axisWorld.normalized * (angDeg * Mathf.Deg2Rad);

                // Distal -> proximal: J5 (wrist yaw), J4 (wrist pitch), J3 (tool roll)
                for (int j = 5; j >= 3; j--)
                {
                    Vector3 axisJ = jointFrames[j].TransformDirection(Vector3.up).normalized; // local +Y of parent side
                    float deltaRad = learn * Vector3.Dot(axisJ, rotVecRad);
                    deltaRad = Mathf.Clamp(deltaRad, -maxStepDeg * Mathf.Deg2Rad, maxStepDeg * Mathf.Deg2Rad);
                    deltaRad *= (jointDir[j] >= 0f ? 1f : -1f);

                    float cur = independentJoints[j].currentJointValue;
                    float next = revoluteUsesDegrees ? cur + deltaRad * Mathf.Rad2Deg : cur + deltaRad;

                    // clamp to joint limits
                    float lo = independentJoints[j].jointLimit.x;
                    float hi = independentJoints[j].jointLimit.y;
                    if (lo < hi) next = Mathf.Clamp(next, lo, hi);

                    // apply
                    SendAction(j, next);

                    // re-sync & re-evaluate error for better convergence (important for horizontal targets)
                    Physics.SyncTransforms();
                    qCur = ee.rotation;
                    qErr = Quaternion.Inverse(qCur) * R_des;
                    qErr.ToAngleAxis(out angDeg, out axisWorld);
                    if (float.IsNaN(axisWorld.x)) break;
                    if (angDeg > 180f) angDeg -= 360f;
                    if (Mathf.Abs(angDeg) < 0.05f) return;

                    rotVecRad = axisWorld.normalized * (angDeg * Mathf.Deg2Rad);
                }
            }
        }
        Vector3 jointOperator = new Vector3(0f, -1f, 0f);

        void ApplyStep(URDFJoint joint, Vector3 errorVec, int i)
        {
            // Get world-space axis from jointOperator
            Vector3 localAxis = jointOperator.normalized;
            Vector3 axisWorld = joint.jointObject.transform.parent.TransformDirection(localAxis);

            // Project error onto axis
            float deltaAngle = Vector3.Dot(axisWorld, errorVec) * learningRate;

            float newVal = joint.currentJointValue + deltaAngle;
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

            //joint.SetJointValue(joint.currentJointValue + chosenDelta);
            SendAction(i, joint.currentJointValue + chosenDelta);

        }



#if UNITY_EDITOR
        void OnGUI()
        {
            int width = 100;
            int height = 20;
            int currentHeight = height;
            int setupHeight = 20;
            foreach (URDFJoint joint in independentJoints)
            {
                GUI.Label(new Rect(10 + instanceID * width, currentHeight, width, height), joint.name);
                currentHeight += setupHeight;
                float val = joint.defaultJointValue;
                if (joint.jointType == URDFJoint.JointType.Revolute || joint.jointType == URDFJoint.JointType.Prismatic)
                {
                    val = GUI.HorizontalSlider(new Rect(10 + instanceID * width, currentHeight, width, height), joint.currentJointValue,
                        joint.jointLimit.x, joint.jointLimit.y);
                }
                else if (joint.jointType == URDFJoint.JointType.Continuous)
                {
                    val = GUI.HorizontalSlider(new Rect(10 + instanceID * width, currentHeight, width, height), joint.currentJointValue,
                        -180f, 180f);
                }
                joint.SetJointValue(val);
                currentHeight += setupHeight;
            }
            GUI.Label(new Rect(10 + instanceID * width, currentHeight, width, height), jaw.name);
            currentHeight += setupHeight;
            float jawval = jaw.defaultJointValue;
            jawval = GUI.HorizontalSlider(new Rect(10 + instanceID * width, currentHeight, width, height), jaw.currentJointValue,
                    jaw.jointLimit.x, jaw.jointLimit.y);
            
            
           
        }
#endif

    }
}
