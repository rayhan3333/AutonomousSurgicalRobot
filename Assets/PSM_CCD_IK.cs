using DVRK;
using System;
using UnityEngine;

/// <summary>
/// CCD IK for a 6-DoF PSM with joint axes = local Y for *all* joints.
/// Joint order (fixed):
///   0: Yaw (rev, local Y)
///   1: Pitch (rev, local Y)
///   2: Insertion (prismatic, local Y translation)
///   3: Tool Roll (rev, local Y)
///   4: Wrist Pitch (rev, local Y)
///   5: Wrist Yaw (rev, local Y)
///
/// - Iteratively reduces both position and orientation error.
/// - When distal joints (roll/wrist) move and shift the tip, CCD continues passing
///   corrections up the chain (rev + pris) to re-hit the target.
/// - Joints are driven via independentJoints[i].SetJointValue(val).
/// - Revolute values can be rad or deg (toggle). Prismatic is in meters.
///
/// Usage:
///   1) Assign jointFrames[i] = the Transform whose local Y is the joint axis origin+direction.
///   2) Assign independentJoints[i] in the same order.
///   3) Set tip (effector) and target.
///   4) Call Solve() each frame (or on demand).
/// </summary>
public class PSM_CCD_IK : MonoBehaviour
{
    [Header("Chain Setup (order must be 0..5 = Yaw,Pitch,Insertion,Roll,WristPitch,WristYaw)")]
    public Transform[] jointFrames = new Transform[6];          // axis origin; axis = frame.up
    //public IndependentJoint[] independentJoints = new IndependentJoint[6];
    public DVRK.PSM psmScript;

    [Tooltip("True for revolute joints; index 2 must be false (prismatic).")]
    public bool[] isRevolute = new bool[6] { true, true, false, true, true, true };

    [Header("End Effector & Target")]
    public Transform tip;               // tool tip frame (world)
    public Transform target;            // desired pose (world)

    [Header("Units / API")]
    [Tooltip("If true: send/expect degrees for revolute joints. Prismatic stays meters.")]
    public bool revoluteUseDegrees = false;

    [Header("Solver Settings")]
    [Min(1)] public int maxIterations = 40;
    [Tooltip("Stop if position error < this (meters).")]
    public float positionTolerance = 0.001f; // 1 mm
    [Tooltip("Stop if orientation error < this (degrees).")]
    public float orientationToleranceDeg = 1.5f;

    [Tooltip("Maximum per-iteration revolute step (degrees if revoluteUseDegrees, else radians).")]
    public float maxRevoluteStep = 4f * Mathf.Deg2Rad;
    [Tooltip("Maximum per-iteration prismatic step (meters).")]
    public float maxPrismaticStep = 0.005f; // 5 mm

    [Header("Weights (per joint)")]
    [Tooltip("How much each joint cares about POSITION error (0..1).")]
    public float[] positionWeights = new float[6] { 1f, 1f, 1f, 0.3f, 0.3f, 0.3f };
    [Tooltip("How much each joint cares about ORIENTATION error (0..1).")]
    public float[] orientationWeights = new float[6] { 0.1f, 0.1f, 0f, 1f, 1f, 1f };

    [Header("Orientation Matching Axes")]
    [Tooltip("Main axis to align with target (usually tool forward).")]
    public Vector3 effectorMainAxisLocal = Vector3.forward;
    [Tooltip("Secondary axis for roll alignment (e.g., tip up).")]
    public Vector3 effectorRollAxisLocal = Vector3.up;
    [Range(0f, 1f)] public float orientationMainWeight = 0.7f;
    [Range(0f, 1f)] public float orientationRollWeight = 0.3f;

    [Header("Joint Limits (optional, applied after each delta)")]
    public bool useLimits = false;
    public float[] minLimit = new float[6] { -90f, -45f, 0.0f, -180f, -90f, -90f }; // deg for rev, m for prism
    public float[] maxLimit = new float[6] { +90f, +45f, 0.24f, +180f, +90f, +90f }; // deg for rev, m for prism
    [Tooltip("Limits for revolute are interpreted in degrees if revoluteUseDegrees=true.")]
    public bool limitsUseDegrees = true;
    public float[] jointDir = new float[6] { 1, 1, -1, 1, 1, 1 };


    // ------------------------------- PUBLIC ENTRY -------------------------------
    public bool Solve()
    {
        if (!Validate()) return false;

        // Convert tolerances consistent with revoluteUseDegrees
        float maxRevStep = revoluteUseDegrees ? maxRevoluteStep * Mathf.Rad2Deg : maxRevoluteStep;
        float oriTol = orientationToleranceDeg;

        for (int it = 0; it < maxIterations; it++)
        {
            // Convergence check
            float posErr = (target.position - tip.position).magnitude;
            float oriErr = Quaternion.Angle(tip.rotation, target.rotation);
            if (posErr < positionTolerance && oriErr < oriTol) return true;

            // Walk joints from end to base (CCD sweep)
            for (int j = 5; j >= 0; j--)
            {
                Transform jf = jointFrames[j];
                if (!jf) continue;
                Vector3 axisWS = jf.up;                 // local Y axis in world
                Vector3 jointOrigin = jf.position;

                // --- POSITION term (revolute): rotate around axis to align vectors in the plane axis
                float posAngleDelta = 0f;
                if (positionWeights[j] > 1e-5f)
                {
                    // current and target vectors from joint to tip/target
                    Vector3 vCur = tip.position - jointOrigin;
                    Vector3 vTgt = target.position - jointOrigin;

                    // Project into plane orthogonal to axis
                    Vector3 pCur = Vector3.ProjectOnPlane(vCur, axisWS);
                    Vector3 pTgt = Vector3.ProjectOnPlane(vTgt, axisWS);
                    float lenCur = pCur.magnitude;
                    float lenTgt = pTgt.magnitude;

                    if (isRevolute[j] && lenCur > 1e-6f && lenTgt > 1e-6f)
                    {
                        posAngleDelta = positionWeights[j] * SignedAngleAroundAxis(pCur, pTgt, axisWS); // radians
                        posAngleDelta = ClampAbs(posAngleDelta, revoluteUseDegrees ? maxRevStep * Mathf.Deg2Rad : maxRevStep);
                    }
                    else if (!isRevolute[j])
                    {
                        // PRISMATIC: slide along axis to reduce distance along that axis
                        float deltaAlong = Vector3.Dot(target.position - tip.position, axisWS);
                        float step = Mathf.Clamp(positionWeights[j] * deltaAlong, -maxPrismaticStep, maxPrismaticStep);
                        ApplyDelta(j, step, isRevolute: false);
                        continue; // prismatic doesn't add orientation here
                    }
                }

                // --- ORIENTATION term (revolute only): rotate around axis to align effector axes
                float oriAngleDelta = 0f;
                if (isRevolute[j] && orientationWeights[j] > 1e-5f)
                {
                    Vector3 fCur = tip.TransformDirection(effectorMainAxisLocal);
                    Vector3 fTgt = target.TransformDirection(effectorMainAxisLocal);
                    Vector3 uCur = tip.TransformDirection(effectorRollAxisLocal);
                    Vector3 uTgt = target.TransformDirection(effectorRollAxisLocal);

                    float main = SignedAngleAroundAxis(fCur, fTgt, axisWS); // radians
                    float roll = SignedAngleAroundAxis(uCur, uTgt, axisWS); // radians

                    oriAngleDelta = orientationWeights[j] * (orientationMainWeight * main + orientationRollWeight * roll);
                    oriAngleDelta = ClampAbs(oriAngleDelta, revoluteUseDegrees ? maxRevStep * Mathf.Deg2Rad : maxRevStep);
                }

                // Total rotation delta for this joint (radians)
                float totalDelta = posAngleDelta + oriAngleDelta;

                // Convert to API units and apply
                if (Mathf.Abs(totalDelta) > 1e-9f)
                {
                    if (revoluteUseDegrees)
                        ApplyDelta(j, totalDelta * Mathf.Rad2Deg, isRevolute: true);
                    else
                        ApplyDelta(j, totalDelta, isRevolute: true);
                }
            }
        }
        // Not fully converged; still applied best effort
        return false;
    }

    // ------------------------------- INTERNALS -------------------------------

    void ApplyDelta(int idx, float delta, bool isRevolute)
    {
        var j = psmScript.independentJoints[idx];
        if (!j) return;
        float dir = (idx < jointDir.Length) ? jointDir[idx] : 1f;
        // Revolute: multiply by dir directly. Prismatic: only the sign matters.
        float signedDelta = isRevolute ? delta * dir
                                       : delta * Mathf.Sign(dir);

        float next = j.currentJointValue + signedDelta;
        Debug.Log("Idx: " + idx + "  Value: " + next);
        if (useLimits)
        {
            // Limits array is in degrees for revolute if limitsUseDegrees=true; prismatic in meters.
            if (isRevolute && !revoluteUseDegrees && limitsUseDegrees)
            {
                // Convert limits deg -> rad for clamping
                float minR = minLimit[idx] * Mathf.Deg2Rad;
                float maxR = maxLimit[idx] * Mathf.Deg2Rad;
                next = Mathf.Clamp(next, minR, maxR);
            }
            else
            {
                next = Mathf.Clamp(next, minLimit[idx], maxLimit[idx]);
            }
        }
        

        j.SetJointValue(next);

        // If your SetJointValue updates transforms asynchronously, you can force sync:
        // Physics.SyncTransforms();
    }

    bool Validate()
    {
        if (jointFrames == null || jointFrames.Length != 6) { Debug.LogError("PSM_CCD_IK: jointFrames[6] required."); return false; }
        if (psmScript.independentJoints == null || psmScript.independentJoints.Count != 6) { Debug.LogError("PSM_CCD_IK: independentJoints[6] required."); return false; }
        if (!tip || !target) { Debug.LogError("PSM_CCD_IK: Assign tip and target."); return false; }
        if (isRevolute == null || isRevolute.Length != 6 || isRevolute[2]) { Debug.LogError("PSM_CCD_IK: isRevolute[6] required; index 2 must be false (prismatic)."); return false; }
        return true;
    }

    static float ClampAbs(float x, float maxAbs) => Mathf.Clamp(x, -maxAbs, maxAbs);

    /// <summary> Signed angle from a?b around axis n (radians). </summary>
    static float SignedAngleAroundAxis(Vector3 a, Vector3 b, Vector3 n)
    {
        Vector3 ap = Vector3.ProjectOnPlane(a, n).normalized;
        Vector3 bp = Vector3.ProjectOnPlane(b, n).normalized;
        float dot = Mathf.Clamp(Vector3.Dot(ap, bp), -1f, 1f);
        float ang = Mathf.Acos(dot);
        float s = Mathf.Sign(Vector3.Dot(n, Vector3.Cross(ap, bp)));
        return ang * s;
    }

    void Start()
    {
        int index = 0;
        foreach (URDFJoint joint in psmScript.independentJoints) {
            jointFrames[index] = joint.transform;
            index++;
        }
    }

    void Update()
    {
        if (target != null)
        {
            //Debug.Log(target.position);
            Solve();
        }
        //Debug.Log("In update");

    }
}
