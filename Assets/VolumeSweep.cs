using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Standalone gizmo visualizer that records the world poses of all Box/Capsule colliders
/// under a tool root, then draws a swept volume colored by overlap with tumor/healthy.
/// Optionally draws tangency rays against a provided surface mesh (mesh + transform).
/// </summary>
public class VolumeSweep : MonoBehaviour
{
    [Header("Tool to observe")]
    public Transform toolRoot;

    [Header("Sampling")]
    [Tooltip("Seconds between samples, e.g., 0.05 = 20 Hz")]
    [Min(0.005f)] public float sampleInterval = 0.05f;
    [Tooltip("How many seconds of history to keep for swept-volume gizmos")]
    [Min(0.5f)] public float historySeconds = 5f;

    [Header("Swept Volume Visualization")]
    [Range(1, 20)] public int samplesPerPathSegment = 4;
    [Range(1, 10)] public int gizmoStrideSnapshots = 1;
    [Range(0f, 1f)] public float volumeAlpha = 0.25f;

    [Header("Overlap Targets (coloring)")]
    public Collider tumorCollider;   // MeshCollider recommended
    public Collider healthyCollider; // MeshCollider recommended

    [Header("Optional Tangency (needs surface mesh for normals)")]
    public bool drawTangencyRays = false;
    [Tooltip("Reference surface used to compute normals for tangency scoring.")]
    public Mesh referenceSurfaceMesh;
    public Transform referenceSurfaceTransform;
    [Range(0.01f, 0.1f)] public float tangencyRayLength = 0.03f;
    [Range(1, 10)] public int tangencyStride = 5; // draw every Nth interpolated sample
    [Tooltip("Use the capsule axis for tangency measurement (Capsules) or tool forward (Boxes).")]
    public Vector3 boxForwardLocal = Vector3.forward; // used for boxes

    // --- colors ---
    Color _freeCol, _tumorCol, _healthyCol, _bothCol;

    void OnValidate()
    {
        _freeCol = new Color(0.6f, 0.6f, 0.6f, volumeAlpha); // gray
        _tumorCol = new Color(1.0f, 0.5f, 0.0f, volumeAlpha); // orange
        _healthyCol = new Color(1.0f, 0.0f, 0.0f, volumeAlpha); // red
        _bothCol = new Color(1.0f, 0.0f, 1.0f, volumeAlpha); // magenta
    }

    // ===========================
    // Internal data structures
    // ===========================
    [Serializable] public enum SegmentShape { Capsule, Box }

    [Serializable]
    public struct SegmentPose
    {
        public SegmentShape shape;
        public string name;

        // Pose (world)
        public Vector3 position;   // for capsule: center of segment; for box: center
        public Quaternion rotation;

        // Capsule specifics
        public float capsuleRadius;
        public float capsuleLength;
        public Vector3 capsuleAxisWS; // normalized

        // Box specifics
        public Vector3 boxSize; // world (lossyScale applied)
    }

    [Serializable]
    public class Snapshot
    {
        public float time;
        public List<SegmentPose> segments = new List<SegmentPose>(16);
    }

    // Rolling history
    readonly List<Snapshot> _history = new List<Snapshot>(256);
    float _nextSampleTime;

    // ===========================
    // Recording
    // ===========================
    void Update()
    {
        if (!toolRoot) return;
        if (Time.time >= _nextSampleTime)
        {
            _nextSampleTime = Time.time + sampleInterval;
            CaptureSnapshot();
            CullOldSnapshots();
        }
    }

    void CaptureSnapshot()
    {
        var snap = new Snapshot { time = Time.time };

        // Capsules
        var caps = toolRoot.GetComponentsInChildren<CapsuleCollider>(includeInactive: false);
        foreach (var cc in caps)
        {
            if (!cc.enabled) continue;

            var tr = cc.transform;

            // Axis in local, per collider direction (0=X,1=Y,2=Z)
            Vector3 axisLocal = (cc.direction == 0 ? Vector3.right :
                                (cc.direction == 1 ? Vector3.up : Vector3.forward));
            Vector3 axisWS = (tr.rotation * axisLocal).normalized;

            // World radius: pick larger scale among the two axes orthogonal to the capsule’s axis
            Vector3 s = AbsVec(tr.lossyScale);
            float worldRadius =
                (cc.direction == 0) ? cc.radius * Mathf.Max(s.y, s.z) :
                (cc.direction == 1) ? cc.radius * Mathf.Max(s.x, s.z) :
                                      cc.radius * Mathf.Max(s.x, s.y);

            // World cylinder segment length: height minus the hemispheres (approx with lossy axis scale)
            float axisScale =
                (cc.direction == 0) ? s.x :
                (cc.direction == 1) ? s.y : s.z;

            float worldLength = Mathf.Max(0f, cc.height * axisScale - 2f * worldRadius);

            // World center of capsule geometry is transform position plus center offset in local -> world
            Vector3 centerWS = tr.TransformPoint(cc.center);

            var pose = new SegmentPose
            {
                shape = SegmentShape.Capsule,
                name = cc.name,
                position = centerWS,
                rotation = tr.rotation,
                capsuleAxisWS = axisWS,
                capsuleRadius = worldRadius,
                capsuleLength = worldLength
            };
            snap.segments.Add(pose);
        }

        // Boxes
        var boxes = toolRoot.GetComponentsInChildren<BoxCollider>(includeInactive: false);
        foreach (var bc in boxes)
        {
            if (!bc.enabled) continue;

            var tr = bc.transform;

            var pose = new SegmentPose
            {
                shape = SegmentShape.Box,
                name = bc.name,
                position = tr.TransformPoint(bc.center),
                rotation = tr.rotation,
                boxSize = Vector3.Scale(bc.size, AbsVec(tr.lossyScale))
            };
            snap.segments.Add(pose);
        }

        _history.Add(snap);
    }

    void CullOldSnapshots()
    {
        float cutoff = Time.time - historySeconds;
        int keepFrom = 0;
        for (; keepFrom < _history.Count; keepFrom++)
            if (_history[keepFrom].time >= cutoff) break;
        if (keepFrom > 0) _history.RemoveRange(0, keepFrom);
    }

    // ===========================
    // Gizmos drawing
    // ===========================
    void OnDrawGizmos()
    {
        if (_history == null || _history.Count < 2) return;

        int stride = Mathf.Max(1, gizmoStrideSnapshots);
        int samples = Mathf.Max(1, samplesPerPathSegment);

        for (int i = 0; i <= _history.Count - 2; i += stride)
        {
            var A = _history[i];
            var B = _history[i + 1];
            if (A.segments.Count != B.segments.Count) continue; // require stable collider set between samples

            for (int segIndex = 0; segIndex < A.segments.Count; segIndex++)
            {
                var aSeg = A.segments[segIndex];
                var bSeg = B.segments[segIndex];

                for (int k = 0; k < samples; k++)
                {
                    float u = (samples == 1) ? 0f : (float)k / (samples - 1);

                    // Interpolated pose
                    Vector3 pos = Vector3.LerpUnclamped(aSeg.position, bSeg.position, u);
                    Quaternion rot = Quaternion.SlerpUnclamped(aSeg.rotation, bSeg.rotation, u);

                    if (aSeg.shape == SegmentShape.Capsule)
                    {
                        float radius = Mathf.Lerp(aSeg.capsuleRadius, bSeg.capsuleRadius, u);
                        float length = Mathf.Lerp(aSeg.capsuleLength, bSeg.capsuleLength, u);
                        Vector3 axisWS = Vector3.Slerp(aSeg.capsuleAxisWS, bSeg.capsuleAxisWS, u).normalized;

                        Vector3 a = pos - axisWS * (length * 0.5f);
                        Vector3 b = pos + axisWS * (length * 0.5f);

                        bool hitTumor = OverlapCapsuleSpecific(a, b, radius, tumorCollider);
                        bool hitHealthy = OverlapCapsuleSpecific(a, b, radius, healthyCollider);

                        Gizmos.color = PickColor(hitTumor, hitHealthy);
                        DrawCapsuleGizmo(a, b, radius);

                        // Optional tangency vs reference surface (every Nth sample)
                        if (drawTangencyRays && (k % Mathf.Max(1, tangencyStride) == 0))
                        {
                            if (TryClosestOnSurface(pos, out Vector3 cp, out Vector3 n))
                            {
                                float tangency = 1f - Mathf.Abs(Vector3.Dot(axisWS, n.normalized)); // 1 = perfect tangent
                                Color rayCol = Color.Lerp(Color.red, Color.blue, Mathf.Clamp01(tangency));
                                Gizmos.color = rayCol;
                                Gizmos.DrawRay(pos, axisWS * tangencyRayLength);

                                // (Optional) draw normal to show reference
                                Gizmos.color = new Color(0f, 1f, 0f, 0.85f);
                                Gizmos.DrawRay(cp, n.normalized * (tangencyRayLength * 0.6f));
                            }
                        }
                    }
                    else // Box
                    {
                        Vector3 size = Vector3.Lerp(aSeg.boxSize, bSeg.boxSize, u);
                        Vector3 half = size * 0.5f;

                        bool hitTumor = OverlapBoxSpecific(pos, half, rot, tumorCollider);
                        bool hitHealthy = OverlapBoxSpecific(pos, half, rot, healthyCollider);

                        Gizmos.color = PickColor(hitTumor, hitHealthy);
                        DrawBoxGizmo(pos, size, rot);

                        // Optional tangency for boxes (use chosen local forward as "motion axis")
                        if (drawTangencyRays && (k % Mathf.Max(1, tangencyStride) == 0))
                        {
                            if (TryClosestOnSurface(pos, out Vector3 cp, out Vector3 n))
                            {
                                Vector3 fwd = (rot * boxForwardLocal).normalized;
                                float tangency = 1f - Mathf.Abs(Vector3.Dot(fwd, n.normalized));
                                Color rayCol = Color.Lerp(Color.red, Color.blue, Mathf.Clamp01(tangency));
                                Gizmos.color = rayCol;
                                Gizmos.DrawRay(pos, fwd * tangencyRayLength);

                                Gizmos.color = new Color(0f, 1f, 0f, 0.85f);
                                Gizmos.DrawRay(cp, n.normalized * (tangencyRayLength * 0.6f));
                            }
                        }
                    }
                }
            }
        }
    }

    // ===========================
    // Physics overlap helpers
    // ===========================
    static bool OverlapCapsuleSpecific(Vector3 a, Vector3 b, float r, Collider target)
    {
        if (!target) return false;
        var hits = Physics.OverlapCapsule(a, b, r, ~0, QueryTriggerInteraction.Ignore);
        if (hits == null || hits.Length == 0) return false;
        foreach (var h in hits) if (h == target) return true;
        return false;
    }

    static bool OverlapBoxSpecific(Vector3 center, Vector3 halfExtents, Quaternion rot, Collider target)
    {
        if (!target) return false;
        var hits = Physics.OverlapBox(center, halfExtents, rot, ~0, QueryTriggerInteraction.Ignore);
        if (hits == null || hits.Length == 0) return false;
        foreach (var h in hits) if (h == target) return true;
        return false;
    }

    // ===========================
    // Surface normal via CPU triangle scan
    // ===========================
    bool TryClosestOnSurface(Vector3 q, out Vector3 closestPointWS, out Vector3 normalWS)
    {
        closestPointWS = q;
        normalWS = Vector3.up;

        if (!referenceSurfaceMesh || !referenceSurfaceTransform) return false;

        var verts = referenceSurfaceMesh.vertices;
        var tris = referenceSurfaceMesh.triangles;
        Matrix4x4 M = referenceSurfaceTransform.localToWorldMatrix;

        // Precompute world verts once per frame for speed? (kept simple here)
        // For typical gizmo usage this is fine; optimize if needed.
        float bestSqr = float.PositiveInfinity;
        Vector3 bestPt = q;
        Vector3 bestN = Vector3.up;

        for (int i = 0; i < tris.Length; i += 3)
        {
            Vector3 a = M.MultiplyPoint3x4(verts[tris[i]]);
            Vector3 b = M.MultiplyPoint3x4(verts[tris[i + 1]]);
            Vector3 c = M.MultiplyPoint3x4(verts[tris[i + 2]]);

            Vector3 cp = ClosestPointOnTriangle(q, a, b, c);
            float d2 = (cp - q).sqrMagnitude;
            if (d2 < bestSqr)
            {
                bestSqr = d2;
                bestPt = cp;
                bestN = Vector3.Normalize(Vector3.Cross(b - a, c - a));
            }
        }

        closestPointWS = bestPt;
        normalWS = bestN;
        return true;
    }

    // Christer Ericson’s closest point on triangle
    static Vector3 ClosestPointOnTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 ab = b - a, ac = c - a, ap = p - a;
        float d1 = Vector3.Dot(ab, ap), d2 = Vector3.Dot(ac, ap);
        if (d1 <= 0f && d2 <= 0f) return a;

        Vector3 bp = p - b;
        float d3 = Vector3.Dot(ab, bp), d4 = Vector3.Dot(ac, bp);
        if (d3 >= 0f && d4 <= d3) return b;

        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0f && d1 >= 0f && d3 <= 0f) return a + (d1 / (d1 - d3)) * ab;

        Vector3 cp = p - c;
        float d5 = Vector3.Dot(ab, cp), d6 = Vector3.Dot(ac, cp);
        if (d6 >= 0f && d5 <= d6) return c;

        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0f && d2 >= 0f && d6 <= 0f) return a + (d2 / (d2 - d6)) * ac;

        float va = d3 * d6 - d5 * d4;
        if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
        {
            float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b + w * (c - b);
        }

        float denom = 1f / (va + vb + vc);
        float v2 = vb * denom;
        float w2 = vc * denom;
        return a + ab * v2 + ac * w2;
    }

    // ===========================
    // Drawing helpers
    // ===========================
    static readonly int[] _boxEdges = new int[]
    {
        0,1, 1,3, 3,2, 2,0,   4,5, 5,7, 7,6, 6,4,   0,4, 1,5, 2,6, 3,7
    };

    static void DrawBoxGizmo(Vector3 center, Vector3 size, Quaternion rot)
    {
        Vector3 he = size * 0.5f;
        Vector3[] c = new Vector3[8];
        c[0] = new Vector3(-he.x, -he.y, -he.z);
        c[1] = new Vector3(he.x, -he.y, -he.z);
        c[2] = new Vector3(-he.x, -he.y, he.z);
        c[3] = new Vector3(he.x, -he.y, he.z);
        c[4] = new Vector3(-he.x, he.y, -he.z);
        c[5] = new Vector3(he.x, he.y, -he.z);
        c[6] = new Vector3(-he.x, he.y, he.z);
        c[7] = new Vector3(he.x, he.y, he.z);
        for (int i = 0; i < 8; i++) c[i] = center + rot * c[i];
        for (int e = 0; e < _boxEdges.Length; e += 2) Gizmos.DrawLine(c[_boxEdges[e]], c[_boxEdges[e + 1]]);
    }

    static void DrawCapsuleGizmo(Vector3 a, Vector3 b, float r)
    {
        Gizmos.DrawSphere(a, r);
        Gizmos.DrawSphere(b, r);

        Vector3 axis = b - a;
        float len = axis.magnitude;
        if (len < 1e-6f) return;
        Vector3 dir = axis / len;

        Vector3 t = Vector3.Cross(dir, Vector3.up);
        if (t.sqrMagnitude < 1e-6f) t = Vector3.Cross(dir, Vector3.right);
        t.Normalize();
        Vector3 s = Vector3.Cross(dir, t);

        int rings = 6, segments = 16;
        for (int i = 0; i <= rings; i++)
        {
            float v = (float)i / rings;
            Vector3 c = Vector3.Lerp(a, b, v);
            Vector3 prev = c + r * (t * Mathf.Cos(0) + s * Mathf.Sin(0));
            for (int j = 1; j <= segments; j++)
            {
                float ang = 2f * Mathf.PI * j / segments;
                Vector3 pt = c + r * (t * Mathf.Cos(ang) + s * Mathf.Sin(ang));
                Gizmos.DrawLine(prev, pt);
                prev = pt;
            }
        }
    }

    Color PickColor(bool hitTumor, bool hitHealthy)
    {
        if (hitTumor && hitHealthy) return _bothCol;
        if (hitHealthy) return _healthyCol;
        if (hitTumor) return _tumorCol;
        return _freeCol;
    }

    static Vector3 AbsVec(Vector3 v) => new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z));
}
