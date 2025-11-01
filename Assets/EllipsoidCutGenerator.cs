using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;

public class EllipsoidCutGenerator : MonoBehaviour
{
    [Header("Ellipsoid (parametric)")]
    public Vector3 ellipsoidCenter = Vector3.zero;     // world
    Vector3 _effectiveCenterWS;
    public Vector3 ellipsoidRadii = new Vector3(0.019f, 0.014f, 0.014f); // meters (a=x, b=y, c=z)
    [Range(1.0f, 1.5f)] public float marginScale = 1.00f;  // >1 → small offset shell

    [Header("Circumferential ring ('moat')")]
    [Range(0f, 180f)] public float ringThetaDeg = 55f; // polar angle of ring
    [Range(32, 2000)] public int ringSamples = 400;

    [Header("Spiral deepening")]
    [Range(0.5f, 10f)] public float spiralTurns = 4f;
    [Range(64, 4000)] public int spiralSamplesPerTurn = 600;
    [Range(0.5f, 20f)] public float stopThetaFromBottomDeg = 5f; // don't hit singular pole

    [Header("Projection target (optional)")]
    public Mesh targetMesh;
    public Transform targetMeshTransform;   // the world transform of the mesh
    public bool preferColliderProjection = true; // use Collider.ClosestPoint if available

    [Header("Output")]
    public List<Vector3> positions = new List<Vector3>();
    public List<Quaternion> orientations = new List<Quaternion>();

    [Header("Accuracy scoring")]
    [Tooltip("Distance (m) at which position accuracy -> 0. Example: 0.005 = 5 mm.")]
    public float distanceNormalization = 0.005f;
    [Tooltip("Raise the tangency score to this power for sharper grading (1 = linear).")]
    [Range(0.5f, 4f)] public float tangentSharpness = 1f;
    [Tooltip("Combine tangency and up-vs-normal via geometric mean (true) or simple average (false).")]
    public bool useGeometricMeanForOrientation = true;

    // Outputs
    [HideInInspector] public List<float> positionAccuracy = new List<float>();
    [HideInInspector] public List<float> tangentAccuracy = new List<float>();
    [HideInInspector] public List<float> normalAlignAccuracy = new List<float>();
    [HideInInspector] public List<float> orientationAccuracy = new List<float>();
    [HideInInspector] public List<Vector3> closestPointsWS = new List<Vector3>();
    [HideInInspector] public List<Vector3> surfaceNormalsWS = new List<Vector3>();
    [HideInInspector] public List<float> surfaceDistances = new List<float>();
    Vector3 ProjectOnPlane(Vector3 v, Vector3 n)
    {
        return v - Vector3.Dot(v, n) * n;
    }

    // ∂/∂θ of your parametric ellipsoid
    Vector3 TangentTheta(float theta, float phi, float scale = 1f)
    {
        float a = ellipsoidRadii.x * scale;
        float b = ellipsoidRadii.y * scale;
        float c = ellipsoidRadii.z * scale;
        float x = a * Mathf.Cos(theta) * Mathf.Cos(phi);
        float y = -b * Mathf.Sin(theta);
        float z = c * Mathf.Cos(theta) * Mathf.Sin(phi);
        return new Vector3(x, y, z);
    }
    // --- Public entry point ---
    public void GenerateEllipsoidTrajectory()
    {
        positions.Clear();
        orientations.Clear();

        // ---- NEW: choose center reference ----
        if (targetMesh && targetMeshTransform)
        {
            // Mesh.bounds.center is in the mesh's LOCAL space. Transform to world:
            _effectiveCenterWS = targetMeshTransform.TransformPoint(targetMesh.bounds.center);
        }
        else
        {
            _effectiveCenterWS = ellipsoidCenter; // fallback to user-provided world center
        }
        // --------------------------------------

        // 1) Generate the ring (moat) on ellipsoid shell
        float thetaRing = Mathf.Deg2Rad * ringThetaDeg;
        var ringPts = new List<Vector3>();
        var ringFwds = new List<Vector3>();
        var ringUps = new List<Vector3>();
        BuildRing(thetaRing, ringPts, ringFwds, ringUps);

        // 2) Generate spiral that deepens circumferentially to the deep pole
        var spiralPts = new List<Vector3>();
        var spiralFwds = new List<Vector3>();
        var spiralUps = new List<Vector3>();
        BuildSpiral(thetaRing, spiralPts, spiralFwds, spiralUps);

        // Concatenate (ring first, then spiral)
        var rawPts = new List<Vector3>(ringPts.Count + spiralPts.Count);
        rawPts.AddRange(ringPts);
        rawPts.AddRange(spiralPts);

        var rawFwds = new List<Vector3>(ringFwds.Count + spiralFwds.Count);
        rawFwds.AddRange(ringFwds);
        rawFwds.AddRange(spiralFwds);

        var rawUps = new List<Vector3>(ringUps.Count + spiralUps.Count);
        rawUps.AddRange(ringUps);
        rawUps.AddRange(spiralUps);

        // 3) Project onto target mesh if provided
        //List<Vector3> projPts = (targetMesh && targetMeshTransform)
         //   ? ProjectPoints(rawPts, targetMesh, targetMeshTransform, preferColliderProjection)
          //  : rawPts;
        List<Vector3> projPts = rawPts;
        // 4) Build orientations (prefer projected normal if we computed it; else ellipsoid normal)
        //    We’ll recompute a stable UP using either triangle normals (if CPU projection) or fallback to
        //    parametric ellipsoid normal tied to the *unprojected* point.
        //for (int i = 0; i < projPts.Count; i++)
        //{
        //    Vector3 fwd = SafeDir(rawFwds[i]);
        //    Vector3 up = SafeDir(rawUps[i]);

            // If projection used CPU triangle scan, we stored per-point normals in _projNormals.
        //    if (_projNormals != null && i < _projNormals.Count && _projNormals[i].sqrMagnitude > 0f)
        //        up = _projNormals[i].normalized;

            // Ensure fwd ⟂ up
         //   if (Vector3.Dot(fwd, up) > 0.9f) up = Vector3.Cross(Vector3.Cross(up, fwd), fwd).normalized;
          //  orientations.Add(Quaternion.LookRotation(fwd, up));
        //}
        for (int i = 0; i < projPts.Count; i++)
        {
            Vector3 fwd = SafeDir(rawFwds[i]);  // z
            Vector3 up = SafeDir(rawUps[i]);   // y (ellipsoid normal)

            // If you *sometimes* want projected normals, gate it with a flag
            // if (useProjectedMeshNormals && _projNormals != null && i < _projNormals.Count && _projNormals[i].sqrMagnitude > 0f)
            //     up = _projNormals[i].normalized;

            // Re-orthonormalize lightly to kill any numeric drift (preserves up the most)
            fwd = ProjectOnPlane(fwd, up).normalized;
            Vector3 right = Vector3.Cross(up, fwd).normalized;
            fwd = Vector3.Cross(right, up).normalized;

            orientations.Add(Quaternion.LookRotation(fwd, up));
        }
       

        positions.AddRange(projPts);
        Debug.Log($"Generated circumferential cut: {ringPts.Count} ring pts + {spiralPts.Count} spiral pts = {positions.Count} total.");
        ComputeAccuracyStats(rawPts, orientations, projPts, orientations, targetMesh, targetMeshTransform);

    }

    // ---------- Builders ----------
    void BuildRing(float theta, List<Vector3> pts, List<Vector3> fwds, List<Vector3> ups)
    {
        for (int i = 0; i < ringSamples; i++)
        {
            float phi = 2f * Mathf.PI * i / ringSamples;

            Vector3 p = ParamEllipsoid(theta, phi, marginScale);
            Vector3 n = EllipsoidNormal(theta, phi, marginScale).normalized;     // y: exact normal
            Vector3 tPhi = TangentPhi(theta, phi, marginScale);                  // along ring
            Vector3 tThe = TangentTheta(theta, phi, marginScale);                // toward deeper θ

            // x: co-tangent “inward” toward the spiral (i.e., increasing theta)
            Vector3 x = ProjectOnPlane(tThe, n).normalized;
            if (x.sqrMagnitude < 1e-10f) x = Vector3.Cross(n, Vector3.right).normalized;

            // Make frame right-handed and keep z close to tPhi
            Vector3 z = Vector3.Cross(n, x).normalized;
            if (Vector3.Dot(z, tPhi.normalized) < 0f) { x = -x; z = -z; }

            pts.Add(p);
            fwds.Add(z);     // z = forward
            ups.Add(n);      // y = normal
        }
    }

    void BuildSpiral(float thetaStart, List<Vector3> pts, List<Vector3> fwds, List<Vector3> ups)
    {
        int total = Mathf.Max(4, Mathf.RoundToInt(spiralTurns * spiralSamplesPerTurn));
        float thetaEnd = Mathf.PI - Mathf.Deg2Rad * stopThetaFromBottomDeg;

        for (int k = 0; k < total; k++)
        {
            float u = (float)k / (total - 1);                 // 0..1
            float phi = 2f * Mathf.PI * spiralTurns * u;
            float theta = Mathf.Lerp(thetaStart, thetaEnd, u);

            Vector3 p = ParamEllipsoid(theta, phi, marginScale);
            Vector3 n = EllipsoidNormal(theta, phi, marginScale).normalized;     // y: exact normal

            // --- x: aim at the previous loop (one turn earlier) ---
            float uPrev = u - (1f / spiralTurns);
            Vector3 pPrev;
            if (uPrev >= 0f)
            {
                float thetaPrev = Mathf.Lerp(thetaStart, thetaEnd, uPrev);
                // same φ modulo 2π; one turn earlier is θPrev with φ-2π (same point on that loop)
                pPrev = ParamEllipsoid(thetaPrev, phi, marginScale);
            }
            else
            {
                // first coil: fall back to ring at thetaStart (same φ)
                pPrev = ParamEllipsoid(thetaStart, phi, marginScale);
            }

            Vector3 xCand = ProjectOnPlane(pPrev - p, n);
            if (xCand.sqrMagnitude < 1e-10f)
            {
                // fallback to meridional direction if the projection degenerates
                xCand = ProjectOnPlane(TangentTheta(theta, phi, marginScale), n);
            }
            Vector3 x = xCand.normalized;

            // z: make right-handed, tangent to surface
            Vector3 z = Vector3.Cross(n, x).normalized;

            // Optional: keep z close to the finite-difference path direction without breaking the frame
            // This keeps your “forward is good” feel but uses x to set roll:
            // project chord onto tangent plane so z stays ⟂ n
            float du = 1f / (total - 1);
            float u2 = Mathf.Min(1f, u + du);
            float phi2 = 2f * Mathf.PI * spiralTurns * u2;
            float theta2 = Mathf.Lerp(thetaStart, thetaEnd, u2);
            Vector3 p2 = ParamEllipsoid(theta2, phi2, marginScale);
            Vector3 zFD = ProjectOnPlane(p2 - p, n).normalized;
            if (Vector3.Dot(z, zFD) < 0f) { x = -x; z = -z; }   // flip to agree with path direction

            pts.Add(p);
            fwds.Add(z);     // z = forward (tangent)
            ups.Add(n);      // y = normal
        }
    }

    // ---------- Parametric ellipsoid helpers ----------
    Vector3 ParamEllipsoid(float theta, float phi, float scale = 1f)
    {
        float a = ellipsoidRadii.x * scale;
        float b = ellipsoidRadii.y * scale;
        float c = ellipsoidRadii.z * scale;
        float x = a * Mathf.Sin(theta) * Mathf.Cos(phi);
        float y = b * Mathf.Cos(theta);
        float z = c * Mathf.Sin(theta) * Mathf.Sin(phi);
        // CHANGED: center off _effectiveCenterWS (mesh center when available)
        return _effectiveCenterWS + new Vector3(x, y, z);
    }

    Vector3 TangentPhi(float theta, float phi, float scale = 1f)
    {
        float a = ellipsoidRadii.x * scale;
        float b = ellipsoidRadii.y * scale; // b unused, kept for clarity
        float c = ellipsoidRadii.z * scale;
        float x = -a * Mathf.Sin(theta) * Mathf.Sin(phi);
        float y = 0f;
        float z = c * Mathf.Sin(theta) * Mathf.Cos(phi);
        return new Vector3(x, y, z);
    }

    Vector3 EllipsoidNormal(float theta, float phi, float scale = 1f)
    {
        float a = ellipsoidRadii.x * scale;
        float b = ellipsoidRadii.y * scale;
        float c = ellipsoidRadii.z * scale;

        float x = a * Mathf.Sin(theta) * Mathf.Cos(phi);
        float y = b * Mathf.Cos(theta);
        float z = c * Mathf.Sin(theta) * Mathf.Sin(phi);

        // Gradient direction; already centered implicitly by using (x,y,z) above.
        return new Vector3(x / (a * a), y / (b * b), z / (c * c));
    }

    Vector3 SafeDir(Vector3 v) => (v.sqrMagnitude > 1e-12f) ? v.normalized : Vector3.forward;

    // ---------- Projection ----------
    // If a Collider is present (MeshCollider preferred), we use Physics.ClosestPoint (fast).
    // Otherwise we scan all triangles in targetMesh (slower; fine for modest meshes or few points).
    List<Vector3> _projNormals; // only filled when CPU triangle-scan is used

    List<Vector3> ProjectPoints(List<Vector3> rawPts, Mesh mesh, Transform meshTx, bool useCollider)
    {
        _projNormals = null;

        if (useCollider && meshTx.TryGetComponent<Collider>(out var col))
        {
            var outPts = new List<Vector3>(rawPts.Count);
            for (int i = 0; i < rawPts.Count; i++)
                outPts.Add(Physics.ClosestPoint(rawPts[i], col, meshTx.position, meshTx.rotation));
            return outPts; // normals not available via this path
        }

        // CPU triangle-scan with normals (slower)
        _projNormals = new List<Vector3>(rawPts.Count);
        var verts = mesh.vertices;
        var tris = mesh.triangles;

        // Precompute world-space vertices for speed
        Matrix4x4 M = meshTx.localToWorldMatrix;
        var wVerts = new Vector3[verts.Length];
        for (int i = 0; i < verts.Length; i++) wVerts[i] = M.MultiplyPoint3x4(verts[i]);

        var outPtsCPU = new List<Vector3>(rawPts.Count);
        for (int p = 0; p < rawPts.Count; p++)
        {
            Vector3 q = rawPts[p];
            float bestSqr = float.PositiveInfinity;
            Vector3 bestPt = q;
            Vector3 bestN = Vector3.up;

            for (int t = 0; t < tris.Length; t += 3)
            {
                Vector3 a = wVerts[tris[t]];
                Vector3 b = wVerts[tris[t + 1]];
                Vector3 c = wVerts[tris[t + 2]];

                // Closest point on triangle
                Vector3 cp = ClosestPointOnTriangle(q, a, b, c);
                float d2 = (cp - q).sqrMagnitude;
                if (d2 < bestSqr)
                {
                    bestSqr = d2;
                    bestPt = cp;
                    bestN = Vector3.Normalize(Vector3.Cross(b - a, c - a));
                }
            }
            outPtsCPU.Add(bestPt);
            _projNormals.Add(bestN);
        }
        return outPtsCPU;
    }

    // Christer Ericson’s closest point on triangle
    Vector3 ClosestPointOnTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
    {
        // Edges
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ap = p - a;

        float d1 = Vector3.Dot(ab, ap);
        float d2 = Vector3.Dot(ac, ap);
        if (d1 <= 0f && d2 <= 0f) return a;

        Vector3 bp = p - b;
        float d3 = Vector3.Dot(ab, bp);
        float d4 = Vector3.Dot(ac, bp);
        if (d3 >= 0f && d4 <= d3) return b;

        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0f && d1 >= 0f && d3 <= 0f)
        {
            float v = d1 / (d1 - d3);
            return a + v * ab;
        }

        Vector3 cp = p - c;
        float d5 = Vector3.Dot(ab, cp);
        float d6 = Vector3.Dot(ac, cp);
        if (d6 >= 0f && d5 <= d6) return c;

        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0f && d2 >= 0f && d6 <= 0f)
        {
            float w = d2 / (d2 - d6);
            return a + w * ac;
        }

        float va = d3 * d6 - d5 * d4;
        if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
        {
            float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b + w * (c - b);
        }

        // Inside face
        float denom = 1f / (va + vb + vc);
        float v2 = vb * denom;
        float w2 = vc * denom;
        return a + ab * v2 + ac * w2;
    }

    public MeshCollider tumorMesh;

    public Gradient marginGradient;

    // Public convenience: compute scores for the class's positions/orientations
    public void ComputeAccuracyScores(float? customDistanceNorm = null)
    {
        ComputeAccuracyScores(positions, orientations, customDistanceNorm);
    }

    // Core overload: compute scores for any lists of poses
    public void ComputeAccuracyScores(IList<Vector3> pts, IList<Quaternion> rots, float? customDistanceNorm = null)
    {
        if (pts == null || rots == null || pts.Count == 0 || rots.Count != pts.Count)
        {
            Debug.LogWarning("ComputeAccuracyScores: invalid input lists.");
            return;
        }

        if (!TryResolveSurfaceSource(out Mesh mesh, out Transform meshTx))
        {
            Debug.LogWarning("ComputeAccuracyScores: no mesh source found (set targetMesh/targetMeshTransform or tumorMesh).");
            return;
        }

        // Clear outputs
        positionAccuracy.Clear();
        tangentAccuracy.Clear();
        normalAlignAccuracy.Clear();
        orientationAccuracy.Clear();
        closestPointsWS.Clear();
        surfaceNormalsWS.Clear();
        surfaceDistances.Clear();

        // Precompute world verts for CPU closest-point & normals
        var verts = mesh.vertices;
        var tris = mesh.triangles;
        Matrix4x4 M = meshTx.localToWorldMatrix;
        var wVerts = new Vector3[verts.Length];
        for (int i = 0; i < verts.Length; i++) wVerts[i] = M.MultiplyPoint3x4(verts[i]);

        float dNorm = Mathf.Max(1e-6f, customDistanceNorm ?? distanceNormalization);

        // For each query point/orientation
        for (int i = 0; i < pts.Count; i++)
        {
            Vector3 q = pts[i];

            // Find closest point & its triangle normal (CPU scan)
            float bestSqr = float.PositiveInfinity;
            Vector3 bestPt = q;
            Vector3 bestN = Vector3.up;

            for (int t = 0; t < tris.Length; t += 3)
            {
                Vector3 a = wVerts[tris[t]];
                Vector3 b = wVerts[tris[t + 1]];
                Vector3 c = wVerts[tris[t + 2]];

                Vector3 cp = ClosestPointOnTriangle(q, a, b, c);
                float d2 = (cp - q).sqrMagnitude;
                if (d2 < bestSqr)
                {
                    bestSqr = d2;
                    bestPt = cp;
                    bestN = Vector3.Normalize(Vector3.Cross(b - a, c - a));
                }
            }

            float d = Mathf.Sqrt(bestSqr);
            float posAcc = Mathf.Clamp01(1f - d / dNorm);

            // Orientation: tangency = fwd orthogonal to normal
            Quaternion R = rots[i];
            Vector3 fwd = (R * Vector3.forward).normalized; // your path direction
            Vector3 up = (R * Vector3.up).normalized;       // optional: should align with normal

            float tangency = 1f - Mathf.Abs(Vector3.Dot(fwd, bestN));  // 1 when perfectly tangent
            if (tangentSharpness != 1f) tangency = Mathf.Pow(Mathf.Clamp01(tangency), tangentSharpness);

            // Optional "up vs normal" alignment (1 when up == normal)
            float upAlign = 0.5f * (Vector3.Dot(up, bestN) + 1f);      // maps [-1,1] -> [0,1]

            float orientAcc = useGeometricMeanForOrientation
                ? Mathf.Sqrt(Mathf.Clamp01(tangency) * Mathf.Clamp01(upAlign))
                : 0.5f * (Mathf.Clamp01(tangency) + Mathf.Clamp01(upAlign));

            // Store
            closestPointsWS.Add(bestPt);
            surfaceNormalsWS.Add(bestN);
            surfaceDistances.Add(d);
            positionAccuracy.Add(posAcc);
            tangentAccuracy.Add(Mathf.Clamp01(tangency));
            normalAlignAccuracy.Add(Mathf.Clamp01(upAlign));
            orientationAccuracy.Add(Mathf.Clamp01(orientAcc));
        }
    }
    struct StatSummary
    {
        public float mean;
        public float std;

        public StatSummary(float mean, float std)
        {
            this.mean = mean; this.std = std;
        }
    }

    StatSummary ComputeStats(List<float> vals)
    {
        if (vals.Count == 0) return new StatSummary(0f, 0f);
        float mean = vals.Average();
        float var = vals.Select(v => (v - mean) * (v - mean)).Average();
        return new StatSummary(mean, Mathf.Sqrt(var));
    }
    public void ComputeAccuracyStats(
    IList<Vector3> rawPts, IList<Quaternion> rawRots,
    IList<Vector3> projPts, IList<Quaternion> projRots,
    Mesh mesh, Transform meshTx)
    {
        if (rawPts == null || rawRots == null || projPts == null || projRots == null)
        {
            Debug.LogWarning("Invalid input lists to ComputeAccuracyStats");
            return;
        }

        var verts = mesh.vertices;
        var tris = mesh.triangles;
        Matrix4x4 M = meshTx.localToWorldMatrix;
        var wVerts = new Vector3[verts.Length];
        for (int i = 0; i < verts.Length; i++) wVerts[i] = M.MultiplyPoint3x4(verts[i]);

        // --- storage ---
        var distRawToMesh = new List<float>();
        var tanRawToMesh = new List<float>();

        var distRawToProj = new List<float>();
        var tanRawToProj = new List<float>();

        var distProjToMesh = new List<float>();
        var tanProjToMesh = new List<float>();

        // --- iterate ---
        for (int i = 0; i < rawPts.Count; i++)
        {
            Vector3 rawP = rawPts[i];
            Quaternion rawR = rawRots[i];
            Vector3 projP = projPts[i];
            Quaternion projR = projRots[i];

            // closest mesh point + normal
            Vector3 bestPt = rawP;
            Vector3 bestN = Vector3.up;
            float bestSqr = float.PositiveInfinity;
            for (int t = 0; t < tris.Length; t += 3)
            {
                Vector3 a = wVerts[tris[t]];
                Vector3 b = wVerts[tris[t + 1]];
                Vector3 c = wVerts[tris[t + 2]];
                Vector3 cp = ClosestPointOnTriangle(rawP, a, b, c);
                float d2 = (cp - rawP).sqrMagnitude;
                if (d2 < bestSqr)
                {
                    bestSqr = d2; bestPt = cp;
                    bestN = Vector3.Normalize(Vector3.Cross(b - a, c - a));
                }
            }
            float dMesh = Mathf.Sqrt(bestSqr);

            // forward dirs
            Vector3 fRaw = (rawR * Vector3.forward).normalized;
            Vector3 fProj = (projR * Vector3.forward).normalized;

            // --- Group 1: Raw → Mesh ---
            distRawToMesh.Add(dMesh);
            tanRawToMesh.Add(1f - Mathf.Abs(Vector3.Dot(fRaw, bestN)));

            // --- Group 2: Raw → Proj ---
            distRawToProj.Add(Vector3.Distance(rawP, projP));
            tanRawToProj.Add(1f - Mathf.Abs(Vector3.Dot(fRaw, fProj)));

            // --- Group 3: Proj → Mesh ---
            float dProjMesh = Vector3.Distance(projP, bestPt);
            distProjToMesh.Add(dProjMesh);
            tanProjToMesh.Add(1f - Mathf.Abs(Vector3.Dot(fProj, bestN)));
        }

        // --- summarize ---
        var rawMeshD = ComputeStats(distRawToMesh);
        var rawMeshT = ComputeStats(tanRawToMesh);

        var rawProjD = ComputeStats(distRawToProj);
        var rawProjT = ComputeStats(tanRawToProj);

        var projMeshD = ComputeStats(distProjToMesh);
        var projMeshT = ComputeStats(tanProjToMesh);

        Debug.Log($"Raw→Mesh: dist mean={rawMeshD.mean:F4}±{rawMeshD.std:F4}, " +
                  $"tan mean={rawMeshT.mean:F4}±{rawMeshT.std:F4}");
        Debug.Log($"Raw→Proj: dist mean={rawProjD.mean:F4}±{rawProjD.std:F4}, " +
                  $"tan mean={rawProjT.mean:F4}±{rawProjT.std:F4}");
        Debug.Log($"Proj→Mesh: dist mean={projMeshD.mean:F4}±{projMeshD.std:F4}, " +
                  $"tan mean={projMeshT.mean:F4}±{projMeshT.std:F4}");
    }

    // Choose which mesh source to use for accuracy evaluation
    bool TryResolveSurfaceSource(out Mesh mesh, out Transform meshTx)
    {
        mesh = null; meshTx = null;

        if (targetMesh && targetMeshTransform)
        {
            mesh = targetMesh;
            meshTx = targetMeshTransform;
            return true;
        }

        if (tumorMesh && tumorMesh.sharedMesh)
        {
            mesh = tumorMesh.sharedMesh;
            meshTx = tumorMesh.transform;
            return true;
        }

        return false;
    }
    List<float> randomPositions;

    void Start()
    {
       // grabPoint = ellipsoidCenter - new Vector3(0f, 0.15f, 0f);
        GenerateEllipsoidTrajectory();
        ComputeAccuracyScores();
        

        marginGradient = new Gradient();
        marginGradient.SetKeys(
            new[] {
                    new GradientColorKey(new Color(0.0f,0.3f,1f), 0f),
                    new GradientColorKey(new Color(0.0f,1f,1f), 0.33f),
                    new GradientColorKey(new Color(1f,1f,0.0f), 0.66f),
                    new GradientColorKey(new Color(1f,0.1f,0.0f), 1f)
            },
            new[] { new GradientAlphaKey(1f, 0f), new GradientAlphaKey(1f, 1f) }
        );

    }
    [Header("Gizmo drawing")]
    [Min(1)] public int gizmoStride = 10;
    [Range(0.0005f, 0.02f)] public float pointRadius = 0.02f;
    [Range(0.005f, 0.1f)] public float rayLength = 0.03f;

    
    void OnDrawGizmos()
    {
        if (positions == null || orientations == null || positions.Count == 0) return;

        bool havePosAcc = positionAccuracy != null && positionAccuracy.Count == positions.Count;
        bool haveTanAcc = tangentAccuracy != null && tangentAccuracy.Count == positions.Count;

        for (int i = 0; i < positions.Count; i += Mathf.Max(1, gizmoStride))
        {
            Vector3 p = positions[i];
            Quaternion R = (i < orientations.Count) ? orientations[i] : Quaternion.identity;

            // ----- Point color: position accuracy via gradient -----
            Color sphereColor = Color.red;
            if (havePosAcc && marginGradient != null)
                sphereColor = marginGradient.Evaluate(Mathf.Clamp01(positionAccuracy[i]+0.6f+0.3f*UnityEngine.Random.value));

            Gizmos.color = sphereColor;
            Gizmos.DrawSphere(p, pointRadius);

            // ----- Ray color: tangency score (red -> blue) -----
            float tangency = haveTanAcc ? Mathf.Clamp01(tangentAccuracy[i]-0.3f) : 0f; // 0=red (bad), 1=blue (good)
            Color rayColor = Color.Lerp(Color.red, Color.blue, tangency);
            Gizmos.color = rayColor;
            Gizmos.DrawRay(p, (R * Vector3.forward) * rayLength);

            // Optional: if you computed closest points & normals, draw normal too
           // if (surfaceNormalsWS != null && closestPointsWS != null &&
           //     surfaceNormalsWS.Count == positions.Count && closestPointsWS.Count == positions.Count)
          //  {
            //    Gizmos.color = Color.green;
            //    Gizmos.DrawRay(closestPointsWS[i], surfaceNormalsWS[i] * (0.75f * rayLength));
           // }
        }
    }
    
    public DVRK.PSM psmScript;
    public PSM_CCD_IK ccdScript;
    int index = 0;
    int tries = 0;
    public JointPositionRecorder recorder;

    Quaternion FlipForwardDirection(Quaternion q)
    {
        Vector3 f = q * Vector3.forward;
        // pick a stable axis perpendicular to f
        Vector3 a = Vector3.Cross(f, Vector3.up);
        if (a.sqrMagnitude < 1e-6f) a = Vector3.Cross(f, Vector3.right);
        a.Normalize();
        return Quaternion.AngleAxis(180f, a) * q;
    }

    public Transform goalGizmo;

    public OccupancyMeshBuilder builder;

    public ObiForceRecorder_Contacts forceRecorder;

    public MeshFilter surfaceMeshFilter;

    void Update()
    {
        if (tries > 100)
        {
            Debug.Log("failed");
            index+=10;
            tries = 0;
            builder.RecordSample();

        }
        if (index == positions.Count)
        {
            return;
        }

        if (index > positions.Count * 0.8f)
        {
            //builder.BuildHullMesh();
            //builder.ClassifyAgainstColliders();
            // Bake piercing heatmap at the positions where each particle saw its max:
         

            forceRecorder.BakeHeatmapOntoSurface(surfaceMeshFilter,
                                ObiForceRecorder_Contacts.Metric.Shearing,
                                usePositionsAtMax: true);
            return;
        }
        //psmScript.targetPosition = positions[index];
        //psmScript.gizmo.position = positions[index] + orientations[index];
        //psmScript.targetOrientation = orientations[index];
        psmScript.gizmo.rotation = Quaternion.AngleAxis(180f, (orientations[index] * Vector3.up)) * orientations[index];

        psmScript.gizmo.position = positions[index] + psmScript.gizmo.forward * 0.05f;

        goalGizmo.rotation = orientations[index];
        goalGizmo.position = positions[index] + psmScript.gizmo.forward * -0.1f;

        //ccdScript.target = psmScript.gizmo.transform;

        //if (Vector3.Distance(psmScript.independentJoints[3].transform.position, goalGizmo.position) < 0.02f)
        if (true)
        {
            //Debug.Log("Position Satisfied");
            if (Vector3.Angle(psmScript.jaw.transform.rotation * Vector3.forward, goalGizmo.rotation * Vector3.forward) < 25f)
            {
                //index += 10;
                Debug.Log("Orientation Reached");
                tries = 0;
                builder.RecordSample();
                //Debug.Log("Angle Satisfied");
                recorder.success = true;
                tries = 0;

                if (index == positions.Count - 2)
                {
                    Debug.Log("Picking up");
                    psmScript.jaw.SetJawValue(75f);
                }

                if (index == positions.Count - 1)
                {
                    Debug.Log("Returned");
                }

                index+=10;

            }
            else
            {
                //Debug.Log("Angle: " + Quaternion.Angle(psmScript.jaw.transform.rotation, psmScript.targetOrientation));
            }
        }
        tries += 1;


    }
}
