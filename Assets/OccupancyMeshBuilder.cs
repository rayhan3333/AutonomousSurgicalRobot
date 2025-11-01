using System;
using System.Collections.Generic;
using UnityEngine;

/// Build a swept volume from BoxColliders sampled over time.
/// - Call RecordSample() each time step (or when tool moved).
/// - Call BuildHullMesh() to create the convex hull mesh of all recorded points.
/// - Call ClassifyAgainstColliders() to split triangles into submeshes:
///     0 = neutral, 1 = intersects tumor, 2 = intersects healthy
/// Assign 3 materials on the MeshRenderer to color them differently.
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class OccupancyMeshBuilder : MonoBehaviour
{
    [Header("Tool segments to sample")]
    public List<BoxCollider> toolBoxes = new List<BoxCollider>();

    [Header("Intersection targets (optional)")]
    public Collider[] tumorColliders;
    public Collider[] healthyColliders;

    [Header("Gizmo")]
    public bool drawRecordedPoints = false;
    public float pointGizmoSize = 0.002f;

    // --- internal ---
    private readonly List<Vector3> _recordedPoints = new List<Vector3>();
    private MeshFilter _mf;
    private MeshRenderer _mr;
    private Mesh _mesh;

    void Awake()
    {
        _mf = GetComponent<MeshFilter>();
        _mr = GetComponent<MeshRenderer>();
        if (_mf.sharedMesh == null) { _mesh = new Mesh { name = "ToolOccupancy" }; _mf.sharedMesh = _mesh; }
        else { _mesh = _mf.sharedMesh; }
    }

    /// Record all 8 world corners from each BoxCollider at this instant.
    public void RecordSample()
    {
        for (int i = 0; i < toolBoxes.Count; i++)
        {
            var box = toolBoxes[i];
            if (!box) continue;

            Vector3 c = box.center;
            Vector3 h = 0.5f * box.size;

            // 8 corners in local space
            for (int sx = -1; sx <= 1; sx += 2)
                for (int sy = -1; sy <= 1; sy += 2)
                    for (int sz = -1; sz <= 1; sz += 2)
                    {
                        Vector3 local = c + new Vector3(sx * h.x, sy * h.y, sz * h.z);
                        Vector3 world = box.transform.TransformPoint(local);
                        _recordedPoints.Add(world);
                    }
        }
    }

    /// Clear all recorded points (start a fresh interval).
    public void ClearRecorded() => _recordedPoints.Clear();

    /// Build/update the convex hull mesh from recorded points.
    public void BuildHullMesh(float epsilon = 1e-6f)
    {
        if (_recordedPoints.Count < 4)
        {
            Debug.LogWarning("Not enough points to build a hull.");
            _mesh.Clear();
            return;
        }

        // Deduplicate a bit (optional)
        var pts = Deduplicate(_recordedPoints, 1e-6f);

        // QuickHull
        List<int> tris;
        var hullVerts = QuickHull3D.Build(pts, out tris, epsilon);
        if (hullVerts == null || tris == null || tris.Count < 3)
        {
            Debug.LogWarning("Hull failed; check point set.");
            _mesh.Clear();
            return;
        }

        _mesh.Clear();
        _mesh.SetVertices(hullVerts);
        _mesh.SetTriangles(tris, 0);  // single submesh for now
        _mesh.RecalculateNormals();
        _mesh.RecalculateBounds();
    }

    /// Split triangles into submeshes by collider membership:
    /// submesh 0 = neutral, 1 = inside any tumor collider, 2 = inside any healthy collider
    public void ClassifyAgainstColliders(float insideTol = 1e-4f, int layerMask = ~0)
    {
        if (_mesh == null || _mesh.vertexCount == 0) return;

        var verts = _mesh.vertices;
        var tris = _mesh.GetTriangles(0);

        List<int> trisNeutral = new List<int>();
        List<int> trisTumor = new List<int>();
        List<int> trisHealthy = new List<int>();

        for (int t = 0; t < tris.Length; t += 3)
        {
            int i0 = tris[t], i1 = tris[t + 1], i2 = tris[t + 2];
            Vector3 v0 = transform.TransformPoint(verts[i0]); // world space
            Vector3 v1 = transform.TransformPoint(verts[i1]);
            Vector3 v2 = transform.TransformPoint(verts[i2]);

            bool inTumor = IsAnyColliderContaining(tumorColliders, v0, insideTol, layerMask) ||
                           IsAnyColliderContaining(tumorColliders, v1, insideTol, layerMask) ||
                           IsAnyColliderContaining(tumorColliders, v2, insideTol, layerMask);

            bool inHealthy = IsAnyColliderContaining(healthyColliders, v0, insideTol, layerMask) ||
                             IsAnyColliderContaining(healthyColliders, v1, insideTol, layerMask) ||
                             IsAnyColliderContaining(healthyColliders, v2, insideTol, layerMask);

            if (inTumor && !inHealthy) { trisTumor.Add(i0); trisTumor.Add(i1); trisTumor.Add(i2); }
            else if (inHealthy && !inTumor) { trisHealthy.Add(i0); trisHealthy.Add(i1); trisHealthy.Add(i2); }
            else { trisNeutral.Add(i0); trisNeutral.Add(i1); trisNeutral.Add(i2); }
        }

        _mesh.subMeshCount = 3;
        _mesh.SetTriangles(trisNeutral, 0);
        _mesh.SetTriangles(trisTumor, 1);
        _mesh.SetTriangles(trisHealthy, 2);
        _mesh.RecalculateNormals(); // per-submesh normals OK for viz

        // Ensure you assign 3 materials on MeshRenderer:
        //   materials[0] = neutral (e.g., gray transparent)
        //   materials[1] = tumor-intersect (e.g., red)
        //   materials[2] = healthy-intersect (e.g., green)
    }

    // --- helpers ---

    List<Vector3> Deduplicate(List<Vector3> src, float tol)
    {
        float tol2 = tol * tol;
        var outPts = new List<Vector3>(src.Count);
        for (int i = 0; i < src.Count; i++)
        {
            Vector3 p = src[i];
            bool dup = false;
            for (int j = 0; j < outPts.Count; j++)
            {
                if ((outPts[j] - p).sqrMagnitude <= tol2) { dup = true; break; }
            }
            if (!dup) outPts.Add(p);
        }
        return outPts;
    }

    bool IsAnyColliderContaining(Collider[] cols, Vector3 p, float tol, int layerMask)
    {
        if (cols == null) return false;
        for (int i = 0; i < cols.Length; i++)
        {
            var c = cols[i]; if (!c) continue;
            if (PointInsideCollider(c, p, tol, layerMask)) return true;
        }
        return false;
    }

    // Practical point-in-collider:
    // - First try ClosestPoint: if distance ~0, we’re inside or on boundary (works for most colliders).
    // - If ambiguous (thin surfaces / non-convex), do an odd-intersections ray test against 'c' only.
    bool PointInsideCollider(Collider c, Vector3 p, float tol, int layerMask)
    {
        Vector3 q = c.ClosestPoint(p);
        if ((q - p).sqrMagnitude <= tol * tol) return true; // inside or on surface

        // Ray parity test
        Vector3 dir = Vector3.right;
        float maxDist = 1000f;
        p += Vector3.up * 1e-4f; // tiny offset to avoid grazing
        var hits = Physics.RaycastAll(new Ray(p, dir), maxDist, layerMask, QueryTriggerInteraction.Ignore);
        int count = 0;
        for (int i = 0; i < hits.Length; i++)
            if (hits[i].collider == c) count++;

        return (count % 2) == 1;
    }

    void OnDrawGizmos()
    {
        if (!drawRecordedPoints) return;
        Gizmos.color = Color.cyan;
        for (int i = 0; i < _recordedPoints.Count; i++)
            Gizmos.DrawSphere(_recordedPoints[i], pointGizmoSize);
    }
}

/// -------- Minimal QuickHull 3D (convex hull for visualization) --------
/// NOTE: good for a few thousand points; assumes general-position (no heavy degeneracy).
static class QuickHull3D
{
    // CHANGE: struct -> class
    class Face
    {
        public int a, b, c;          // indices into pts
        public Vector3 n;            // outward normal (world)
        public float d;              // plane offset (n·x + d = 0)
        public List<int> outside;    // point indices assigned to this face
    }

    public static List<Vector3> Build(List<Vector3> pts, out List<int> outTris, float eps = 1e-6f)
    {
        outTris = null;
        int n = pts.Count;
        if (n < 4) return null;

        // 1) Find initial tetrahedron (p0,p1,p2,p3)
        int iMinX = 0, iMaxX = 0;
        for (int i = 1; i < n; i++)
        {
            if (pts[i].x < pts[iMinX].x) iMinX = i;
            if (pts[i].x > pts[iMaxX].x) iMaxX = i;
        }
        if (iMinX == iMaxX) return null;

        int iMaxDistLine = -1; float maxLine2 = -1f;
        Vector3 a = pts[iMinX], b = pts[iMaxX];
        Vector3 ab = b - a; float ab2 = ab.sqrMagnitude + eps;
        for (int i = 0; i < n; i++)
        {
            if (i == iMinX || i == iMaxX) continue;
            float t = Vector3.Dot(pts[i] - a, ab) / ab2;
            Vector3 proj = a + Mathf.Clamp01(t) * ab;
            float d2 = (pts[i] - proj).sqrMagnitude;
            if (d2 > maxLine2) { maxLine2 = d2; iMaxDistLine = i; }
        }
        if (iMaxDistLine < 0) return null;

        int iMaxDistPlane = -1; float maxPlane = -1f;
        Vector3 c = pts[iMaxDistLine];
        Vector3 n0 = Vector3.Cross(ab, c - a).normalized;
        for (int i = 0; i < n; i++)
        {
            if (i == iMinX || i == iMaxX || i == iMaxDistLine) continue;
            float h = Mathf.Abs(Vector3.Dot(pts[i] - a, n0));
            if (h > maxPlane) { maxPlane = h; iMaxDistPlane = i; }
        }
        if (iMaxDistPlane < 0) return null;
        Vector3 d = pts[iMaxDistPlane];

        // initial simplex indices
        int i0 = iMinX, i1 = iMaxX, i2 = iMaxDistLine, i3 = iMaxDistPlane;

        // Build 4 faces, oriented outward from the tetra centroid
        Vector3 center = (pts[i0] + pts[i1] + pts[i2] + pts[i3]) * 0.25f;
        var faces = new List<Face>(8);
        AddFace(faces, pts, center, i0, i1, i2);
        AddFace(faces, pts, center, i0, i3, i1);
        AddFace(faces, pts, center, i0, i2, i3);
        AddFace(faces, pts, center, i1, i3, i2);

        // Assign remaining points to faces' outside lists
        var assigned = new bool[n];
        assigned[i0] = assigned[i1] = assigned[i2] = assigned[i3] = true;
        for (int i = 0; i < n; i++)
        {
            if (assigned[i]) continue;
            int best = -1; float bestDist = eps;
            for (int f = 0; f < faces.Count; f++)
            {
                float dist = Vector3.Dot(faces[f].n, pts[i]) + faces[f].d;
                if (dist > bestDist) { bestDist = dist; best = f; }
            }
            if (best >= 0)
            {
                faces[best].outside ??= new List<int>();   // SAFE INIT
                faces[best].outside.Add(i);                // OK now (Face is class)
            }
        }

        // Main loop
        var tmpFaces = new List<Face>();
        while (true)
        {
            int fIdx = -1, farIdx = -1; float farDist = eps;
            // pick face with farthest outside point
            for (int f = 0; f < faces.Count; f++)
            {
                var outList = faces[f].outside;
                if (outList == null || outList.Count == 0) continue;
                for (int k = 0; k < outList.Count; k++)
                {
                    int pi = outList[k];
                    float dist = Vector3.Dot(faces[f].n, pts[pi]) + faces[f].d;
                    if (dist > farDist)
                    {
                        farDist = dist; fIdx = f; farIdx = pi;
                    }
                }
            }
            if (fIdx < 0) break; // done

            // Find all faces visible from far point
            var visible = new HashSet<int>();
            var stack = new Stack<int>();
            stack.Push(fIdx);
            visible.Add(fIdx);
            while (stack.Count > 0)
            {
                int fi = stack.Pop();
                // approximate adjacency by checking all
                for (int fj = 0; fj < faces.Count; fj++)
                {
                    if (visible.Contains(fj)) continue;
                    if (ShareEdge(faces[fi], faces[fj]))
                    {
                        float dist = Vector3.Dot(faces[fj].n, pts[farIdx]) + faces[fj].d;
                        if (dist > eps) { visible.Add(fj); stack.Push(fj); }
                    }
                }
            }

            // Collect horizon edges (edges of visible faces not shared twice)
            var edgeCount = new Dictionary<(int, int), int>();
            foreach (int fi in visible)
            {
                var F = faces[fi];
                CountEdge(edgeCount, F.a, F.b);
                CountEdge(edgeCount, F.b, F.c);
                CountEdge(edgeCount, F.c, F.a);
            }
            var horizon = new List<(int, int)>();
            foreach (var kv in edgeCount)
                if (kv.Value == 1) horizon.Add(kv.Key);

            // Remove visible faces
            tmpFaces.Clear();
            for (int f = 0; f < faces.Count; f++)
                if (!visible.Contains(f)) tmpFaces.Add(faces[f]);
            faces.Clear(); faces.AddRange(tmpFaces);

            // Create new faces linking horizon to far point
            foreach (var e in horizon)
            {
                var newF = MakeOrientedFace(pts, center, e.Item1, e.Item2, farIdx);
                faces.Add(newF);
            }

            // Reassign outside points (simple but robust: recompute)
            Array.Fill(assigned, false);
            assigned[i0] = assigned[i1] = assigned[i2] = assigned[i3] = true;
            for (int f = 0; f < faces.Count; f++) faces[f].outside?.Clear();

            for (int i = 0; i < n; i++)
            {
                int best = -1; float bestDist = eps;
                for (int f = 0; f < faces.Count; f++)
                {
                    float dist = Vector3.Dot(faces[f].n, pts[i]) + faces[f].d;
                    if (dist > bestDist) { bestDist = dist; best = f; }
                }
                if (best >= 0)
                {
                    faces[best].outside ??= new List<int>();
                    faces[best].outside.Add(i);
                }
            }
        }

        // Output triangles and unique vertices
        var used = new HashSet<int>();
        foreach (var f in faces) { used.Add(f.a); used.Add(f.b); used.Add(f.c); }

        var map = new Dictionary<int, int>();
        var outVerts = new List<Vector3>(used.Count);
        int idx = 0;
        foreach (int i in used) { map[i] = idx++; outVerts.Add(pts[i]); }

        outTris = new List<int>(faces.Count * 3);
        foreach (var f in faces)
        {
            outTris.Add(map[f.a]); outTris.Add(map[f.b]); outTris.Add(map[f.c]);
        }
        return outVerts;
    }

    static void AddFace(List<Face> faces, List<Vector3> pts, Vector3 center, int ia, int ib, int ic)
    {
        faces.Add(MakeOrientedFace(pts, center, ia, ib, ic));
    }

    static Face MakeOrientedFace(List<Vector3> pts, Vector3 center, int ia, int ib, int ic)
    {
        Vector3 a = pts[ia], b = pts[ib], c = pts[ic];
        Vector3 n = Vector3.Normalize(Vector3.Cross(b - a, c - a));
        float d = -Vector3.Dot(n, a);
        // Ensure outward: center should be on the negative side
        if (Vector3.Dot(n, center) + d > 0f)
        {
            (ia, ib) = (ib, ia);
            n = -n; d = -d;
        }
        return new Face { a = ia, b = ib, c = ic, n = n, d = d, outside = null };
    }

    static bool ShareEdge(Face A, Face B)
    {
        int shared = 0;
        if (A.a == B.a || A.a == B.b || A.a == B.c) shared++;
        if (A.b == B.a || A.b == B.b || A.b == B.c) shared++;
        if (A.c == B.a || A.c == B.b || A.c == B.c) shared++;
        return shared >= 2;
    }

    static void CountEdge(Dictionary<(int, int), int> map, int i, int j)
    {
        var key = (Mathf.Min(i, j), Mathf.Max(i, j)); // undirected
        map.TryGetValue(key, out int v);
        map[key] = v + 1;
    }
}