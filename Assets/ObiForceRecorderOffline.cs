using UnityEngine;
using Obi;
using System.Collections;
// ObiActor, ObiSolver, ObiNativeContactList
// 'Oni.Contact' lives in the Oni namespace in Obi. You can use fully-qualified Oni.Contact below.

[RequireComponent(typeof(ObiActor))]
public class ObiForceRecorder_Contacts : MonoBehaviour
{
    public enum Metric { Piercing, Shearing }
    public enum VertexPosition { AtMaxMetric, Current }

    [Header("Actor & topology")]
    public ObiActor actor;                 // auto if left null
    [Tooltip("Mesh with the same vertex count/order as the actor's particles (cloth/soft body).")]
    public Mesh referenceTopology;

    [Header("Output (optional)")]
    public MeshFilter outputFilter;        // auto-created if null
    public MeshRenderer outputRenderer;    // auto-created if null

    [Header("Display")]
    public Gradient piercingGradient;      // set nice red/yellow
    public Gradient shearingGradient;      // set nice blue/cyan
    public float clampAtNewton = 0f;       // 0 = auto scale to max recorded

    [Header("Force conversion")]
    [Tooltip("If <= 0, uses Time.fixedDeltaTime / solver.substeps.")]
    public float substepDeltaTime = -1f;

    // --- internals ---
    public ObiSolver solver;
    float[] maxPierce_actor;               // per actor-particle
    float[] maxShear_actor;
    Vector3[] posAtMaxPierce;              // world position when max occurred
    Vector3[] posAtMaxShear;

    Obi.ObiNativeIntList actorToSolver;                   // actor index -> solver index
    int[] solverToActor;                   // solver index -> actor index (-1 if not ours)

    void Reset()
    {
        actor = GetComponent<ObiActor>();
    }
    Coroutine initCo;
    bool hooked;

    void OnEnable()
    {
        if (!actor) actor = GetComponent<ObiActor>();
        Debug.Log("Enabled");

        // (Re)start a waiter that initializes as soon as the solver is ready.
        if (initCo != null) StopCoroutine(initCo);
        initCo = StartCoroutine(InitWhenSolverReady());
    }

    void OnDisable()
    {
        if (initCo != null) { StopCoroutine(initCo); initCo = null; }
        Unhook();
        solver = null;
    }

    IEnumerator InitWhenSolverReady()
    {
        // Wait until Obi sets actor.solver (can take a few frames).
        while (actor == null || actor.solver == null) yield return null;

        solver = actor.solver;
        Debug.Log($"Init with solver: {solver.name}");
        Allocate();
        Hook();
    }

    void Hook()
    {
        if (hooked || solver == null) return;
        solver.OnCollision += Solver_OnCollision;
        hooked = true;
    }

    void Unhook()
    {
        if (!hooked) return;
        if (solver != null) solver.OnCollision -= Solver_OnCollision;
        hooked = false;
    }

    void Allocate()
    {
        Debug.Log("Allocating");
        int apc = actor.particleCount;
        maxPierce_actor = new float[apc];
        maxShear_actor = new float[apc];
        posAtMaxPierce = new Vector3[apc];
        posAtMaxShear = new Vector3[apc];

        actorToSolver = actor.solverIndices;
        int spc = solver.positions.count;
        solverToActor = new int[spc];
        for (int i = 0; i < spc; i++) solverToActor[i] = -1;
        for (int ai = 0; ai < apc; ai++)
        {
            int si = actorToSolver[ai];
            if (si >= 0 && si < spc) solverToActor[si] = ai;
        }

        // default gradients if none provided
        if (piercingGradient == null)
        {
            piercingGradient = new Gradient();
            piercingGradient.SetKeys(
                new[] { new GradientColorKey(new Color(0.15f, 0, 0), 0f), new GradientColorKey(Color.red, 0.7f), new GradientColorKey(Color.yellow, 1f) },
                new[] { new GradientAlphaKey(1, 0), new GradientAlphaKey(1, 1) }
            );
        }
        if (shearingGradient == null)
        {
            shearingGradient = new Gradient();
            shearingGradient.SetKeys(
                new[] { new GradientColorKey(new Color(0, 0, 0.2f), 0f), new GradientColorKey(Color.blue, 0.6f), new GradientColorKey(Color.cyan, 1f) },
                new[] { new GradientAlphaKey(1, 0), new GradientAlphaKey(1, 1) }
            );
        }
    }



    // ---- This is the callback you referenced ----
    [SerializeField] float contactDistanceThreshold = 0.01f; // prune speculative contacts
   // [SerializeField] float substepDeltaTime = -1f;           // <=0 => auto

    void Solver_OnCollision(object sender, ObiNativeContactList contacts)
    {
        
        if (solver == null || actor == null) return;

        // impulse->force conversion window:
        float dt = (substepDeltaTime > 0f) ? substepDeltaTime
                                           : Time.fixedDeltaTime / Mathf.Max(1, solver.substeps);

        int count = contacts.count;
        for (int i = 0; i < count; i++)
        {
            Oni.Contact c = contacts[i];
            if (c.distance > contactDistanceThreshold) continue; // keep only real/near contacts

            // --- bodyA is a SIMPLEX index, get its particle(s):
            int start = solver.simplexCounts.GetSimplexStartAndSize(c.bodyA, out int simplexSize);
            if (simplexSize <= 0) continue;

            // Iterate all particles in the simplex (triangle/edge/point):
            for (int s = 0; s < simplexSize; s++)
            {
                int solverPi = solver.simplices[start + s];

                // Map to actor & local index:
                var pa = solver.particleToActor[solverPi]; // ObiSolver.ParticleInActor
                if (pa.actor != actor) continue;           // skip other actors (if any)

                int ai = pa.indexInActor;                  // index into actor arrays

                // Estimate forces from impulses during this substep:
                float pierceN = Mathf.Max(0f, c.normalImpulse) / dt;
                float shearN = new Vector2(c.tangentImpulse, c.bitangentImpulse).magnitude / dt;

                //Debug.Log("Pierce: " + pierceN);
                //Debug.Log("Shear: " + shearN);

                // Particle world position (good proxy for the contact point on the particle side):
                Vector4 p4 = solver.positions[solverPi];
                Vector3 pw = solver.transform.TransformPoint(new Vector3(p4.x, p4.y, p4.z));

                if (pierceN > maxPierce_actor[ai]) { maxPierce_actor[ai] = pierceN; posAtMaxPierce[ai] = pw; }
                if (shearN > maxShear_actor[ai]) { maxShear_actor[ai] = shearN; posAtMaxShear[ai] = pw; }
            }
        }
    }

    public void ResetRecording()
    {
        if (maxPierce_actor != null) System.Array.Clear(maxPierce_actor, 0, maxPierce_actor.Length);
        if (maxShear_actor != null) System.Array.Clear(maxShear_actor, 0, maxShear_actor.Length);
        // positions refill on next maxima
    }

    // -------- Bake a smooth shaded heatmap mesh you can view/export --------
    public Mesh BakeHeatmapMesh(Metric metric, VertexPosition where, Mesh existing = null, bool recalcNormals = true)
    {
        if (!referenceTopology)
        {
            Debug.LogError("Assign referenceTopology (same vertex count/order as actor.particleCount).");
            return null;
        }
        if (referenceTopology.vertexCount != actor.particleCount)
        {
            Debug.LogError($"Vertex count mismatch. Actor: {actor.particleCount}, Mesh: {referenceTopology.vertexCount}");
            return null;
        }

        int n = actor.particleCount;
        var verts = new Vector3[n];
        var cols = new Color[n];

        // Choose vertex positions
        if (where == VertexPosition.AtMaxMetric)
        {
            var src = (metric == Metric.Piercing) ? posAtMaxPierce : posAtMaxShear;
            for (int ai = 0; ai < n; ai++)
            {
                if (src[ai] != Vector3.zero)
                    verts[ai] = transform.InverseTransformPoint(src[ai]);
                else
                {
                    int si = actorToSolver[ai];
                    if (si >= 0)
                    {
                        Vector4 p4 = solver.positions[si];
                        Vector3 pw = solver.transform.TransformPoint(new Vector3(p4.x, p4.y, p4.z));
                        verts[ai] = transform.InverseTransformPoint(pw);
                    }
                    else
                        verts[ai] = referenceTopology.vertices[ai];
                }
            }
        }
        else
        {
            for (int ai = 0; ai < n; ai++)
            {
                int si = actorToSolver[ai];
                if (si >= 0)
                {
                    Vector4 p4 = solver.positions[si];
                    Vector3 pw = solver.transform.TransformPoint(new Vector3(p4.x, p4.y, p4.z));
                    verts[ai] = transform.InverseTransformPoint(pw);
                }
                else
                    verts[ai] = referenceTopology.vertices[ai];
            }
        }

        // Colors from max forces
        float[] data = (metric == Metric.Piercing) ? maxPierce_actor : maxShear_actor;
        float vmax = clampAtNewton > 0 ? clampAtNewton : 1f;
        if (clampAtNewton <= 0f)
            for (int i = 0; i < n; i++) if (data[i] > vmax) vmax = data[i];

        var grad = (metric == Metric.Piercing) ? piercingGradient : shearingGradient;
        for (int i = 0; i < n; i++)
        {
            float t = Mathf.Clamp01(data[i] / vmax);
            cols[i] = grad.Evaluate(t);
        }

        // Build mesh
        Mesh m = existing ?? new Mesh { name = $"Obi_{metric}_Heatmap" };
        m.indexFormat = (referenceTopology.vertexCount > 65535)
                        ? UnityEngine.Rendering.IndexFormat.UInt32
                        : UnityEngine.Rendering.IndexFormat.UInt16;
        m.SetVertices(verts);
        m.SetTriangles(referenceTopology.triangles, 0);
        if (recalcNormals) m.RecalculateNormals();   // smooth shading
        m.colors = cols;
        m.RecalculateBounds();

        // Ensure an output container
        if (!outputFilter)
        {
            var go = new GameObject($"{metric}_HeatmapMesh");
            go.transform.SetParent(transform, true);
            outputFilter = go.AddComponent<MeshFilter>();
            outputRenderer = go.AddComponent<MeshRenderer>();
        }
        outputFilter.sharedMesh = m;

        if (outputRenderer && outputRenderer.sharedMaterial == null)
        {
            // Use a shader that multiplies/albedo by vertex color; URP Unlit works for quick view.
            outputRenderer.sharedMaterial = new Material(Shader.Find("Universal Render Pipeline/Unlit"));
        }

        return m;
    }

    // ------- CONFIG knobs for projection -------
    [Header("Surface bake (particles -> surface vertices)")]
    [Tooltip("Max neighbors per vertex used when blending particle values.")]
    public int kNeighbors = 12;
    [Tooltip("Search radius in meters. If <= 0, uses global KNN only.")]
    public float searchRadius = 0.03f;
    [Tooltip("Weight = 1/(epsilon + dist^power). Higher -> sharper (more like max).")]
    public float distancePower = 2.0f;
    [Tooltip("If true, use MAX of neighbors instead of weighted average.")]
    public bool useMaxInsteadOfAverage = false;

    // Returns arrays of particle world positions and their scalar values (max force) for a chosen metric.
    // If usePositionsAtMax=true, uses the particle position at the moment the maximum occurred.
    void GetParticleField(Metric metric, bool usePositionsAtMax,
                          out Vector3[] partPosW, out float[] partVal)
    {
        int n = actor.particleCount;
        partPosW = new Vector3[n];
        partVal = new float[n];

        float[] srcVals = (metric == Metric.Piercing) ? maxPierce_actor : maxShear_actor;
        Vector3[] srcPos = (metric == Metric.Piercing) ? posAtMaxPierce : posAtMaxShear;

        for (int i = 0; i < srcVals.Length; i++) {
            Debug.Log("Value: " + srcVals[i]);
        }
        for (int ai = 0; ai < n; ai++)
        {
            partVal[ai] = srcVals[ai];

            if (usePositionsAtMax && srcPos[ai] != Vector3.zero)
            {
                partPosW[ai] = srcPos[ai];
            }
            else
            {
                // current world position (solver space -> world)
                int si = actorToSolver[ai];
                if (si >= 0)
                {
                    Vector4 p4 = solver.positions[si];
                    partPosW[ai] = solver.transform.TransformPoint(new Vector3(p4.x, p4.y, p4.z));
                }
                else partPosW[ai] = transform.TransformPoint(referenceTopology ? referenceTopology.vertices[ai] : Vector3.zero);
            }
        }
    }

    // Core blender: for each surface vertex (world), blend nearby particle values -> color
    void PaintHeatmapOnMeshVertices(Mesh surfaceMesh, Transform surfaceXform,
                                    Metric metric, bool usePositionsAtMax,
                                    Gradient grad, float clampAtNewtonOverride)
    {
        if (surfaceMesh == null) { Debug.LogError("Surface mesh is null."); return; }

        // 1) Gather particle field
        GetParticleField(metric, usePositionsAtMax, out var P, out var V);

        // Optional: compute auto clamp if not provided
        float vmax = (clampAtNewtonOverride > 0 ? clampAtNewtonOverride : 0f);
        if (vmax <= 0f)
        {
            for (int i = 0; i < V.Length; i++) if (V[i] > vmax) vmax = V[i];
            if (vmax <= 1e-6f) vmax = 1f;
        }

        // 2) Get surface vertices in WORLD space for distance queries
        var vertsL = surfaceMesh.vertices;
        int m = vertsL.Length;
        var colors = new Color[m];
        var vertsW = new Vector3[m];
        for (int i = 0; i < m; i++) vertsW[i] = surfaceXform.TransformPoint(vertsL[i]);

        // 3) Naive KNN / radius blend (good enough for a few 10Ks of particles)
        float rad2 = (searchRadius > 0f ? searchRadius * searchRadius : -1f);
        const float eps = 1e-6f;

        // Temporary storage per vertex
        int[] bestIdx = new int[Mathf.Max(1, kNeighbors)];
        float[] bestD2 = new float[bestIdx.Length];

        for (int vi = 0; vi < m; vi++)
        {
            Vector3 pw = vertsW[vi];

            // (a) collect neighbors
            int found = 0;
            for (int j = 0; j < bestIdx.Length; j++) { bestIdx[j] = -1; bestD2[j] = float.PositiveInfinity; }

            for (int pi = 0; pi < P.Length; pi++)
            {
                float d2 = (P[pi] - pw).sqrMagnitude;

                if (rad2 > 0f && d2 > rad2) continue; // outside radius

                // insert into top-K
                int slot = -1;
                float worst = -1f; int worstIdx = -1;
                for (int k = 0; k < bestIdx.Length; k++)
                {
                    if (bestIdx[k] < 0) { slot = k; break; }
                    if (bestD2[k] > worst) { worst = bestD2[k]; worstIdx = k; }
                }
                if (slot >= 0) { bestIdx[slot] = pi; bestD2[slot] = d2; found++; }
                else if (d2 < worst) { bestIdx[worstIdx] = pi; bestD2[worstIdx] = d2; }
            }

            // If radius found nothing, fall back to pure KNN (scan all and keep K best)
            if (found == 0 && rad2 > 0f)
            {
                for (int j = 0; j < bestIdx.Length; j++) { bestIdx[j] = -1; bestD2[j] = float.PositiveInfinity; }
                for (int pi = 0; pi < P.Length; pi++)
                {
                    float d2 = (P[pi] - pw).sqrMagnitude;
                    int slot = -1;
                    float worst = -1f; int worstIdx = -1;
                    for (int k = 0; k < bestIdx.Length; k++)
                    {
                        if (bestIdx[k] < 0) { slot = k; break; }
                        if (bestD2[k] > worst) { worst = bestD2[k]; worstIdx = k; }
                    }
                    if (slot >= 0) { bestIdx[slot] = pi; bestD2[slot] = d2; }
                    else if (d2 < worst) { bestIdx[worstIdx] = pi; bestD2[worstIdx] = d2; }
                }
            }

            // (b) aggregate -> value
            float val = 0f;
            if (useMaxInsteadOfAverage)
            {
                float maxv = 0f; bool any = false;
                for (int k = 0; k < bestIdx.Length; k++)
                {
                    int pi = bestIdx[k]; if (pi < 0) continue;
                    maxv = Mathf.Max(maxv, V[pi]); any = true;
                }
                val = any ? maxv : 0f;
            }
            else
            {
                float wsum = 0f, acc = 0f;
                for (int k = 0; k < bestIdx.Length; k++)
                {
                    int pi = bestIdx[k]; if (pi < 0) continue;
                    float d = Mathf.Sqrt(Mathf.Max(bestD2[k], eps));
                    float w = 1.0f / Mathf.Pow(eps + d, distancePower);
                    acc += w * V[pi]; wsum += w;
                }
                val = (wsum > 0f ? acc / wsum : 0f);
            }

            float t = Mathf.Clamp01(val / vmax);
            colors[vi] = grad.Evaluate(t);
        }

        // 4) Assign vertex colors (positions/normals untouched; use smooth shader)
        surfaceMesh.colors = colors;
    }

    // Bake onto a MeshFilter (static surface)
    public Mesh BakeHeatmapOntoSurface(MeshFilter surface, Metric metric,
                                       bool usePositionsAtMax, float clampOverride = 0f,
                                       bool recalcNormals = true)
    {
        if (!surface || !surface.sharedMesh) { Debug.LogError("No MeshFilter/surface mesh."); return null; }

        // Work on a copy so you don't overwrite the original asset
        Mesh m = Instantiate(surface.sharedMesh);
        m.name = $"{surface.sharedMesh.name}_{metric}_Heatmap";

        var grad = (metric == Metric.Piercing) ? piercingGradient : shearingGradient;
        PaintHeatmapOnMeshVertices(m, surface.transform, metric, usePositionsAtMax, grad, clampOverride);
        if (recalcNormals) m.RecalculateNormals();
        m.RecalculateBounds();

        surface.sharedMesh = m;
        EnsureColorMaterial(surface.GetComponent<MeshRenderer>());
        return m;
    }

    // Bake onto a SkinnedMeshRenderer (deformed surface). We bake current pose, color it, and assign to a MeshFilter child for viewing/export.
    public Mesh BakeHeatmapOntoSkinned(SkinnedMeshRenderer skinned, Metric metric,
                                       bool usePositionsAtMax, float clampOverride = 0f,
                                       bool recalcNormals = true)
    {
        if (!skinned) { Debug.LogError("No SkinnedMeshRenderer."); return null; }

        // Bake current pose to a new mesh (local to skinned.transform)
        var baked = new Mesh { name = $"{skinned.sharedMesh.name}_baked" };
        skinned.BakeMesh(baked, true);

        var grad = (metric == Metric.Piercing) ? piercingGradient : shearingGradient;
        PaintHeatmapOnMeshVertices(baked, skinned.transform, metric, usePositionsAtMax, grad, clampOverride);
        if (recalcNormals) baked.RecalculateNormals();
        baked.RecalculateBounds();

        // Display: put into/under this recorder for convenience
        if (!outputFilter)
        {
            var go = new GameObject($"{metric}_HeatmapSurface");
            go.transform.SetParent(transform, true);
            outputFilter = go.AddComponent<MeshFilter>();
            outputRenderer = go.AddComponent<MeshRenderer>();
        }
        outputFilter.sharedMesh = baked;
        EnsureColorMaterial(outputRenderer);
        return baked;
    }

    void EnsureColorMaterial(MeshRenderer mr)
    {
        if (!mr) return;
        if (mr.sharedMaterial == null)
            mr.sharedMaterial = new Material(Shader.Find("Universal Render Pipeline/Unlit"));
        // For URP Lit with vertex colors, use a ShaderGraph that multiplies BaseColor by Vertex Color.
    }
}