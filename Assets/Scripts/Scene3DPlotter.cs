// Scene3DPlotter.cs
// Put in Assets/Scripts. Add to an empty GameObject, assign targets, press Play.
// It draws XYZ axes, a ground grid, wireframes for meshes, and optional trajectories.

// MatplotlibAxes3D.cs — axes with ticks + faint grid planes (XY, XZ, YZ)
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

public class Scene3DPlotter : MonoBehaviour
{
    [Header("Auto-size from these objects (optional)")]
    public List<Transform> targets;
    public bool includeChildren = true;

    [Header("Manual override (leave zero to auto)")]
    public Vector3 originWorld = Vector3.zero;   // custom origin; if zero + have bounds → uses bounds.min
    public Vector3 axisLengths = Vector3.zero;   // if zero → uses bounds.size

    [Header("Style")]
    [Range(0.001f, 0.05f)] public float axisLineWidth = 0.01f;
    [Range(0.001f, 0.05f)] public float tickSize = 0.02f;
    public Color xColor = Color.red, yColor = Color.green, zColor = Color.blue;
    public Color labelColor = Color.black;       // black pops on white scene background
    public string numericFormat = "G3";

    [Header("Ticks (per-axis)")]
    public int xTickCount = 6;
    public int yTickCount = 6;
    public int zTickCount = 6;
    public bool shareNiceTicks = false;

    [Header("Axis labels")]
    public string xLabel = "X";
    public string yLabel = "Y";
    public string zLabel = "Z";
    public float labelOffset = 0.04f;

    [Header("Faint grid planes")]
    public bool drawGridPlanes = true;
    [Tooltip("Grid line step as a fraction of the tick step (e.g., 0.5 = 2 minor lines per major).")]
    [Range(0.1f, 1f)] public float minorGridFactor = 0.5f;
    [Range(0.1f, 1f)] public float gridWidthScale = 0.5f;  // relative to axisLineWidth
    [Range(0f, 1f)] public float gridAlpha = 0.15f;        // faint
    public Color gridColor = new Color(0f, 0f, 0f, 1f);     // drawn with gridAlpha

    Transform _root;
    Material _lineMat;

    void Start() { Build(); }

    public void Build()
    {
        if (_root != null) DestroyImmediate(_root.gameObject);
        _root = new GameObject("MatplotlibAxes3D").transform;
        _root.SetParent(transform, false);

        _lineMat = new Material(Shader.Find("Sprites/Default"));

        // Bounds → origin + lengths
        Bounds b;
        bool haveBounds = TryComputeBounds(out b);
        Vector3 origin = (axisLengths == Vector3.zero && originWorld == Vector3.zero && haveBounds) ? b.min : originWorld;
        Vector3 lengths = (axisLengths == Vector3.zero && haveBounds) ? b.size : axisLengths;
        if (lengths == Vector3.zero) lengths = new Vector3(0.5f, 0.5f, 0.5f);

        // Nice steps
        Vector3 steps = ComputeTickSteps(lengths);
        Vector3 minorSteps = new Vector3(steps.x * minorGridFactor, steps.y * minorGridFactor, steps.z * minorGridFactor);

        // Axes + ticks + labels
        BuildAxis("X", origin, Vector3.right, lengths.x, steps.x, xTickCount, xColor, 0);
        BuildAxis("Y", origin, Vector3.up, lengths.y, steps.y, yTickCount, yColor, 1);
        BuildAxis("Z", origin, Vector3.forward, lengths.z, steps.z, zTickCount, zColor, 2);
        MakeText(xLabel, origin + Vector3.right * (lengths.x + labelOffset), labelColor);
        MakeText(yLabel, origin + Vector3.up * (lengths.y + labelOffset), labelColor);
        MakeText(zLabel, origin + Vector3.forward * (lengths.z + labelOffset), labelColor);

        // Faint grid planes (XY @ z=origin.z, XZ @ y=origin.y, YZ @ x=origin.x)
        if (drawGridPlanes)
        {
            Color gc = new Color(gridColor.r, gridColor.g, gridColor.b, gridAlpha);
            float w = Mathf.Max(0.0005f, axisLineWidth * gridWidthScale);

            DrawGridXY(origin, lengths, minorSteps.x, minorSteps.y, gc, w);
            DrawGridXZ(origin, lengths, minorSteps.x, minorSteps.z, gc, w);
            DrawGridYZ(origin, lengths, minorSteps.y, minorSteps.z, gc, w);
        }
    }

    // ---------- Bounds ----------
    bool TryComputeBounds(out Bounds worldBounds)
    {
        worldBounds = new Bounds();
        var list = new List<MeshFilter>();
        foreach (var t in targets)
        {
            if (!t) continue;
            if (includeChildren) list.AddRange(t.GetComponentsInChildren<MeshFilter>(true));
            else { var mf = t.GetComponent<MeshFilter>(); if (mf) list.Add(mf); }
        }
        list.RemoveAll(mf => mf.sharedMesh == null);
        if (list.Count == 0) return false;

        bool started = false;
        foreach (var mf in list)
        {
            var local = mf.sharedMesh.bounds;
            foreach (var c in BoundsCorners(local))
            {
                var w = mf.transform.TransformPoint(c);
                if (!started) { worldBounds = new Bounds(w, Vector3.zero); started = true; }
                else worldBounds.Encapsulate(w);
            }
        }
        worldBounds.Expand(worldBounds.size.magnitude * 0.02f);
        return true;
    }

    IEnumerable<Vector3> BoundsCorners(Bounds b)
    {
        Vector3 min = b.min, max = b.max;
        yield return new Vector3(min.x, min.y, min.z);
        yield return new Vector3(max.x, min.y, min.z);
        yield return new Vector3(min.x, max.y, min.z);
        yield return new Vector3(max.x, max.y, min.z);
        yield return new Vector3(min.x, min.y, max.z);
        yield return new Vector3(max.x, min.y, max.z);
        yield return new Vector3(min.x, max.y, max.z);
        yield return new Vector3(max.x, max.y, max.z);
    }

    // ---------- Steps ----------
    Vector3 ComputeTickSteps(Vector3 lengths)
    {
        if (shareNiceTicks)
        {
            float maxRange = Mathf.Max(lengths.x, Mathf.Max(lengths.y, lengths.z));
            float step = NiceStep(maxRange, Mathf.Max(xTickCount, Mathf.Max(yTickCount, zTickCount)));
            return new Vector3(step, step, step);
        }
        return new Vector3(
            NiceStep(lengths.x, xTickCount),
            NiceStep(lengths.y, yTickCount),
            NiceStep(lengths.z, zTickCount)
        );
    }

    float NiceStep(float range, int desiredTicks)
    {
        if (desiredTicks < 2) desiredTicks = 2;
        if (range <= 0f) return 0.1f;
        float rough = range / (desiredTicks - 1);
        float pow10 = Mathf.Pow(10f, Mathf.Floor(Mathf.Log10(rough)));
        float n = rough / pow10;
        float nice;
        if (n < 1.5f) nice = 1f;
        else if (n < 3f) nice = 2f;
        else if (n < 7f) nice = 5f;
        else nice = 10f;
        return nice * pow10;
    }

    // ---------- Axes + ticks ----------
    void BuildAxis(string name, Vector3 origin, Vector3 dir, float length, float step, int tickCount, Color c, int axisIndex)
    {
        MakeLine($"{name}_Axis", origin, origin + dir * length, c, axisLineWidth);

        // pick a stable perpendicular for tick marks
        Vector3 ortho = PickOrtho(dir);
        int n = Mathf.Max(2, tickCount);
        float maxVal = length;
        float s = step > 0f ? step : length / (n - 1);

        for (float d = 0f; d <= maxVal + 1e-6f; d += s)
        {
            Vector3 p = origin + dir * d;
            MakeLine($"{name}_tick_{d:F3}",
                     p - ortho * (tickSize * 0.5f),
                     p + ortho * (tickSize * 0.5f),
                     c, axisLineWidth * 0.8f);

            float value = d + GetOriginComponent(origin, axisIndex);
            var textPos = p + ortho * (tickSize * 1.2f);
            MakeText(value.ToString(numericFormat), textPos, labelColor);
        }
    }

    Vector3 PickOrtho(Vector3 dir)
    {
        Vector3 up = Vector3.up;
        if (Mathf.Abs(Vector3.Dot(dir, up)) > 0.9f) up = Vector3.right;
        Vector3 ortho = Vector3.Cross(dir, up).normalized;
        return (ortho.sqrMagnitude < 1e-6f) ? Vector3.right : ortho;
    }

    float GetOriginComponent(Vector3 v, int axis) => axis == 0 ? v.x : (axis == 1 ? v.y : v.z);

    // ---------- Grid planes ----------
    void DrawGridXY(Vector3 origin, Vector3 len, float stepX, float stepY, Color c, float w)
    {
        float xMax = len.x, yMax = len.y; float z = origin.z;
        for (float x = 0f; x <= xMax + 1e-6f; x += Mathf.Max(1e-6f, stepX))
            MakeLine("g_xy_x_" + x.ToString("F4"),
                new Vector3(origin.x + x, origin.y, z),
                new Vector3(origin.x + x, origin.y + yMax, z),
                c, w);
        for (float y = 0f; y <= yMax + 1e-6f; y += Mathf.Max(1e-6f, stepY))
            MakeLine("g_xy_y_" + y.ToString("F4"),
                new Vector3(origin.x, origin.y + y, z),
                new Vector3(origin.x + xMax, origin.y + y, z),
                c, w);
    }

    void DrawGridXZ(Vector3 origin, Vector3 len, float stepX, float stepZ, Color c, float w)
    {
        float xMax = len.x, zMax = len.z; float y = origin.y;
        for (float x = 0f; x <= xMax + 1e-6f; x += Mathf.Max(1e-6f, stepX))
            MakeLine("g_xz_x_" + x.ToString("F4"),
                new Vector3(origin.x + x, y, origin.z),
                new Vector3(origin.x + x, y, origin.z + zMax),
                c, w);
        for (float z = 0f; z <= zMax + 1e-6f; z += Mathf.Max(1e-6f, stepZ))
            MakeLine("g_xz_z_" + z.ToString("F4"),
                new Vector3(origin.x, y, origin.z + z),
                new Vector3(origin.x + xMax, y, origin.z + z),
                c, w);
    }

    void DrawGridYZ(Vector3 origin, Vector3 len, float stepY, float stepZ, Color c, float w)
    {
        float yMax = len.y, zMax = len.z; float x = origin.x;
        for (float y = 0f; y <= yMax + 1e-6f; y += Mathf.Max(1e-6f, stepY))
            MakeLine("g_yz_y_" + y.ToString("F4"),
                new Vector3(x, origin.y + y, origin.z),
                new Vector3(x, origin.y + y, origin.z + zMax),
                c, w);
        for (float z = 0f; z <= zMax + 1e-6f; z += Mathf.Max(1e-6f, stepZ))
            MakeLine("g_yz_z_" + z.ToString("F4"),
                new Vector3(x, origin.y, origin.z + z),
                new Vector3(x, origin.y + yMax, origin.z + z),
                c, w);
    }

    // ---------- Drawing helpers ----------
    void MakeLine(string name, Vector3 a, Vector3 b, Color c, float width)
    {
        var go = new GameObject(name);
        go.transform.SetParent(_root, false);
        var lr = go.AddComponent<LineRenderer>();
        lr.useWorldSpace = true;
        lr.positionCount = 2;
        lr.SetPositions(new[] { a, b });
        lr.material = _lineMat;
        lr.startColor = lr.endColor = c;
        lr.numCapVertices = 2; lr.numCornerVertices = 2;
        lr.widthMultiplier = 1f;
        lr.widthCurve = new AnimationCurve(new Keyframe(0, width), new Keyframe(1, width));
        lr.shadowCastingMode = ShadowCastingMode.Off;
        lr.receiveShadows = false;
    }

    void MakeText(string text, Vector3 pos, Color c)
    {
        var go = new GameObject("Label_" + text);
        go.transform.SetParent(_root, false);
        go.transform.position = pos;

        var tm = go.AddComponent<TextMesh>();
        tm.text = text;
        tm.color = c;
        tm.anchor = TextAnchor.MiddleLeft;
        tm.characterSize = 0.02f;
        tm.fontSize = 16;

        go.AddComponent<BillboardToCamera>();
    }

    public class BillboardToCamera : MonoBehaviour
    {
        void LateUpdate()
        {
            var cam = Camera.main;
            if (!cam) return;
            transform.rotation = Quaternion.LookRotation(transform.position - cam.transform.position, cam.transform.up);
        }
    }
}