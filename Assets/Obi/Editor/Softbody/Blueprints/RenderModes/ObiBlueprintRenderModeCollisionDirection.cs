using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

namespace Obi
{
    public class ObiBlueprintRenderModeCollisionDirection : ObiBlueprintRenderMode
    {
        public override string name
        {
            get { return "Collision direction"; }
        }

        public ObiBlueprintRenderModeCollisionDirection(ObiSoftbodySurfaceBlueprintEditor editor) : base(editor)
        {
        }

        public override void OnSceneRepaint(SceneView sceneView)
        {
            using (new Handles.DrawingScope(Color.red, Matrix4x4.identity))
            {
                var blueprint = (ObiSoftbodySurfaceBlueprint)editor.blueprint;

                List<Vector3> lines = new List<Vector3>();

                for (int i = 0; i < blueprint.activeParticleCount; ++i)
                {
                    lines.Add(blueprint.positions[i]);
                    lines.Add(blueprint.positions[i] - blueprint.restOrientations[i] * blueprint.restNormals[i] * blueprint.restNormals[i].w);
                }

                Handles.DrawLines(lines.ToArray());
            }
        }

    }
}