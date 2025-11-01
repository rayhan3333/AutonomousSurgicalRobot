using System.Collections;
using System.Collections.Generic;
using DVRK;
using NUnit.Framework;
using UnityEngine;

public class EllipsoidTrajectory : MonoBehaviour
{
    private Vector3 ellipsoidCenter = new Vector3(1.051f, -.556f, 1.619f);
    //private Vector3 ellipsoidCenter = new Vector3(.326f, 0.0f, 1.714f);

    private Vector3 ellipsoidRadii = new Vector3(.45f / 2f, .74f / 2f, .59f / 2f);  // (rx, ry, rz)
    //private Vector3 ellipsoidRadii = new Vector3(.43f / 2f, .275f / 2f, .38f / 2f);  // (rx, ry, rz)


    public Transform PSMReturn;

    private Vector3 grabPoint;
    public int latitudeResolution = 20;
    public int longitudeResolution = 10; // how many longitudes to sweep

    public List<Vector3> positions = new List<Vector3>();
    public List<Quaternion> orientations = new List<Quaternion>();

    public PSM psmScript;

    public Transform ee;

    public JointPositionRecorder recorder;

    void Start()
    {
        grabPoint = ellipsoidCenter - new Vector3(0f, 0.15f, 0f);
        GenerateEllipsoidTrajectory();

    }

    void GenerateEllipsoidTrajectory()
{
    float dTheta = Mathf.PI / (latitudeResolution - 1); // θ from 0 to π
    float dPhi = 2 * Mathf.PI / longitudeResolution;    // φ from 0 to 2π

    positions.Clear();
    orientations.Clear();

    int forwardLookahead = 8;
    int latMinIndex = 4;                              // skip top 3 latitudes
    int latMaxIndex = latitudeResolution - 9;        // skip bottom 10 latitudes

    for (int lon = 0; lon < longitudeResolution; lon++)
    {


        float phi = lon * dPhi;
        bool descending = (lon % 2 == 0);
        int step = descending ? 1 : -1;
      
        // Consistent latitude range for both directions
        int latStart = descending ? latMinIndex : latMaxIndex;
        int latEnd = descending ? latMaxIndex : latMinIndex;

        for (int latIdx = latStart; descending ? (latIdx <= latEnd) : (latIdx >= latEnd); latIdx += step)
        {

            
            float theta = latIdx * dTheta;

            // Compute position
            float x = ellipsoidRadii.x * Mathf.Sin(theta) * Mathf.Cos(phi);
            float y = ellipsoidRadii.y * Mathf.Cos(theta);
            float z = ellipsoidRadii.z * Mathf.Sin(theta) * Mathf.Sin(phi);
            Vector3 point = ellipsoidCenter + new Vector3(x, y, z);

            if (lon == longitudeResolution - 1 && !descending && latIdx == 14)
            {
                Debug.Log("Generated pickup");

                positions.Add(Vector3.Lerp(point, grabPoint, 0.6f));
                Vector3 direction = (grabPoint - point).normalized;
                orientations.Add(Quaternion.LookRotation(direction, Vector3.up));
                positions.Add(PSMReturn.position);
                orientations.Add(Quaternion.LookRotation(direction, Vector3.up));
                return;
            }
            positions.Add(point);

            // Orientation lookahead
            if (latIdx < 7)
            {
                forwardLookahead = 3;
            }
            else
            {
                forwardLookahead = 8;
            }
            int lookaheadIdx = latIdx + (descending ? forwardLookahead : forwardLookahead);

            if (lookaheadIdx >= latMinIndex && lookaheadIdx <= latMaxIndex+8)
            {
                float thetaOffset = lookaheadIdx * dTheta;
                float xo = ellipsoidRadii.x * Mathf.Sin(thetaOffset) * Mathf.Cos(phi);
                float yo = ellipsoidRadii.y * Mathf.Cos(thetaOffset);
                float zo = ellipsoidRadii.z * Mathf.Sin(thetaOffset) * Mathf.Sin(phi);
                Vector3 futurePoint = ellipsoidCenter + new Vector3(xo, yo, zo);

                Vector3 forward = (futurePoint - point).normalized;
                orientations.Add(Quaternion.LookRotation(forward, Vector3.up));
            }
            else
            {
                // Fallback
                if (orientations.Count > 0)
                    orientations.Add(orientations[orientations.Count - 1]);
                else
                    orientations.Add(Quaternion.identity);
            }
        }
    }

    Debug.Log($"Generated {positions.Count} trajectory points with skipped top and bottom latitudes.");
}
    void OnDrawGizmos()
    {
        if (positions == null || orientations == null) return;

        for (int i = 0; i < positions.Count; i++)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(positions[i], 0.01f);

            // Draw forward vector
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(positions[i], orientations[i] * Vector3.forward * 0.05f);
        }
    }
    int index = 0;
    public int tries = 0;
    void Update()
    {
        if (tries > 1000)
        {
            Debug.Log("failed");
            index++;
            tries = 0;

        }
        if (index == positions.Count)
        {
            return;
        }
        psmScript.targetPosition = positions[index];
        psmScript.gizmo.position = positions[index];
        psmScript.targetOrientation = orientations[index];
        psmScript.gizmo.rotation = orientations[index];

        if (Vector3.Distance(psmScript.independentJoints[3].transform.position, psmScript.targetPosition) < 0.02f)
        {
            //Debug.Log("Position Satisfied");
            if (Vector3.Angle(psmScript.jaw.transform.rotation * Vector3.forward, psmScript.targetOrientation * Vector3.forward) < 3f)
            {
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

                index++;

            }
            else
            {
                //Debug.Log("Angle: " + Quaternion.Angle(psmScript.jaw.transform.rotation, psmScript.targetOrientation));
            }
        }
        else
        {
            tries += 1;

        }

        
    }
}
