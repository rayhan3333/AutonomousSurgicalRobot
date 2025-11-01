using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using DVRK;
using Unity.VisualScripting.Antlr3.Runtime;

public class JointPositionRecorder : MonoBehaviour
{
    // public URDFJoint[] joints; // Your independent joints
    // public URDFJoint jaw;
    private StreamWriter writer;
    private float logInterval = 0.1f; // 10 Hz
    private float nextLogTime = 0f;
    public PSM psmScript;

    public List<float[]> storedJointPositions = new List<float[]>();

    public bool success = false;


    void Start()
    {
        string path = Application.dataPath + "/joint_log4.csv";
        writer = new StreamWriter(path, false, Encoding.UTF8);
        writer.WriteLine("time,j0,j1,j2,j3,j4,j5,jaw"); // header
    }

    void Update()
    {
        // if (Time.time >= nextLogTime)
        // {
        //     nextLogTime += logInterval;

        //     float[] jointPositions = new float[6];
        //     for (int i = 0; i < 6; i++)
        //     {
        //         jointPositions[i] = psmScript.independentJoints[i].currentJointValue;
        //     }
        //     storedJointPositions.Add(jointPositions);
        // }

        // if (success)
        // {
        //     Debug.Log("Writing positions");
        //     foreach (float[] joints in storedJointPositions)
        //     {
        //         string line = Time.time.ToString("F2");

        //         foreach (float value in joints)
        //         {
        //             line += "," + value.ToString("F4");
        //         }
        //         line += "," + psmScript.jaw.currentJointValue.ToString("F4");
        //         writer.WriteLine(line);

        //     }
        //     storedJointPositions.Clear();
        //     success = false;
        // }

        if (success)
        {
            Debug.Log("Writing positions");
            string line = Time.time.ToString("F2");

            foreach (URDFJoint joint in psmScript.independentJoints)
            {

                line += "," + joint.currentJointValue.ToString("F4");


            }
            line += "," + psmScript.jaw.currentJointValue.ToString("F4");

            writer.WriteLine(line);

            success = false;
        }

    
    }

    void OnApplicationQuit()
    {
        
        writer.Flush();
        writer.Close();
    }
}
