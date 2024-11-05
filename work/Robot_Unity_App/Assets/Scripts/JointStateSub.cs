using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSub : MonoBehaviour
{
    public ArticulationBody[] articulationBodies;
    public string[] jointNames;
    public string topicName = "/joint_states";
    public int jointLength = 19;
    private List<string> jointNamesList;
    private ROSConnection ros;

    // Set Parameters
    public float stiffness = 0F;
    public float damping = 10000F;
    public float forceLimit = 100F;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>(topicName, Callback);

        for (int i = 0; i < jointLength; i++)
        {
            Debug.Log("Bodies index:"+articulationBodies[i].index);
            SetParameters(articulationBodies[i]);
            Debug.Log("articulation param:"+articulationBodies[i]);
        }
        
        jointNamesList = new List<string>(jointNames);
    }

    void Callback(JointStateMsg msg)
    {
        int index;
        for (int i = 0; i < msg.name.Length; i++)
        {
            index = jointNamesList.IndexOf(msg.name[i]);
            if (index != -1)
            {
                ArticulationDrive aDrive = articulationBodies[index].xDrive;
                if (i < msg.position.Length)
                    aDrive.target = Mathf.Rad2Deg * (float)msg.position[i];
                if (i < msg.velocity.Length)
                    aDrive.targetVelocity = Mathf.Rad2Deg * (float)msg.velocity[i];
                float effort = float.NaN;
                if (i < msg.effort.Length)
                    effort = (float) msg.effort[i];
                articulationBodies[index].xDrive = aDrive;
            }
        }
    }

    private void SetParameters(ArticulationBody joint)
    {
        ArticulationDrive drive = joint.xDrive;
        // drive.lowerLimit = -30;
        // drive.upperLimit = 30;
        drive.stiffness = stiffness;
        drive.damping = damping;
        drive.forceLimit = forceLimit;
        // drive.target = 0;
        // drive.targetVelocity = 0;

        joint.xDrive = drive;
    }
}
