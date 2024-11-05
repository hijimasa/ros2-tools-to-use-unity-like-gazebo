using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

using Unity.Robotics.Core;

public class JointStatePub : MonoBehaviour
{
    public string topicName = "/joint_states";
    public int jointLength = 19;  
    float time;

    public ArticulationBody[] articulationBodies;
    private ROSConnection ros;

    public string frameId = "";
    public string[] jointName = new string[] {};
    public double[] position = new double[] {};
    public double[] velocity = new double[] {};
    public double[] effort = new double[] {};

    // Set Parameters
    public float stiffness = 10000;
    public float damping = 100;
    public float forceLimit = 100;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(topicName, 15);

        for (int i = 0; i < jointLength; i++)
        {
            Debug.Log("Bodies index:"+articulationBodies[i].index);
            SetParameters(articulationBodies[i]);
            Debug.Log("articulation param:"+articulationBodies[i]);
        }
    }

    void FixedUpdate()
    {
        time += Time.deltaTime;
        if (time<0.05f) return;
        time = 0.0f;
        var timestamp = new TimeStamp(Clock.Now);

        ArticulationDrive xDrive = this.articulationBodies[1].xDrive;

        if (Input.GetKey(KeyCode.Z)){
            xDrive.target -= 1f;
        }
        else if (Input.GetKey(KeyCode.X)){
            xDrive.target += 1f;      
        }

        JointStateMsg joint_msg = new JointStateMsg{
            header = new HeaderMsg
            {
                frame_id = frameId,
                stamp = new TimeMsg{
                    sec = timestamp.Seconds,
                    nanosec = timestamp.NanoSeconds,
                },
            },
            name = jointName,
            position = position,
            velocity = velocity,
            effort = effort,
        };

        this.articulationBodies[1].xDrive = xDrive;  
        joint_msg.position[1] = xDrive.target/Mathf.Rad2Deg;
        Debug.Log("pos:"+joint_msg.position[1]);
        ros.Publish(topicName, joint_msg);
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
