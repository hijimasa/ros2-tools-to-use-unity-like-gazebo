using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

[InitializeOnLoad]
[ExecuteInEditMode]
public class RemoteCommandListener : MonoBehaviour
{
    private static RemoteCommandListener instance = null;
    private TcpListener listener = null;
    private TcpClient client = null;
    private bool isRunning = false;

    private class StartUpData : ScriptableSingleton<StartUpData>
    {
        [SerializeField]
        private int _callCount;
        public bool IsStartUp()
        {
            return _callCount++ == 0;
        }
    }

    static RemoteCommandListener()
    {
        // エディタが完全に起動した後に初期化する
        EditorApplication.delayCall += RemoteCommandListener.Initialize;
    }

    private static void Initialize()
    {
        // StartUpDataの初期化をInitialize内で行う
        if (!StartUpData.instance.IsStartUp())
            return;

        // 既に存在する場合、再作成しない
        if (instance != null) return;

        GameObject obj = new GameObject("RemoteCommandListener");
        instance = obj.AddComponent<RemoteCommandListener>();
        
        EditorApplication.delayCall -= RemoteCommandListener.Initialize;
    }

    private async void Start()
    {
        if (isRunning) return; // 多重起動防止
        isRunning = true;

        if (listener != null)
        {
            listener.Stop();
            listener = null;
        }
        listener = new TcpListener(IPAddress.Any, 5000);
        listener.Start();
        Debug.Log("Listening for URDF import commands...");

        try
        {
            if (client != null)
            {
                client.Close();
                client = null;
            }
            while (isRunning)
            {
                // listenerが停止された場合はループを終了
                if (!listener.Server.IsBound)
                    break;

                client = await listener.AcceptTcpClientAsync();
                if (client != null)
                {
                    HandleClient(client);
                }
            }
        }
        catch (ObjectDisposedException)
        {
            Debug.Log("Listener was stopped, ending listening loop.");
        }
        finally
        {
            // listenerが停止されている場合、終了処理を行う
            listener?.Stop();
        }
    }

    private async void HandleClient(TcpClient client)
    {
        NetworkStream stream = client.GetStream();
        byte[] buffer = new byte[1024];
        int bytesRead = await stream.ReadAsync(buffer, 0, buffer.Length);
        string commandString = Encoding.UTF8.GetString(buffer, 0, bytesRead);
        string[] commands = commandString.Split(' ');

        string urdfFilePath = commands[1];

        Debug.Log("Received URDF path: " + urdfFilePath);
        ImportSettings settings = new ImportSettings();
        GameObject robotObject = UrdfRobotExtensions.CreateRuntime(urdfFilePath, settings);

        JointStateSub jointStateSub = robotObject.AddComponent<JointStateSub>();
        List<GameObject> childObjectsWithArticulationBody = FindArticulationBodyObjectsInChildren(robotObject);
        List<ArticulationBody> articulationBodyList = new List<ArticulationBody>();
        List<string> jointNameList = new List<string>();
        foreach (GameObject child in childObjectsWithArticulationBody)
        {
            ArticulationBody body = child.GetComponent<ArticulationBody>();
            Debug.Log("Received joint type: " + body.jointType);
            if (System.Enum.IsDefined(typeof(ArticulationJointType), body.jointType))
            {
                if (body.jointType != ArticulationJointType.FixedJoint)
                {
                    UrdfJoint urdfJoint = child.GetComponent<UrdfJoint>();
                    articulationBodyList.Add(body);
                    jointNameList.Add(urdfJoint.name);
                }
            }
        }
        jointStateSub.articulationBodies = articulationBodyList.ToArray();
        jointStateSub.jointNames = jointNameList.ToArray();
        jointStateSub.jointLength = articulationBodyList.Count;

/*        
        GameObject rosConnector = GameObject.Find("RosConnectorObject");
        
        UrdfRobot robot = robotObject.GetComponent<UrdfRobot>();
        robot.SetRigidbodiesIsKinematic(false);
        robot.SetCollidersConvex(true);
        robot.SetUseUrdfInertiaData(true);
        robot.SetRigidbodiesUseGravity(true);

       // 一番上にあるUrdfLinkコンポーネントにIsBaseLinkを設定
        List<GameObject> childObjectsWithUrdfLink = GetChildObjectsWithComponent<UrdfLink>(robotObject);
        foreach (GameObject child in childObjectsWithUrdfLink)
        {
            UrdfLink link = child.GetComponent<UrdfLink>();
            link.IsBaseLink = true;
            break;
        }

        HingeJoint[] HingeJoints = FindObjectsOfType<HingeJoint>();
        foreach (HingeJoint joint in HingeJoints)
        {
            //JointSpring spring = joint.spring;
            //spring.spring = 1000;
            //spring.damper = 1000;
            //spring.targetPosition = 0;
            //joint.spring = spring;
            joint.extendedLimits = true;
            joint.useAcceleration = true;
            joint.enablePreprocessing = false;
        }

        FixedJoint[] FixedJoints = FindObjectsOfType<FixedJoint>();
        foreach (FixedJoint joint in FixedJoints)
        {
            joint.enablePreprocessing = false;
        }

        // Create JointStatePublisher & JointStateSubscriber
        JointStatePatcher jointStatePatcherInstance = rosConnector.AddComponent<JointStatePatcher>();
        jointStatePatcherInstance.UrdfRobot = robotObject.GetComponent<UrdfRobot>();
        jointStatePatcherInstance.SetPublishJointStates(true);
        jointStatePatcherInstance.SetSubscribeJointStates(true);
        JointStatePublisher jointStatePublisher = rosConnector.GetComponent<JointStatePublisher>();
        jointStatePublisher.Topic = "/" + robotName + "/joint_states";
        jointStatePublisher.FrameId = robotName;
        JointStateSubscriber  jointStateSubscriber = rosConnector.GetComponent<JointStateSubscriber>();
        jointStateSubscriber.Topic = "/" + robotName + "/joint_commands";
*/
    }

    private static void CleanUp()
    {
        if (instance != null)
        {
            instance.StopListener();
            DestroyImmediate(instance.gameObject); // 不要なオブジェクトを削除
            instance = null;
        }
    }

    private void StopListener()
    {
        isRunning = false;

        // clientがnullでないことを確認してからCloseする
        if (client != null)
        {
            client.Close();
            client = null; // 次回呼び出し時に二重Closeを防ぐ
        }

        // listenerがnullでないことを確認してからStopする
        if (listener != null)
        {
            listener.Stop();
            listener = null; // 次回呼び出し時に二重Stopを防ぐ
        }
    }

    private void OnApplicationQuit()
    {
        StopListener();
    }
    
        // 特定のコンポーネントを持つ子オブジェクトを取得するメソッド
    private List<GameObject> GetChildObjectsWithComponent<T>(GameObject parent) where T : Component
    {
        List<GameObject> objectsWithComponent = new List<GameObject>();

        // 親オブジェクトのすべての子オブジェクトをループ
        foreach (Transform child in parent.transform)
        {
            // 子オブジェクトが指定されたコンポーネントを持っているか確認
            if (child.GetComponent<T>() != null)
            {
                objectsWithComponent.Add(child.gameObject); // リストに追加
            }
        }

        return objectsWithComponent;
    }

    // ArticulationBodyを持つGameObjectを探してリストとして返す
    public List<GameObject> FindArticulationBodyObjectsInChildren(GameObject parent)
    {
        List<GameObject> articulationBodies = new List<GameObject>();
        SearchArticulationBodies(parent.transform, articulationBodies);
        return articulationBodies;
    }

    // 再帰的にArticulationBodyを持つ子オブジェクトを検索
    private void SearchArticulationBodies(Transform parent, List<GameObject> articulationBodies)
    {
        // 現在のオブジェクトがArticulationBodyを持っている場合、リストに追加
        ArticulationBody articulationBody = parent.GetComponent<ArticulationBody>();
        if (articulationBody != null)
        {
            articulationBodies.Add(parent.gameObject);
        }

        // 子オブジェクトを再帰的にチェック
        foreach (Transform child in parent)
        {
            SearchArticulationBodies(child, articulationBodies);
        }
    }
}

