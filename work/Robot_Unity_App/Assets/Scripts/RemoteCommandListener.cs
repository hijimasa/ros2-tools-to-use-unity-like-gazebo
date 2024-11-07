using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Xml;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

using UnitySensors.Sensor.Camera;
using UnitySensors.Sensor.LiDAR;
using UnitySensors.ROS.Publisher.Camera;
using UnitySensors.ROS.Publisher.Image;

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

        // Parse URDF File
        XmlDocument xmlDoc = new XmlDocument();
        xmlDoc.Load(urdfFilePath);

        // Create JointStatePublisher & JointStateSubscriber
        JointStatePub jointStatePub = robotObject.AddComponent<JointStatePub>();
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
                    jointNameList.Add(urdfJoint.jointName);
                    
                    var parameters = GetUnityDriveApiParameters(xmlDoc, urdfJoint.jointName);
                    ArticulationDrive drive = body.xDrive;
                    drive.stiffness = parameters["stiffness"];
                    drive.damping = parameters["damping"];
                    drive.forceLimit = parameters["force_limit"];
                    body.xDrive = drive;
                }
            }
        }

        jointStatePub.articulationBodies = articulationBodyList.ToArray();
        jointStatePub.jointName = jointNameList.ToArray();
        jointStatePub.jointLength = articulationBodyList.Count;
        XmlNode jointStateParam = xmlDoc.SelectSingleNode("//robot/ros2_control/hardware/param[@name='joint_states_topic']");
        if (jointStateParam != null)
        {
            jointStatePub.topicName = jointStateParam.InnerText;
        }

        jointStateSub.articulationBodies = articulationBodyList.ToArray();
        jointStateSub.jointName = jointNameList.ToArray();
        jointStateSub.jointLength = articulationBodyList.Count;
        XmlNode jointCommandParam = xmlDoc.SelectSingleNode("//robot/ros2_control/hardware/param[@name='joint_commands_topic']");
        if (jointCommandParam != null)
        {
            jointStateSub.topicName = jointCommandParam.InnerText;
        }

        // Physics Materialの生成
        // URDFのファイルパスからファイル名を取り除く
        string directoryPath = Path.GetDirectoryName(urdfFilePath);
        // "Assets"以前の文字列を取り除く
        int assetsIndex = directoryPath.IndexOf("Assets");
        if (assetsIndex >= 0)
        {
            directoryPath = directoryPath.Substring(assetsIndex);
        }
        // ディレクトリが存在するか確認し、存在しなければ作成する
        if (!Directory.Exists(directoryPath + "/PhysicsMaterials"))
        {
            Directory.CreateDirectory(directoryPath + "/PhysicsMaterials");
            Debug.Log("Directory created at: " + directoryPath + "/PhysicsMaterials");
        }
        // <robot>要素を取得
        XmlNode robotNode = xmlDoc.SelectSingleNode("/robot");
        if (robotNode != null)
        {
            // 全ての<physics_material>要素を取得
            XmlNodeList physicsMaterials = robotNode.SelectNodes("physics_material");
            foreach (XmlNode physicsMaterial in physicsMaterials)
            {
                PhysicsMaterial newMaterial = new PhysicsMaterial();
                XmlNode frictionNode = physicsMaterial.SelectSingleNode("friction");
                if (frictionNode != null)
                {
                    newMaterial.staticFriction = TryParseFloat(frictionNode.Attributes["static"]?.Value);
                    newMaterial.dynamicFriction = TryParseFloat(frictionNode.Attributes["dynamic"]?.Value);
                }

                string materialName = physicsMaterial.Attributes["name"]?.Value;
                string path = directoryPath + "/PhysicsMaterials/" + materialName + ".physicMaterial";

                AssetDatabase.CreateAsset(newMaterial, path);
                AssetDatabase.SaveAssets();
            }
        }

        // Physics Materialの適用
        // <robot>要素を取得
        if (robotNode != null)
        {
            // 全ての<link>要素を取得
            XmlNodeList links = robotNode.SelectNodes("link");
            foreach (XmlNode link in links)
            {
                XmlNode collisionNode = link.SelectSingleNode("collision");
                if (collisionNode != null)
                {
                    XmlNode physicsMaterial = collisionNode.SelectSingleNode("physics_material");
                    if (physicsMaterial != null)
                    {
                        string materialName = physicsMaterial.Attributes["name"]?.Value;
                        string linkName = link.Attributes["name"]?.Value;
                        GameObject targetObject = FindInChildrenByName(robotObject.transform, linkName);
                        if (targetObject != null)
                        {
                            Transform collisionTransform = targetObject.transform.Find("Collisions");
                            if (collisionTransform != null)
                            {
                                Transform unnamedCollision = collisionTransform.GetChild(0);
                                Transform targetCollision = unnamedCollision.GetChild(0);
                                if (targetCollision != null)
                                {
                                    Debug.Log(materialName + ": " + linkName);
                                    Collider meshCollider = targetCollision.gameObject.GetComponent<Collider>();

                                    string path = directoryPath + "/PhysicsMaterials/" + materialName + ".physicMaterial";
                                    PhysicsMaterial loadedMaterial = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(path);
                                    
                                    meshCollider.material = loadedMaterial;
                                }
                            }
                        }
                    }
                }
            }
        }

        // unityタグからセンサの読み込み
        // <robot>要素を取得
        if (robotNode != null)
        {
            // <unity>要素を取得
            XmlNode unityNode = robotNode.SelectSingleNode("unity");
            if (unityNode != null)
            {
                // 全ての<physics_material>要素を取得
                XmlNodeList unitySensors = unityNode.SelectNodes("sensor");
                foreach (XmlNode sensor in unitySensors)
                {
                    string sensorType = sensor.Attributes["type"]?.Value;
                    string sensorLinkName = sensor.Attributes["name"]?.Value;
                    GameObject targetObject = FindInChildrenByName(robotObject.transform, sensorLinkName);
                    if (targetObject != null)
                    {
                        switch (sensorType)
                        {
                            case "lidar":
                                Debug.Log("sensor type 'lidar' found");
                                //LiDARSensor lidarSensor = targetObject.AddComponent<LiDARSensor>();
                                break;
                            case "camera":
                                Debug.Log("sensor type 'camera' found");
                                //RGBCameraSensor cameraSensor = targetObject.AddComponent<RGBCameraSensor>();
                                //CameraInfoMsgPublisher cameraInfoPublisher = targetObject.AddComponent<CameraInfoMsgPublisher>();
                                //CameraImageMsgPublisher cameraImagePublisher = targetObject.AddComponent<CameraImageMsgPublisher>();
                                //cameraInfoPublisher.topicName = "/"+ robotObject.name + "/" + sensorLinkName + "/camera_info";
                                //cameraImagePublisher.topicName = "/"+ robotObject.name + "/" + sensorLinkName + "/image_raw";
                                break;
                            case "depth_camera":
                                Debug.Log("sensor type 'depth_camera' found");
                                //DepthCameraSensor depthCameraSensor = targetObject.AddComponent<DepthCameraSensor>();
                                //CameraInfoMsgPublisher depthCameraInfoPublisher = targetObject.AddComponent<CameraInfoMsgPublisher>();
                                //CameraImageMsgPublisher depthCameraImagePublisher = targetObject.AddComponent<CameraImageMsgPublisher>();
                                //depthCameraInfoPublisher.topicName = "/"+ robotObject.name + "/" + sensorLinkName + "/depth_camera_info";
                                //depthCameraImagePublisher.topicName = "/"+ robotObject.name + "/" + sensorLinkName + "/depth_image_raw";
                                break;
                            default:
                                Debug.Log("undefined sensor type found");
                                break;
                        }
                    }
                }
            }
        }

/*
       // 一番上にあるUrdfLinkコンポーネントにIsBaseLinkを設定
        List<GameObject> childObjectsWithUrdfLink = GetChildObjectsWithComponent<UrdfLink>(robotObject);
        foreach (GameObject child in childObjectsWithUrdfLink)
        {
            UrdfLink link = child.GetComponent<UrdfLink>();
            link.IsBaseLink = true;
            break;
        }
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

    GameObject FindInChildrenByName(Transform parent, string name)
    {
        // 現在のオブジェクトが目的の名前かを確認
        if (parent.name == name)
        {
            return parent.gameObject;
        }

        // 子オブジェクトを再帰的にチェック
        foreach (Transform child in parent)
        {
            GameObject result = FindInChildrenByName(child, name);
            if (result != null)
            {
                return result;
            }
        }
        
        // 見つからなかった場合
        return null;
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

    public static Dictionary<string, float> GetUnityDriveApiParameters(XmlDocument xmlDoc, string targetJointName)
    {
        var parameters = new Dictionary<string, float>();

        // <robot>要素を取得
        XmlNode robotNode = xmlDoc.SelectSingleNode("/robot");
        if (robotNode != null)
        {
            // 全ての<joint>要素を取得
            XmlNodeList jointNodes = robotNode.SelectNodes("joint");

            foreach (XmlNode jointNode in jointNodes)
            {
                // 指定されたname属性のjointタグを探す
                if (jointNode.Attributes["name"]?.Value == targetJointName)
                {
                    // unity_drive_api内の各パラメータの取得
                    XmlNode unityDriveApiNode = jointNode.SelectSingleNode("unity_drive_api");
                    if (unityDriveApiNode != null)
                    {
                        // stiffness, damping, force_limitをfloat型に変換して追加
                        parameters["stiffness"] = TryParseFloat(unityDriveApiNode.Attributes["stiffness"]?.Value);
                        parameters["damping"] = TryParseFloat(unityDriveApiNode.Attributes["damping"]?.Value);
                        parameters["force_limit"] = TryParseFloat(unityDriveApiNode.Attributes["force_limit"]?.Value);
                    }
                    else
                    {
                        Console.WriteLine("unity_drive_api element not found.");
                        parameters["stiffness"] = 0.0F;
                        parameters["damping"] = 0.0F;
                        parameters["force_limit"] = 0.0F;
                    }
                    return parameters; // 見つけたら戻り値を返す
                }
            }

            // 指定されたnameが見つからなかった場合
            Console.WriteLine("Joint with the specified name not found.");
        }
        else
        {
            Console.WriteLine("Robot element not found.");
        }

        parameters["stiffness"] = 0.0F;
        parameters["damping"] = 0.0F;
        parameters["force_limit"] = 0.0F;

        return parameters;
    }

    // 文字列をfloatに変換するためのヘルパーメソッド
    private static float TryParseFloat(string value)
    {
        return float.TryParse(value, out float result) ? result : 0f;
    }
}

