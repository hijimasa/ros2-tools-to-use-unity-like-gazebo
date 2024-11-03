using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEditor;
using UnityEngine;
using RosSharp.RosBridgeClient;

[InitializeOnLoad]
[ExecuteInEditMode]
public class RemoteCommandListener : MonoBehaviour
{
    private static RemoteCommandListener instance = null;
    private TransferFromRosHandlerMinimum transferHandler;
    private TcpListener listener;
    private bool rosConnectorFound = false;
    private bool isRunning = false;

    static RemoteCommandListener()
    {
        // エディタ起動時、または再生/停止時に呼び出される
        //Initialize();
        EditorApplication.delayCall += RemoteCommandListener.Initialize;
    }


    private static void Initialize()
    {
        Debug.Log("Initializing...");
        // 既に存在する場合、再作成しない
        if (instance != null) return;
        Debug.Log("Initializing2...");

        GameObject obj = new GameObject("RemoteCommandListener");
        instance = obj.AddComponent<RemoteCommandListener>();
        
        EditorApplication.delayCall -= RemoteCommandListener.Initialize;
    }

    private async void Start()
    {
        if (isRunning) return; // 多重起動防止
        isRunning = true;

        transferHandler = new TransferFromRosHandlerMinimum();
        rosConnectorFound = transferHandler.CheckForRosConnector();

        if (!rosConnectorFound)
        {
            transferHandler.CreateRosConnector();
            rosConnectorFound = transferHandler.CheckForRosConnector();
        }

        listener = new TcpListener(IPAddress.Any, 5000);
        listener.Start();
        Debug.Log("Listening for URDF import commands...");

        while (isRunning)
        {
            TcpClient client = await listener.AcceptTcpClientAsync();
            HandleClient(client);
        }
    }

    private async void HandleClient(TcpClient client)
    {
        NetworkStream stream = client.GetStream();
        byte[] buffer = new byte[1024];
        int bytesRead = await stream.ReadAsync(buffer, 0, buffer.Length);
        string commandString = Encoding.UTF8.GetString(buffer, 0, bytesRead);
        string[] commands = commandString.Split(' ');

        string robotNameParameter = commands[1];
        string urdfParameter = commands[2];
        string assetPath = commands[3];

        Debug.Log("Received Asset path: " + assetPath);
        transferHandler.TransferUrdf(assetPath, urdfParameter, robotNameParameter);

        while (!transferHandler.GenerateModelIfReady())
        {
            Thread.Sleep(100);
        }

        client.Close();
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
        listener?.Stop();
    }

    private void OnApplicationQuit()
    {
        StopListener();
    }
}

