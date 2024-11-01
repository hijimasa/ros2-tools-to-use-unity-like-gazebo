using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using UnityEditor;
using UnityEngine;

public class RemoteCommandListener : MonoBehaviour
{
    private TcpListener listener;

    async void Start()
    {
        listener = new TcpListener(IPAddress.Any, 5000);
        listener.Start();
        Debug.Log("Listening for URDF import commands...");

        while (true)
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
        string urdfPath = Encoding.UTF8.GetString(buffer, 0, bytesRead);

        Debug.Log("Received URDF path: " + urdfPath);
        URDFImportCommand.ImportURDF(urdfPath);

        client.Close();
    }

    private void OnApplicationQuit()
    {
        listener?.Stop();
    }
}
