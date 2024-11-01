using UnityEditor;
using UnityEngine;
using RosSharp.RosBridgeClient.UrdfTransfer; // URDF Importerが提供するクラスを利用

public class URDFImportCommand : MonoBehaviour
{
    public static void ImportURDF(string urdfPath)
    {
        if (!System.IO.File.Exists(urdfPath))
        {
            Debug.LogError("URDF file does not exist at path: " + urdfPath);
            return;
        }

        // URDF Importerを呼び出して指定されたパスからインポート
        //UrdfImporter.Import(urdfPath);

        Debug.Log("URDF file imported successfully: " + urdfPath);
    }
}
