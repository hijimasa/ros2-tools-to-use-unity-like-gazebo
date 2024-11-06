using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SocialPlatforms;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEditor;

[System.Serializable]
public class PreviousScenePosition
{
    public Vector3 editorCamPosition;
    public Quaternion editorCamRotation;
}

public class CameraMover : MonoBehaviour
{
    [SerializeField, Range(0.1f, 10.0f)]
    private float _positionStep = 2.0f;
    [SerializeField, Range(30.0f, 150.0f)]
    private float _mouseSensitive = 90.0f;

    private bool _cameraMoveActive = true;
    private Transform _camTransform;
    private Vector3 _startMousePos;
    private Vector3 _presentCamRotation;
    private Vector3 _presentCamPos;
    private Quaternion _initialCamRotation;
    private bool _uiMessageActiv;
    private GameObject mainCamera = null;

    private void OnEnable()
    {
        EditorApplication.playModeStateChanged += OnPlayModeChanged;
    }

    private void OnPlayModeChanged(PlayModeStateChange state)
    {
        if (state == PlayModeStateChange.ExitingEditMode)
        {
            // プレイモードに入る直前にSceneViewのカメラ位置と回転をキャッシュ
            SceneView sceneView = SceneView.lastActiveSceneView;
            if (sceneView != null)
            {
                PreviousScenePosition data = new PreviousScenePosition();

                data.editorCamPosition = sceneView.camera.transform.position;
                data.editorCamRotation = sceneView.camera.transform.rotation;
                Debug.Log("Exiting Edit Mode - _editorCamPosition saved: " + data.editorCamPosition);

                // データをJSONに変換し、ファイルに保存
                string json = JsonUtility.ToJson(data);
                System.IO.File.WriteAllText(Application.persistentDataPath + "/previous_scene_position.json", json);
            }
        }
        else if (state == PlayModeStateChange.EnteredPlayMode)
        {
            if (mainCamera == null)
            {
                mainCamera = GameObject.Find("Main Camera");
            }
            if (mainCamera != null)
            {
                string path = Application.persistentDataPath + "/previous_scene_position.json";
                if (System.IO.File.Exists(path))
                {
                    // ファイルからデータを読み込んでデシリアライズ
                    string json = System.IO.File.ReadAllText(path);
                    PreviousScenePosition data = JsonUtility.FromJson<PreviousScenePosition>(json);

                    // 保存したエディタモードのカメラ位置と回転を適用
                    mainCamera.transform.position = data.editorCamPosition;
                    mainCamera.transform.rotation = data.editorCamRotation;
                    Debug.Log("Entered Play Mode - Main Camera positioned at _editorCamPosition: " + data.editorCamPosition);
                }
                else
                {
                    Debug.Log("Entered Play Mode - file not found");
                }
            }
            else
            {
                Debug.LogWarning("Main Camera が見つかりませんでした。");
            }
        }
    }

    void Start()
    {
        _camTransform = this.gameObject.transform;
        if (mainCamera == null)
        {
            mainCamera = GameObject.Find("Main Camera");
        }
        if (mainCamera != null)
        {
            mainCamera.transform.SetParent(this.transform);
            Debug.Log("Main Camera を CameraMover の子に設定しました。");
        }
        else
        {
            Debug.LogWarning("Main Camera が見つかりませんでした。");
        }

        _initialCamRotation = this.gameObject.transform.rotation;
    }

    void Update()
    {
        CamControlIsActive();
        if (_cameraMoveActive)
        {
            ResetCameraRotation();
            CameraRotationMouseControl();
            CameraSlideMouseControl();
            CameraPositionKeyControl();
            CameraZoomMouseScroll();
        }
    }

    public void CamControlIsActive()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            _cameraMoveActive = !_cameraMoveActive;

            if (_uiMessageActiv == false)
            {
                StartCoroutine(DisplayUiMessage());
            }
            Debug.Log("CamControl : " + _cameraMoveActive);
        }
    }

    private void ResetCameraRotation()
    {
        if(Input.GetKeyDown(KeyCode.P))
        {
            this.gameObject.transform.rotation = _initialCamRotation;
            Debug.Log("Cam Rotate : " + _initialCamRotation.ToString());    
        }
    }

    // 右クリックで回転
    private void CameraRotationMouseControl()
    {
        if (Input.GetMouseButtonDown(1))
        {
            _startMousePos = Input.mousePosition;
            _presentCamRotation.x = _camTransform.transform.eulerAngles.x;
            _presentCamRotation.y = _camTransform.transform.eulerAngles.y;
        }

        if (Input.GetMouseButton(1))
        {
            float x = (_startMousePos.x - Input.mousePosition.x) / Screen.width;
            float y = (_startMousePos.y - Input.mousePosition.y) / Screen.height;
            float eulerX = _presentCamRotation.x + y * _mouseSensitive;
            float eulerY = _presentCamRotation.y + x * _mouseSensitive;
            _camTransform.rotation = Quaternion.Euler(eulerX, eulerY, 0);
        }
    }

    // 中ボタンで平行移動
    private void CameraSlideMouseControl()
    {
        if (Input.GetMouseButtonDown(2))
        {
            _startMousePos = Input.mousePosition;
            _presentCamPos = _camTransform.position;
        }

        if (Input.GetMouseButton(2))
        {
            float x = (_startMousePos.x - Input.mousePosition.x) / Screen.width;
            float y = (_startMousePos.y - Input.mousePosition.y) / Screen.height;
            x = x * _positionStep;
            y = y * _positionStep;
            Vector3 velocity = _camTransform.rotation * new Vector3(x, y, 0);
            velocity = velocity + _presentCamPos;
            _camTransform.position = velocity;
        }
    }

    private void CameraZoomMouseScroll()
    {
        float scrollInput = Input.GetAxis("Mouse ScrollWheel");
        _camTransform.position += _camTransform.forward * scrollInput * _positionStep;
    }

    private void CameraPositionKeyControl()
    {
        Vector3 campos = _camTransform.position;

        if (Input.GetKey(KeyCode.D)) { campos += _camTransform.right * Time.deltaTime * _positionStep; }
        if (Input.GetKey(KeyCode.A)) { campos -= _camTransform.right * Time.deltaTime * _positionStep; }
        if (Input.GetKey(KeyCode.E)) { campos += _camTransform.up * Time.deltaTime * _positionStep; }
        if (Input.GetKey(KeyCode.Q)) { campos -= _camTransform.up * Time.deltaTime * _positionStep; }
        if (Input.GetKey(KeyCode.W)) { campos += _camTransform.forward * Time.deltaTime * _positionStep; }
        if (Input.GetKey(KeyCode.S)) { campos -= _camTransform.forward * Time.deltaTime * _positionStep; }

        _camTransform.position = campos;
    }

    private IEnumerator DisplayUiMessage()
    {
        _uiMessageActiv = true;
        float time = 0;
        while (time < 2)
        {
            time = time + Time.deltaTime;
            yield return null;
        }
        _uiMessageActiv = false;
    }

    void OnGUI()
    {
        if (_uiMessageActiv == false) { return; }
        GUI.color = Color.black;
        if (_cameraMoveActive == true)
        {
            GUI.Label(new Rect(Screen.width / 2 - 50, Screen.height - 30, 100, 20), "カメラ操作 有効");
        }

        if (_cameraMoveActive == false)
        {
            GUI.Label(new Rect(Screen.width / 2 - 50, Screen.height - 30, 100, 20), "カメラ操作 無効");
        }
    }
}

[InitializeOnLoad]
public class CameraMoverPlacer : MonoBehaviour
{
    private static CameraMover instance = null;

    private class StartUpData : ScriptableSingleton<StartUpData>
    {
        [SerializeField]
        private int _callCount;
        public bool IsStartUp()
        {
            return _callCount++ == 0;
        }
    }

    static CameraMoverPlacer()
    {
        // エディタが完全に起動した後に初期化する
        EditorApplication.delayCall += CameraMoverPlacer.Initialize;
    }

    private static void Initialize()
    {
        // StartUpDataの初期化をInitialize内で行う
        if (!StartUpData.instance.IsStartUp())
            return;

        // 既に存在する場合、再作成しない
        if (instance != null) return;

        GameObject obj = new GameObject("CameraMover");
        instance = obj.AddComponent<CameraMover>();
        
        EditorApplication.delayCall -= CameraMoverPlacer.Initialize;
    }

}
