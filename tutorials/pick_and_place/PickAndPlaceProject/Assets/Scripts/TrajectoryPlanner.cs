using System;
using System.Collections;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEngine.Networking;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TrajectoryPlanner : MonoBehaviour
{
    /// <summary>
    /// Defines the state machine for how the system selects a target coordinate.
    /// - RandomAutomated: The system infinitely picks and places random coordinates to generate VLA datasets.
    /// - MouseClick: The user manually clicks the screen to define the target coordinate.
    /// - GeminiVision: The VLM acts as the orchestrator, determining the coordinates via camera feed.
    /// </summary>
    public enum InputMode
    {
        RandomAutomated,
        MouseClick,
        GeminiVision
    }

    [Header("Input Mode Settings")]
    public InputMode currentInputMode = InputMode.MouseClick;

    [Header("Gemini Vision Settings")]
    [HideInInspector] public string geminiApiKey = "";
    [TextArea(3, 10)]
    public string geminiPrompt = "Point to the cube. The label returned should be an identifying name for the object detected. The answer should follow the json format: [{\"point\": [y, x], \"label\": \"cube\"}]. The points are in [y, x] format normalized to 0-1000.";
    public bool saveGeminiLogs = true;

    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "niryo_moveit";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_NiryoOne;
    public GameObject NiryoOne { get => m_NiryoOne; set => m_NiryoOne = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;

    [Header("Randomization Settings")]
    [SerializeField] float m_MinRadius = 0.2f;
    [SerializeField] float m_MaxRadius = 0.366f;
    [SerializeField] float m_SpawnY = 0.645f;
    [SerializeField] float m_RestartDelay = 1.0f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

    public bool m_WaitingForInput = false;
    public Vector3 CurrentPickPosition;

    string m_CurrentLogDir = "";
    float m_GeminiRequestStartTime = 0;
    float m_GeminiRequestDuration = 0;
    float m_MoveItStartTime = 0;
    float m_MoveItDuration = 0;
    float m_ExecutionStartTime = 0;
    float m_ExecutionDuration = 0;

    void Update()
    {
        // If we are in manual mode and waiting for input, project a physical Raycast from the camera
        // to mathematically intersect with the virtual 3D table plane.
        if (currentInputMode == InputMode.MouseClick && m_WaitingForInput && Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            Plane tablePlane = new Plane(Vector3.up, new Vector3(0, m_SpawnY, 0));

            if (tablePlane.Raycast(ray, out float distance))
            {
                CurrentPickPosition = ray.GetPoint(distance);
                m_WaitingForInput = false;
                PublishJoints(); // Trigger the ROS planning sequence
            }
        }
    }

    /// <summary>
    /// Initializes the ROS connection and dynamically binds the Unity ArticulationBodies 
    /// (the physics-driven robotic joints) to their corresponding ROS topics.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<MoverServiceRequest, MoverServiceResponse>(m_RosServiceName);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += SourceDestinationPublisher.LinkNames[i];
            m_JointArticulationBodies[i] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

        // Find left and right fingers
        var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";

        m_RightGripper = m_NiryoOne.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_NiryoOne.transform.Find(leftGripper).GetComponent<ArticulationBody>();

        var targetPlacementComponent = m_TargetPlacement.GetComponent<Unity.Robotics.PickAndPlace.TargetPlacement>();
        if (targetPlacementComponent != null)
        {
            targetPlacementComponent.OnTargetPlaced += OnTargetPlaced;
        }

        CurrentPickPosition = m_Target.transform.position;
        if (currentInputMode == InputMode.MouseClick) m_WaitingForInput = true;
        else if (currentInputMode == InputMode.GeminiVision) StartCoroutine(PerformGeminiVisionRequest());
    }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = 0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 0.01f;
        rightDrive.target = -0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>NiryoMoveitJoints</returns>
    NiryoMoveitJointsMsg CurrentJointConfig()
    {
        var joints = new NiryoMoveitJointsMsg();

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joints.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return joints;
    }

    /// <summary>
    /// Packages the current joint states, the calculated Pick Pose, and the target Place Pose 
    /// into a standardized ROS Message (MoverServiceRequest). It sends this request to the 
    /// external MoveIt backend running in Linux/Docker.
    /// </summary>
    public void PublishJoints()
    {
        m_MoveItStartTime = Time.realtimeSinceStartup;
        var request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new PoseMsg
        {
            position = (CurrentPickPosition + m_PickPoseOffset).To<FLU>(),

            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        request.place_pose = new PoseMsg
        {
            position = (m_TargetPlacement.transform.position + m_PickPoseOffset).To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        m_Ros.SendServiceMessage<MoverServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
    }

    /// <summary>
    /// Callback triggered asynchronously when the Linux ROS/MoveIt backend finishes calculating the inverse kinematics.
    /// </summary>
    void TrajectoryResponse(MoverServiceResponse response)
    {
        m_MoveItDuration = Time.realtimeSinceStartup - m_MoveItStartTime;

        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned. Valid path found by MoveIt.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogWarning("No trajectory returned from MoverService. The pose might possess impossible rotational constraints! Randomizing both Cube and Placement zone for a fresh start.");
            
            m_ExecutionDuration = 0;
            SaveTelemetryData(true, "MoveIt failed to find a valid trajectory path.");

            // Randomize Drop-Off
            m_TargetPlacement.transform.position = GetRandomPosition();
            
            // Randomize Cube Target
            Vector3 newTargetPos = GetRandomPosition();
            m_Target.transform.position = newTargetPos;
            var targetRb = m_Target.GetComponent<Rigidbody>();
            if (targetRb != null)
            {
                targetRb.velocity = Vector3.zero;
                targetRb.angularVelocity = Vector3.zero;
            }

            if (currentInputMode == InputMode.MouseClick)
            {
                m_WaitingForInput = true;
            }
            else if (currentInputMode == InputMode.RandomAutomated)
            {
                CurrentPickPosition = m_Target.transform.position;
                PublishJoints();
            }
            else if (currentInputMode == InputMode.GeminiVision)
            {
                StartCoroutine(PerformGeminiVisionRequest());
            }
        }
    }

    /// <summary>
    /// Translates the mathematical array of waypoints returned by MoveIt into physical motor torques 
    /// applied to the Unity ArticulationBodies.
    /// The trajectory contains 4 stages: PreGrasp, Grasp, PickUp, and Place.
    /// </summary>
    IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        m_ExecutionStartTime = Time.realtimeSinceStartup;
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();

            // Wait briefly for the object physics to settle
            yield return new WaitForSeconds(1.0f);

            var targetPlacementComponent = m_TargetPlacement.GetComponent<Unity.Robotics.PickAndPlace.TargetPlacement>();
            if (targetPlacementComponent != null && targetPlacementComponent.CurrentState != Unity.Robotics.PickAndPlace.TargetPlacement.PlacementState.InsidePlaced)
            {
                Debug.LogWarning("Pick and place sequence finished, but the cube isn't in the zone. Retrying...");
                m_ExecutionDuration = Time.realtimeSinceStartup - m_ExecutionStartTime;
                SaveTelemetryData(true, "Cube missed the placement zone physics trigger.");
                if (currentInputMode == InputMode.MouseClick)
                {
                    m_WaitingForInput = true;
                }
                else if (currentInputMode == InputMode.RandomAutomated)
                {
                    CurrentPickPosition = m_Target.transform.position;
                    PublishJoints();
                }
                else if (currentInputMode == InputMode.GeminiVision)
                {
                    StartCoroutine(PerformGeminiVisionRequest());
                }
            }
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }

    /// <summary>
    /// Event triggered by TargetPlacement.cs when the cube successfully settles in the goal box.
    /// </summary>
    void OnTargetPlaced()
    {
        m_ExecutionDuration = Time.realtimeSinceStartup - m_ExecutionStartTime;
        SaveTelemetryData(false, "");
        StartCoroutine(RestartRoutine()); // Triggers Domain Randomization for next loop
    }

    /// <summary>
    /// Domain Randomization Engine: Automatically randomizes the 3D position of both the Cube 
    /// and the Target Placement Zone.
    /// </summary>
    IEnumerator RestartRoutine()
    {
        yield return new WaitForSeconds(m_RestartDelay);

        // Randomize the drop-off position
        m_TargetPlacement.transform.position = GetRandomPosition();

        // Randomize the cube position
        Vector3 newTargetPos = GetRandomPosition();
        m_Target.transform.position = newTargetPos;
        var targetRb = m_Target.GetComponent<Rigidbody>();
        if (targetRb != null)
        {
            targetRb.velocity = Vector3.zero;
            targetRb.angularVelocity = Vector3.zero;
        }

        if (currentInputMode == InputMode.MouseClick)
        {
            m_WaitingForInput = true;
        }
        else if (currentInputMode == InputMode.RandomAutomated)
        {
            CurrentPickPosition = m_Target.transform.position;
            PublishJoints();
        }
        else if (currentInputMode == InputMode.GeminiVision)
        {
            StartCoroutine(PerformGeminiVisionRequest());
        }
    }

    IEnumerator PerformGeminiVisionRequest()
    {
        m_GeminiRequestStartTime = Time.realtimeSinceStartup;
        m_GeminiRequestDuration = 0;
        m_MoveItDuration = 0;
        m_ExecutionDuration = 0;

        string keyPath = System.IO.Path.Combine(Application.dataPath, "../gemini_api_key.txt");
        if (System.IO.File.Exists(keyPath))
        {
            geminiApiKey = System.IO.File.ReadAllText(keyPath).Trim();
        }
        else
        {
            Debug.LogError("API Key file missing! Please open gemini_api_key.txt in your PickAndPlaceProject folder and paste your key.");
            if (currentInputMode == InputMode.MouseClick) m_WaitingForInput = true;
            yield break;
        }

        yield return new WaitForEndOfFrame();

        Camera cam = Camera.main;
        RenderTexture rt = new RenderTexture(cam.pixelWidth, cam.pixelHeight, 24);
        cam.targetTexture = rt;
        Texture2D screenShot = new Texture2D(cam.pixelWidth, cam.pixelHeight, TextureFormat.RGB24, false);
        cam.Render();
        RenderTexture.active = rt;
        screenShot.ReadPixels(new Rect(0, 0, cam.pixelWidth, cam.pixelHeight), 0, 0);
        cam.targetTexture = null;
        RenderTexture.active = null; 
        Destroy(rt);

        byte[] imageBytes = screenShot.EncodeToPNG();
        string base64Image = System.Convert.ToBase64String(imageBytes);

        m_CurrentLogDir = string.Empty;
        if (saveGeminiLogs)
        {
            m_CurrentLogDir = System.IO.Path.Combine(Application.dataPath, "../GeminiOutputs/Action_" + DateTime.Now.ToString("yyyyMMdd_HHmmss"));
            System.IO.Directory.CreateDirectory(m_CurrentLogDir);
            System.IO.File.WriteAllBytes(System.IO.Path.Combine(m_CurrentLogDir, "raw_frame.png"), imageBytes);
        }

        string schemaStr = "\"responseMimeType\": \"application/json\", \"responseJsonSchema\": { \"type\": \"array\", \"items\": { \"type\": \"object\", \"properties\": { \"point\": { \"type\": \"array\", \"items\": { \"type\": \"integer\" } }, \"label\": { \"type\": \"string\" } }, \"required\": [\"point\", \"label\"] } }";
        string jsonPayload = "{ \"contents\": [ { \"parts\": [ { \"text\": \"" + geminiPrompt.Replace("\"", "\\\"").Replace("\n", " ") + "\" }, { \"inline_data\": { \"mime_type\": \"image/png\", \"data\": \"" + base64Image + "\" } } ] } ], \"generationConfig\": { \"temperature\": 1.0, " + schemaStr + " } }";
        string url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-robotics-er-1.6-preview:generateContent?key=" + geminiApiKey;

        using (UnityWebRequest request = new UnityWebRequest(url, "POST"))
        {
            byte[] bodyRaw = System.Text.Encoding.UTF8.GetBytes(jsonPayload);
            request.uploadHandler = new UploadHandlerRaw(bodyRaw);
            request.downloadHandler = new DownloadHandlerBuffer();
            request.SetRequestHeader("Content-Type", "application/json");

            Debug.Log("Sending request to Gemini Robotics API...");
            yield return request.SendWebRequest();

            m_GeminiRequestDuration = Time.realtimeSinceStartup - m_GeminiRequestStartTime;

            if (request.result == UnityWebRequest.Result.ConnectionError || request.result == UnityWebRequest.Result.ProtocolError)
            {
                Debug.LogError("Gemini API Error: " + request.error + " | " + request.downloadHandler.text);
                SaveTelemetryData(true, "Gemini API Error: " + request.error);
                if (currentInputMode == InputMode.MouseClick) m_WaitingForInput = true;
            }
            else
            {
                string responseText = request.downloadHandler.text;
                Debug.Log("Gemini Response: " + responseText);

                if (saveGeminiLogs && !string.IsNullOrEmpty(m_CurrentLogDir))
                {
                    System.IO.File.WriteAllText(System.IO.Path.Combine(m_CurrentLogDir, "gemini_response.json"), responseText);
                }

                Match match = Regex.Match(responseText, @"point.*?\[\s*(\d+)\s*,\s*(\d+)\s*\]");
                if (match.Success)
                {
                    float yNorm = float.Parse(match.Groups[1].Value);
                    float xNorm = float.Parse(match.Groups[2].Value);

                    float screenX = cam.pixelWidth * (xNorm / 1000f);
                    float screenY = cam.pixelHeight * (1f - (yNorm / 1000f)); 

                    if (saveGeminiLogs && !string.IsNullOrEmpty(m_CurrentLogDir))
                    {
                        int px = (int)screenX;
                        int py = (int)screenY;
                        for (int i = -10; i <= 10; i++)
                        {
                            for (int w = -1; w <= 1; w++)
                            {
                                if (px + i >= 0 && px + i < screenShot.width && py + w >= 0 && py + w < screenShot.height)
                                    screenShot.SetPixel(px + i, py + w, Color.red);
                                if (px + w >= 0 && px + w < screenShot.width && py + i >= 0 && py + i < screenShot.height)
                                    screenShot.SetPixel(px + w, py + i, Color.red);
                            }
                        }
                        screenShot.Apply();
                        System.IO.File.WriteAllBytes(System.IO.Path.Combine(m_CurrentLogDir, "labeled_frame.png"), screenShot.EncodeToPNG());
                    }

                    Ray ray = cam.ScreenPointToRay(new Vector3(screenX, screenY, 0));
                    Plane tablePlane = new Plane(Vector3.up, new Vector3(0, m_SpawnY, 0));

                    if (tablePlane.Raycast(ray, out float distance))
                    {
                        CurrentPickPosition = ray.GetPoint(distance);
                        PublishJoints();
                    }
                    else 
                    {
                        Debug.LogError("Raycast from Gemini point did not hit table.");
                        SaveTelemetryData(true, "Raycast from Gemini point did not hit table physics plane.");
                        if (currentInputMode == InputMode.MouseClick) m_WaitingForInput = true;
                    }
                }
                else
                {
                    Debug.LogError("Could not parse point from Gemini response.");
                    SaveTelemetryData(true, "Could not parse coordinate regex from Gemini response.");
                    if (currentInputMode == InputMode.MouseClick) m_WaitingForInput = true;
                }
            }
        }
    }

    void SaveTelemetryData(bool failed, string reason)
    {
        if (!saveGeminiLogs || string.IsNullOrEmpty(m_CurrentLogDir) || currentInputMode != InputMode.GeminiVision) return;

        string telemetry = "{\n";
        telemetry += $"  \"gemini_request_seconds\": {m_GeminiRequestDuration.ToString("F3", System.Globalization.CultureInfo.InvariantCulture)},\n";
        telemetry += $"  \"moveit_planning_seconds\": {m_MoveItDuration.ToString("F3", System.Globalization.CultureInfo.InvariantCulture)},\n";
        telemetry += $"  \"robot_execution_seconds\": {m_ExecutionDuration.ToString("F3", System.Globalization.CultureInfo.InvariantCulture)},\n";
        float total = m_GeminiRequestDuration + m_MoveItDuration + m_ExecutionDuration;
        telemetry += $"  \"total_action_seconds\": {total.ToString("F3", System.Globalization.CultureInfo.InvariantCulture)},\n";
        telemetry += $"  \"status\": \"{(failed ? "FAILED" : "SUCCESS")}\"";
        if (failed) telemetry += $",\n  \"error_reason\": \"{reason}\"";
        telemetry += "\n}";

        System.IO.File.WriteAllText(System.IO.Path.Combine(m_CurrentLogDir, "telemetry.json"), telemetry);
    }

    Vector3 GetRandomPosition()
    {
        float randomAngle = UnityEngine.Random.Range(0f, Mathf.PI * 2f);
        float randomRadius = Mathf.Sqrt(UnityEngine.Random.Range(m_MinRadius * m_MinRadius, m_MaxRadius * m_MaxRadius));
        return new Vector3(randomRadius * Mathf.Cos(randomAngle), m_SpawnY, randomRadius * Mathf.Sin(randomAngle));
    }
}
