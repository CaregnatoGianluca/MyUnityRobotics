using UnityEngine;
using UnityEngine.Networking;
using System.Collections;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace MitsubishiVisionApp
{
    /// <summary>
    /// This is the Core Orchestrator of the autonomous system. 
    /// It manages the interaction between the Vision system (Eyes), the Artificial Intelligence (Brain), 
    /// the Mathematical Translation (Homography), and the Physical Hardware (Robot Actor).
    /// </summary>
    public class VisionAppManager : MonoBehaviour
    {
        [Header("System References")]
        public WebcamStream webcamStream;
        public CalibrationController calibrationController;
        public MitsubishiTCPClient tcpClient;

        [Header("Gemini AI API")]
        public string geminiPrompt = "Identify the center of the wooden cube in this image. Return its [x, y] pixel coordinates.";
        
        [Header("Robot Configuration")]
        public float pickHeightZ = 20.0f;

        [Header("Digital Twin Integration")]
        [Tooltip("The 3D cube Target GameObject in the Unity scene.")]
        public Transform digitalTwinTarget;
        [Tooltip("The existing ROS publisher script from the tutorial.")]
        public SourceDestinationPublisher rosPublisher;
        [Tooltip("Offset to perfectly align the 3D table to your physical table origin.")]
        public Vector3 unityOriginOffset = new Vector3(0, 0.63f, 0);

        private string m_ApiKey;
        private bool m_IsProcessing = false;

        void Start()
        {
            string keyPath = System.IO.Path.Combine(Application.dataPath, "../../gemini_api_key.txt");
            if (System.IO.File.Exists(keyPath))
            {
                m_ApiKey = System.IO.File.ReadAllText(keyPath).Trim();
            }
            else
            {
                Debug.LogError("[VisionManager] No API Key found! Place gemini_api_key.txt next to Assets folder.");
            }
        }

        void Update()
        {
            // DEBUG TEST: Press Spacebar to test TCP socket without the camera or homography
            if (Input.GetKeyDown(KeyCode.Space))
            {
                float testX = 250.0f;
                float testY = 0.0f;
                float testZ = 20.0f; // Table pickup height
                
                Debug.Log($"[VisionManager] TEST MODE: Sending hardcoded coordinate (X: {testX}, Y: {testY}, Z: {testZ}) to robot...");
                string payload = tcpClient.FormatMelfaPositionString(testX, testY, testZ);
                _ = tcpClient.SendCommandAsync(payload);

                // Digital Twin Synchronization for Spacebar Test
                if (digitalTwinTarget != null)
                {
                    float unityZ = (testX / 1000f) + unityOriginOffset.z;
                    float unityX = (-testY / 1000f) + unityOriginOffset.x;
                    float unityY = (testZ / 1000f) + unityOriginOffset.y;

                    digitalTwinTarget.position = new Vector3(unityX, unityY, unityZ);
                    Debug.Log($"[Digital Twin] Teleported 3D Target to Unity coordinates: {digitalTwinTarget.position}");

                    // Find TrajectoryPlanner and trigger the Pick sequence
                    var planner = FindObjectOfType<TrajectoryPlanner>();
                    if (planner != null)
                    {
                        planner.CurrentPickPosition = digitalTwinTarget.position;
                        planner.PublishJoints();
                        Debug.Log("[Digital Twin] Triggered ROS 3D Pick Sequence via Spacebar!");
                    }
                }
            }
        }

        /// <summary>
        /// Attached to a UI Button. Kicks off the entire autonomous sequence.
        /// It strictly prevents execution if the mathematical camera calibration has not been completed.
        /// </summary>
        public void TriggerPickSequence()
        {
            if (m_IsProcessing) return; // Prevent overlapping requests
            
            // Safety Check: We cannot translate AI pixels to physical space without the calibration matrix.
            if (calibrationController.ActiveHomographyMatrix == null)
            {
                Debug.LogError("[VisionManager] CANNOT EXECUTE! You must click the 4 pieces of tape to calibrate the Homography first!");
                return;
            }

            // Start the sequence on a background thread (Coroutine) so the Unity UI does not freeze.
            StartCoroutine(ExecuteVisionSequence());
        }

        /// <summary>
        /// The main execution pipeline: Capture Image -> Query AI -> Calculate Math -> Move Robots.
        /// </summary>
        private IEnumerator ExecuteVisionSequence()
        {
            m_IsProcessing = true;
            Debug.Log("[VisionManager] Starting Vision Sequence...");

            // 1. Capture Frame
            Texture2D frame = webcamStream.CaptureFrame();
            if (frame == null)
            {
                Debug.LogError("[VisionManager] Webcam frame is null.");
                m_IsProcessing = false;
                yield break;
            }

            // 2. Build JSON Payload
            // We encode the raw image into a Base64 string to safely transmit it over HTTP.
            byte[] imageBytes = frame.EncodeToPNG();
            string base64Image = System.Convert.ToBase64String(imageBytes);
            
            // PROMPT ENGINEERING & JSON SCHEMA:
            // This is arguably the most critical part of the AI integration. 
            // We force the VLM to return its spatial reasoning as a strictly formatted JSON array.
            // By defining `responseJsonSchema`, we prevent the AI from generating conversational text 
            // and guarantee we receive parsable [x, y] integer pixels.
            string schemaStr = "\"responseMimeType\": \"application/json\", \"responseJsonSchema\": { \"type\": \"array\", \"items\": { \"type\": \"object\", \"properties\": { \"point\": { \"type\": \"array\", \"items\": { \"type\": \"integer\" } }, \"label\": { \"type\": \"string\" } }, \"required\": [\"point\", \"label\"] } }";
            string jsonPayload = "{ \"contents\": [ { \"parts\": [ { \"text\": \"" + geminiPrompt.Replace("\"", "\\\"").Replace("\n", " ") + "\" }, { \"inline_data\": { \"mime_type\": \"image/png\", \"data\": \"" + base64Image + "\" } } ] } ], \"generationConfig\": { \"temperature\": 1.0, " + schemaStr + " } }";
            
            string url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-robotics-er-1.6-preview:generateContent?key=" + m_ApiKey;

            // 3. Send HTTP Request to Google's Gemini Servers
            using (UnityWebRequest request = new UnityWebRequest(url, "POST"))
            {
                byte[] bodyRaw = System.Text.Encoding.UTF8.GetBytes(jsonPayload);
                request.uploadHandler = new UploadHandlerRaw(bodyRaw);
                request.downloadHandler = new DownloadHandlerBuffer();
                request.SetRequestHeader("Content-Type", "application/json");

                Debug.Log("[VisionManager] Awaiting Gemini AI...");
                yield return request.SendWebRequest(); // Wait for AI response without blocking the main thread

                if (request.result != UnityWebRequest.Result.Success)
                {
                    Debug.LogError("[VisionManager] Gemini API Error: " + request.error);
                }
                else
                {
                    // 4. Parse AI response
                    string responseText = request.downloadHandler.text;
                    Match match = Regex.Match(responseText, @"point.*?\[\s*(\d+)\s*,\s*(\d+)\s*\]");
                    if (match.Success)
                    {
                        float px = float.Parse(match.Groups[1].Value);
                        float py = float.Parse(match.Groups[2].Value);
                        Debug.Log($"[VisionManager] Gemini Found Cube at Pixel: [{px}, {py}]");

                        // 5. Homography Mathematical Translation
                        // We take the 2D digital pixels provided by the AI and multiply them against 
                        // the 8x8 Homography matrix. This instantly transforms the digital space into 
                        // the physical workspace, outputting exact real-world millimeters.
                        Vector2 pixelPoint = new Vector2(px, py);
                        Vector2 physicalCoord = HomographyCalculator.TransformPixelToRobot(calibrationController.ActiveHomographyMatrix, pixelPoint);
                        
                        Debug.Log($"[VisionManager] Homography Mathed to Robot Coordinates: X:{physicalCoord.x}mm, Y:{physicalCoord.y}mm");

                        // 6. Send to Physical Robot asynchronously
                        string payload = tcpClient.FormatMelfaPositionString(physicalCoord.x, physicalCoord.y, pickHeightZ);
                        _ = tcpClient.SendCommandAsync(payload);

                        // 7. Digital Twin Synchronization
                        if (digitalTwinTarget != null && rosPublisher != null)
                        {
                            // Convert Mitsubishi Millimeters (Right-Handed) to Unity Meters (Left-Handed)
                            // Mitsubishi +X is Forward -> Unity +Z
                            // Mitsubishi +Y is Left -> Unity -X
                            // Mitsubishi +Z is Up -> Unity +Y
                            float unityZ = (physicalCoord.x / 1000f) + unityOriginOffset.z;
                            float unityX = (-physicalCoord.y / 1000f) + unityOriginOffset.x;
                            float unityY = (pickHeightZ / 1000f) + unityOriginOffset.y;

                            digitalTwinTarget.position = new Vector3(unityX, unityY, unityZ);
                            Debug.Log($"[Digital Twin] Teleported 3D Target to Unity coordinates: {digitalTwinTarget.position}");

                            // Trigger the 3D robot to move via ROS
                            rosPublisher.Publish();
                            Debug.Log("[Digital Twin] Triggered ROS 3D Pick Sequence!");
                        }
                    }
                    else
                    {
                        Debug.LogError("[VisionManager] Gemini failed to find the cube or coordinate parse failed.");
                    }
                }
            }
            
            m_IsProcessing = false;
        }
    }
}
