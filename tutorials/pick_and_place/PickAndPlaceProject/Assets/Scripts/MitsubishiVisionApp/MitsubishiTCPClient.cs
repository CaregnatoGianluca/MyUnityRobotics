using UnityEngine;
using System.Threading.Tasks;
using System.Net.Sockets;
using System.Text;

namespace MitsubishiVisionApp
{
    /// <summary>
    /// Handles the low-level network communication between the Unity Physics/AI engine
    /// and the legacy Mitsubishi industrial robot controller. 
    /// This acts as the translation layer from modern C# to MELFA-BASIC V.
    /// </summary>
    public class MitsubishiTCPClient : MonoBehaviour
    {
        [Header("TCP Socket Settings")]
        public string robotIP = "127.0.0.1";
        public int robotPort = 12345;

        private TcpListener m_Server;
        private TcpClient m_ConnectedRobot;
        private NetworkStream m_Stream;

        private bool m_MoveCompleted = false;

        void Start()
        {
            StartServer();
        }

        private async void StartServer()
        {
            try
            {
                // Unity is now the SERVER! It waits for the robot to connect to IT.
                m_Server = new TcpListener(System.Net.IPAddress.Any, robotPort);
                m_Server.Start();
                Debug.Log($"<color=green>[TCP Server]</color> Unity is listening on port {robotPort}. Waiting for Robot to connect...");

                // Wait for the robot to execute: Open "COM5:" As #1
                m_ConnectedRobot = await m_Server.AcceptTcpClientAsync();
                m_Stream = m_ConnectedRobot.GetStream();
                
                Debug.Log("<color=green>[TCP Server]</color> Robot connected successfully!");
                
                // Start listening to everything the robot says
                _ = ListenToRobotAsync();
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"<color=red>[TCP Server]</color> Failed to start server: {ex.Message}");
            }
        }

        private async Task ListenToRobotAsync()
        {
            byte[] buffer = new byte[1024];
            while (m_ConnectedRobot != null && m_ConnectedRobot.Connected && m_Stream != null)
            {
                try
                {
                    int bytesRead = await m_Stream.ReadAsync(buffer, 0, buffer.Length);
                    if (bytesRead > 0)
                    {
                        string response = Encoding.ASCII.GetString(buffer, 0, bytesRead).Trim();
                        Debug.Log($"<color=cyan>[Robot Says]</color> {response}");

                        if (response.Contains("MOVE COMPLETED"))
                        {
                            m_MoveCompleted = true;
                        }
                    }
                }
                catch
                {
                    break;
                }
            }
        }

        public async Task<bool> SendCommandAsync(string command)
        {
            if (m_Stream == null || !m_ConnectedRobot.Connected)
            {
                Debug.LogError("<color=red>[TCP Server]</color> Cannot send command! The Robot has not connected to Unity yet. Make sure you run the program in RT ToolBox3 first!");
                return false;
            }

            try
            {
                m_MoveCompleted = false; // Reset flag before sending

                // 1. Send the coordinates
                byte[] data = Encoding.ASCII.GetBytes(command);
                await m_Stream.WriteAsync(data, 0, data.Length);
                Debug.Log($"<color=green>[TCP Server]</color> Sent coordinates: {command.Replace("\r", "")}");

                // 2. Wait up to 10 seconds for the robot to complete the move
                int timeoutMs = 10000;
                int elapsedMs = 0;
                while (!m_MoveCompleted && elapsedMs < timeoutMs)
                {
                    await Task.Delay(100);
                    elapsedMs += 100;
                }

                if (m_MoveCompleted)
                {
                    Debug.Log("<color=green>[TCP Server]</color> Sequence finished successfully.");
                    return true;
                }
                else
                {
                    Debug.LogWarning("<color=red>[TCP Server]</color> Timed out waiting for MOVE COMPLETED.");
                    return false;
                }
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"<color=red>[TCP Server]</color> Network Error during transmission: {ex.Message}");
                return false;
            }
        }

        public string FormatMelfaPositionString(float x, float y, float z, float pitch = 180.0f)
        {
            int ix = Mathf.RoundToInt(x * 100f);
            int iy = Mathf.RoundToInt(y * 100f);
            int iz = Mathf.RoundToInt(z * 100f);
            
            // Exactly matching the Python repo: Send raw comma-separated variables without a newline.
            return $"{ix},{iy},{iz}";
        }

        void OnDestroy()
        {
            if (m_Stream != null) m_Stream.Close();
            if (m_ConnectedRobot != null) m_ConnectedRobot.Close();
            if (m_Server != null) m_Server.Stop();
        }
    }
}
