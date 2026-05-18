using UnityEngine;
using UnityEngine.UI;

namespace MitsubishiVisionApp
{
    /// <summary>
    /// Handles the real-time physical video feed integration. This script acts as the "Eyes" of the system,
    /// establishing a hardware connection to the USB Webcam and providing raw image data to the AI Orchestrator.
    /// </summary>
    public class WebcamStream : MonoBehaviour
    {
        [Header("Settings")]
        public RawImage displayImage;
        public int desiredWidth = 1920;
        public int desiredHeight = 1080;
        
        private WebCamTexture m_WebcamTexture;

        void Start()
        {
            // Initialize the hardware connection. We select the first available camera device (index 0).
            if (WebCamTexture.devices.Length > 0)
            {
                WebCamDevice device = WebCamTexture.devices[0];
                
                // We enforce a strict resolution limit (e.g., 1080p).
                // This is critical because sending raw 4K textures to the AI would cause severe network latency
                // and potentially crash the Unity memory heap.
                m_WebcamTexture = new WebCamTexture(device.name, desiredWidth, desiredHeight);
                if (displayImage != null)
                {
                    displayImage.texture = m_WebcamTexture;
                }
                m_WebcamTexture.Play();
                Debug.Log($"[Webcam] Started {device.name} at {m_WebcamTexture.width}x{m_WebcamTexture.height}");
            }
            else
            {
                Debug.LogError("[Webcam] No hardware cameras found on this system!");
            }
        }

        /// <summary>
        /// Extracts the current frame from the live video stream.
        /// This creates a static 2D image (Texture2D) that can be encoded into a PNG payload
        /// and sent over the network to the Vision-Language Model.
        /// </summary>
        /// <returns>A static Texture2D representing the current physical environment.</returns>
        public Texture2D CaptureFrame()
        {
            if (m_WebcamTexture == null || !m_WebcamTexture.isPlaying) return null;

            Texture2D frame = new Texture2D(m_WebcamTexture.width, m_WebcamTexture.height);
            frame.SetPixels(m_WebcamTexture.GetPixels());
            frame.Apply();
            return frame;
        }

        void OnDestroy()
        {
            if (m_WebcamTexture != null)
            {
                m_WebcamTexture.Stop();
            }
        }
    }
}
