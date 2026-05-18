using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using System.Collections.Generic;

namespace MitsubishiVisionApp
{
    /// <summary>
    /// Manages the User Interface (UI) interaction for the Camera Calibration phase.
    /// It captures the user's mouse clicks on the raw video feed to generate the 4 digital pixel coordinates
    /// required to calculate the Homography Transformation Matrix.
    /// </summary>
    public class CalibrationController : MonoBehaviour, IPointerClickHandler
    {
        [Header("UI References")]
        [Tooltip("The UI RawImage that is playing the WebCamTexture")]
        public RawImage webcamDisplay;
        
        [Header("Physical World Coordinates (Millimeters)")]
        [Tooltip("The EXACT physical X,Y coordinates of the 4 pieces of tape on the table, measured by the robot teach pendant.")]
        public Vector2[] robotCalibrationPoints = new Vector2[4] {
            new Vector2(250, -100), // Bottom Left Tape
            new Vector2(250, 100),  // Bottom Right Tape
            new Vector2(400, 100),  // Top Right Tape
            new Vector2(400, -100)  // Top Left Tape
        };

        private List<Vector2> m_ClickedPixels = new List<Vector2>();
        private int m_CalibrationIndex = 0;
        
        // This is the magic 3x3 matrix. If it is null, the system is not calibrated.
        public float[] ActiveHomographyMatrix { get; private set; }

        /// <summary>
        /// Captures the mouse click on the RawImage UI element.
        /// It mathematically maps the UI Canvas RectTransform coordinates back into the raw pixel space 
        /// of the underlying Webcam texture.
        /// </summary>
        public void OnPointerClick(PointerEventData eventData)
        {
            if (m_CalibrationIndex >= 4)
            {
                Debug.LogWarning("[Calibration] Already clicked 4 points. Click 'Reset' if you want to recalibrate.");
                return;
            }

            // Convert the screen click into a local UI coordinate
            RectTransform rectTransform = webcamDisplay.rectTransform;
            if (RectTransformUtility.ScreenPointToLocalPointInRectangle(rectTransform, eventData.position, eventData.pressEventCamera, out Vector2 localPoint))
            {
                // Normalize local point to a 0-1 percentage
                Vector2 normalizedPoint = new Vector2(
                    (localPoint.x - rectTransform.rect.x) / rectTransform.rect.width,
                    (localPoint.y - rectTransform.rect.y) / rectTransform.rect.height
                );

                // Convert that percentage into the raw underlying Texture pixels
                Texture texture = webcamDisplay.texture;
                if (texture != null)
                {
                    // Note: Unity UI origins are bottom-left, Gemini expects top-left.
                    // We flip the Y axis here so the pixel perfectly matches Gemini's perspective.
                    Vector2 pixelCoord = new Vector2(
                        normalizedPoint.x * texture.width,
                        (1.0f - normalizedPoint.y) * texture.height
                    );

                    m_ClickedPixels.Add(pixelCoord);
                    m_CalibrationIndex++;
                    Debug.Log($"[Calibration] Captured Point {m_ClickedPixels.Count} at Pixel: {pixelCoord}");

                    if (m_CalibrationIndex == 4)
                    {
                        CalculateHomography();
                    }
                }
            }
        }

        /// <summary>
        /// Calls the custom Math solver to generate the transformation matrix.
        /// Once this completes, the system is permanently calibrated and ready for AI integration.
        /// </summary>
        private void CalculateHomography()
        {
            Debug.Log("[Calibration] 4 points collected! Computing 8x8 Matrix...");
            
            try
            {
                ActiveHomographyMatrix = HomographyCalculator.FindHomography(m_ClickedPixels.ToArray(), robotCalibrationPoints);
                Debug.Log("[Calibration] SUCCESS! Homography Matrix Calculated.");
            }
            catch (System.Exception e)
            {
                Debug.LogError($"[Calibration] Math Failed: {e.Message}");
            }
        }

        public void ResetCalibration()
        {
            m_ClickedPixels.Clear();
            ActiveHomographyMatrix = null;
            Debug.Log("[Calibration] Memory cleared. Please click the 4 pieces of tape again.");
        }
    }
}
