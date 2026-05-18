using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.PickAndPlace
{
    /// <summary>
    /// This script acts as the "Goal Area" sensor. It uses Unity Physics (Colliders and Rigidbodies) 
    /// to determine if the robotic arm has successfully picked up the Target cube and placed it 
    /// inside this destination zone. It visually changes color based on the cube's state.
    /// </summary>
    [RequireComponent(typeof(MeshRenderer))]
    [RequireComponent(typeof(BoxCollider))]
    public class TargetPlacement : MonoBehaviour
    {
        const string k_NameExpectedTarget = "Target";
        
        // Shader property ID used to change the color of the goal area efficiently
        static readonly int k_ShaderColorId = Shader.PropertyToID("_Color");
        
        // The maximum velocity (magnitude) the cube can have to be considered "at rest" (placed).
        const float k_MaximumSpeedForStopped = 0.01f;

        [SerializeField]
        [Tooltip("The specific 3D cube we are trying to pick and place.")]
        GameObject m_Target;
        
        [SerializeField]
        [Range(0, 255)]
        [Tooltip("Transparency of the goal area (0 = invisible, 255 = solid).")]
        int m_ColorAlpha = 100;

        // References to the visual and physical components of both the Goal Area and the Cube
        MeshRenderer m_TargetMeshRenderer;
        float m_ColorAlpha01 => m_ColorAlpha / 255f; // Converts 0-255 alpha to 0.0-1.0 float for shaders
        MeshRenderer m_MeshRenderer;
        BoxCollider m_BoxCollider;
        
        // State Machine variables tracking the current status of the cube
        PlacementState m_CurrentState;
        PlacementState m_LastColoredState;

        // An event that fires when the cube is successfully placed. Other scripts can subscribe to this.
        public event Action OnTargetPlaced;

        /// <summary>
        /// Property that automatically handles color updates and event firing whenever the state changes.
        /// </summary>
        public PlacementState CurrentState
        {
            get => m_CurrentState;
            private set
            {
                if (m_CurrentState != value) // Only trigger if the state actually changes
                {
                    m_CurrentState = value;
                    UpdateStateColor(); // Change visual color (Red -> Yellow -> Green)
                    
                    if (m_CurrentState == PlacementState.InsidePlaced)
                    {
                        OnTargetPlaced?.Invoke(); // Fire the success event!
                    }
                }
            }
        }

        public enum PlacementState
        {
            Outside,          // Cube is far away (Color: Red)
            InsideFloating,   // Cube is inside the goal zone, but still moving/held by robot (Color: Yellow)
            InsidePlaced      // Cube is inside the goal zone AND has stopped moving (Color: Green)
        }

        // Start is called before the first frame update
        void Start()
        {
            // Check for mis-configurations and disable if something has changed without this script being updated
            // These are warnings because this script does not contain critical functionality
            if (m_Target == null)
            {
                m_Target = GameObject.Find(k_NameExpectedTarget);
            }

            if (m_Target == null)
            {
                Debug.LogWarning($"{nameof(TargetPlacement)} expects to find a GameObject named " +
                    $"{k_NameExpectedTarget} to track, but did not. Can't track placement state.");
                enabled = false;
                return;
            }

            if (!TrySetComponentReferences())
            {
                enabled = false;
                return;
            }
            InitializeState();
        }

        bool TrySetComponentReferences()
        {
            m_TargetMeshRenderer = m_Target.GetComponent<MeshRenderer>();
            if (m_TargetMeshRenderer == null)
            {
                Debug.LogWarning($"{nameof(TargetPlacement)} expects a {nameof(MeshRenderer)} to be attached " +
                    $"to {k_NameExpectedTarget}. Cannot check bounds without it, so cannot track placement state.");
                return false;
            }

            // Assume these are here because they are RequiredComponent components
            m_MeshRenderer = GetComponent<MeshRenderer>();
            m_BoxCollider = GetComponent<BoxCollider>();
            return true;
        }

        void OnValidate()
        {
            // Useful for visualizing state in editor, but doesn't wholly guarantee accurate coloring in EditMode
            // Enter PlayMode to see color update correctly
            if (m_Target != null)
            {
                if (TrySetComponentReferences())
                {
                    InitializeState();
                }
            }
        }

        void InitializeState()
        {
            // At startup, perform a hard mathematical check to see if the bounding box of the cube 
            // is intersecting with the bounding box of this goal area.
            if (m_Target.GetComponent<BoxCollider>().bounds.Intersects(m_BoxCollider.bounds))
            {
                CurrentState = IsTargetStoppedInsideBounds() ?
                    PlacementState.InsidePlaced : PlacementState.InsideFloating;
            }
            else
            {
                CurrentState = PlacementState.Outside;
            }
        }

        /// <summary>
        /// Unity Physics Event: Fires automatically the exact frame the cube's collider touches the goal area.
        /// </summary>
        void OnTriggerEnter(Collider other)
        {
            if (other.gameObject.name == m_Target.name)
            {
                CurrentState = PlacementState.InsideFloating; // Cube has entered, turn Yellow
            }
        }

        /// <summary>
        /// Unity Physics Event: Fires automatically the exact frame the cube's collider leaves the goal area.
        /// </summary>
        void OnTriggerExit(Collider other)
        {
            if (other.gameObject.name == m_Target.name)
            {
                CurrentState = PlacementState.Outside; // Cube has left, turn Red
            }
        }

        /// <summary>
        /// Calculates if the cube is fully inside the goal area AND has zero kinetic energy.
        /// </summary>
        bool IsTargetStoppedInsideBounds()
        {
            // 1. Check Kinetic Energy: Is the magnitude of its velocity vector near zero?
            var targetIsStopped = m_Target.GetComponent<Rigidbody>().velocity.magnitude < k_MaximumSpeedForStopped;
            
            // 2. Check Spatial Positioning: Is the absolute center of the cube entirely inside our goal box?
            var targetIsInBounds = m_BoxCollider.bounds.Contains(m_TargetMeshRenderer.bounds.center);

            return targetIsStopped && targetIsInBounds;
        }

        // Update is called once per frame (Runs continuously)
        void Update()
        {
            // If the cube is currently inside the zone (Yellow or Green), continuously check its velocity.
            // If it drops to 0, it promotes from Floating (Yellow) to Placed (Green).
            if (CurrentState != PlacementState.Outside)
            {
                CurrentState = IsTargetStoppedInsideBounds() ?
                    PlacementState.InsidePlaced : PlacementState.InsideFloating;
            }
        }

        /// <summary>
        /// Visually updates the material color of the goal area without instantiating new materials.
        /// </summary>
        void UpdateStateColor()
        {
            if (m_CurrentState == m_LastColoredState)
            {
                return; // Prevent redundant GPU calls
            }

            // MaterialPropertyBlocks are the most highly-optimized way to change a color in Unity.
            // It modifies the GPU draw call directly without creating garbage collection overhead.
            var mpb = new MaterialPropertyBlock();
            Color stateColor;
            
            switch (m_CurrentState)
            {
                case PlacementState.Outside:
                    stateColor = Color.red; // Failure / Waiting
                    break;
                case PlacementState.InsideFloating:
                    stateColor = Color.yellow; // In Progress
                    break;
                case PlacementState.InsidePlaced:
                    stateColor = Color.green; // Success
                    break;
                default:
                    Debug.LogError($"No state handling implemented for {m_CurrentState}");
                    stateColor = Color.magenta;
                    break;
            }

            stateColor.a = m_ColorAlpha01; // Apply transparency
            mpb.SetColor(k_ShaderColorId, stateColor);
            m_MeshRenderer.SetPropertyBlock(mpb); // Push color to GPU
            m_LastColoredState = m_CurrentState;
        }
    }
}
