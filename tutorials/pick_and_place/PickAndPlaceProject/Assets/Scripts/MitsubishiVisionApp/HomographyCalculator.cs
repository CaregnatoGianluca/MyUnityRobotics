using UnityEngine;
using System;

namespace MitsubishiVisionApp
{
    /// <summary>
    /// A pure C# mathematical solver that entirely eliminates the need for heavy 3D computer vision libraries like OpenCV.
    /// It calculates a Planar Homography Matrix to map 2D pixels directly to physical robot coordinates.
    /// </summary>
    public static class HomographyCalculator
    {
        /// <summary>
        /// Calculates the 3x3 Homography Transformation Matrix from 4 source pixels to 4 physical robot coordinates.
        /// By taking exactly 4 points, this creates a strictly determined system of 8 linear equations.
        /// </summary>
        /// <returns>A 1D array of 9 floats representing the 3x3 matrix.</returns>
        public static float[] FindHomography(Vector2[] src, Vector2[] dst)
        {
            if (src.Length != 4 || dst.Length != 4)
                throw new ArgumentException("Exactly 4 points are required for homography.");

            float[,] P = new float[8, 9];

            for (int i = 0; i < 4; i++)
            {
                float x = src[i].x;
                float y = src[i].y;
                float u = dst[i].x;
                float v = dst[i].y;

                P[i * 2, 0] = -x;
                P[i * 2, 1] = -y;
                P[i * 2, 2] = -1;
                P[i * 2, 3] = 0;
                P[i * 2, 4] = 0;
                P[i * 2, 5] = 0;
                P[i * 2, 6] = x * u;
                P[i * 2, 7] = y * u;
                P[i * 2, 8] = -u;

                P[i * 2 + 1, 0] = 0;
                P[i * 2 + 1, 1] = 0;
                P[i * 2 + 1, 2] = 0;
                P[i * 2 + 1, 3] = -x;
                P[i * 2 + 1, 4] = -y;
                P[i * 2 + 1, 5] = -1;
                P[i * 2 + 1, 6] = x * v;
                P[i * 2 + 1, 7] = y * v;
                P[i * 2 + 1, 8] = -v;
            }

            GaussianElimination(P, 8);

            float[] H = new float[9];
            for (int i = 0; i < 8; i++)
            {
                H[i] = P[i, 8];
            }
            H[8] = 1.0f; // h33 is 1

            return H;
        }

        /// <summary>
        /// A custom implementation of Gaussian Elimination to solve the 8x8 system of linear equations.
        /// This is required because C# lacks a native matrix solver. It avoids importing heavy numerical libraries,
        /// keeping the system lightweight and Unity-native.
        /// </summary>
        private static void GaussianElimination(float[,] A, int n)
        {
            for (int i = 0; i < n; i++)
            {
                // Search for maximum in this column
                float maxEl = Math.Abs(A[i, i]);
                int maxRow = i;
                for (int k = i + 1; k < n; k++)
                {
                    if (Math.Abs(A[k, i]) > maxEl)
                    {
                        maxEl = Math.Abs(A[k, i]);
                        maxRow = k;
                    }
                }

                // Swap maximum row with current row
                for (int k = i; k < n + 1; k++)
                {
                    float tmp = A[maxRow, k];
                    A[maxRow, k] = A[i, k];
                    A[i, k] = tmp;
                }

                // Make all rows below this one 0 in current column
                for (int k = i + 1; k < n; k++)
                {
                    float c = -A[k, i] / A[i, i];
                    for (int j = i; j < n + 1; j++)
                    {
                        if (i == j)
                            A[k, j] = 0;
                        else
                            A[k, j] += c * A[i, j];
                    }
                }
            }

            // Solve equation Ax=b for an upper triangular matrix A
            for (int i = n - 1; i >= 0; i--)
            {
                A[i, n] = A[i, n] / A[i, i];
                A[i, i] = 1;
                for (int j = i - 1; j >= 0; j--)
                {
                    A[j, n] -= A[j, i] * A[i, n];
                    A[j, i] = 0;
                }
            }
        }

        /// <summary>
        /// Converts a single 2D pixel coordinate (returned by the AI) into a 2D physical robot millimeter coordinate.
        /// This mathematical translation implicitly handles camera rotation, skew, and Z-height, 
        /// meaning the user never has to physically measure the camera's exact placement.
        /// </summary>
        public static Vector2 TransformPixelToRobot(float[] H, Vector2 pixel)
        {
            float x = pixel.x;
            float y = pixel.y;

            float u = H[0] * x + H[1] * y + H[2];
            float v = H[3] * x + H[4] * y + H[5];
            float w = H[6] * x + H[7] * y + H[8];

            if (Math.Abs(w) > 1e-6f)
            {
                return new Vector2(u / w, v / w);
            }
            return Vector2.zero; // Degenerate case
        }
    }
}
