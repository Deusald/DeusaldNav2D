// MIT License

// DeusaldNav2D:
// Copyright (c) 2020 Adam "Deusald" Orliński

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

using System;
using System.Collections.Generic;
using ClipperLib;
using DeusaldSharp;

namespace DeusaldNav2D
{
    public class Nav2D
    {
        #region Types

        public enum Accuracy
        {
            NoPoint     = 1,
            OnePoint    = 10,
            TwoPoints   = 100,
            ThreePoints = 1000,
            FourPoints  = 10000,
            FivePoints  = 100000
        }

        #endregion Types

        #region Variables

        private Vector2              _LeftBottomMapCorner;
        private Vector2              _RightUpperMapCorner;
        private bool                 _AreObstaclesDirty;
        private List<List<IntPoint>> _ExitExtendPoints;

        private readonly List<Obstacle> _Obstacles;
        private readonly Accuracy       _Accuracy;
        private readonly ClipperOffset  _ClipperOffset;

        #endregion Variables

        #region Properties

        public float      AgentRadius { get; }
        public Obstacle[] Obstacles   => _Obstacles.ToArray();

        #endregion Properties

        #region Init Methods

        public Nav2D(Vector2 leftBottomMapCorner, Vector2 rightUpperMapCorner, float agentRadius, Accuracy accuracy)
        {
            _LeftBottomMapCorner = leftBottomMapCorner;
            _RightUpperMapCorner = rightUpperMapCorner;
            AgentRadius          = agentRadius;
            _Obstacles           = new List<Obstacle>();
            _ClipperOffset       = new ClipperOffset();
            _ExitExtendPoints    = new List<List<IntPoint>>();
            _Accuracy            = accuracy;
        }

        #endregion Init Methods

        #region Public Methods

        public void Update()
        {
            if (!_AreObstaclesDirty) return;

            foreach (var obstacle in _Obstacles)
                obstacle.RefreshObstacle();
        }

        public Obstacle AddObstacle(Vector2[] points, Vector2 position, float rotation, float extraOffset = 0f)
        {
            Obstacle newObstacle = new Obstacle(points, position, rotation, extraOffset, this);
            _Obstacles.Add(newObstacle);
            newObstacle.DirtyFlagEnabled += MarkObstaclesDirty;
            return newObstacle;
        }

        public Obstacle AddObstacle(float radius, Vector2 position, float extraOffset = 0f)
        {
            Vector2[] points      = GetHexagonPoints(radius);
            Obstacle  newObstacle = new Obstacle(points, position, 0f, extraOffset, this);
            _Obstacles.Add(newObstacle);
            newObstacle.DirtyFlagEnabled += MarkObstaclesDirty;
            return newObstacle;
        }

        internal IntPoint ParseToIntPoint(Vector2 vector2)
        {
            int x = (int) MathF.Round(vector2.x * (float) _Accuracy);
            int y = (int) MathF.Round(vector2.y * (float) _Accuracy);
            return new IntPoint(x, y);
        }

        internal Vector2 ParseToVector2(IntPoint intPoint)
        {
            float x = intPoint.X / (float) _Accuracy;
            float y = intPoint.Y / (float) _Accuracy;
            return new Vector2(x, y);
        }

        internal void ExtendThePolygon(ref List<IntPoint> enterPoints, ref Vector2[] extendedPoints, float extraOffset)
        {
            double distance = (AgentRadius + extraOffset) * (float) _Accuracy;
            _ClipperOffset.Clear();
            _ExitExtendPoints.Clear();

            _ClipperOffset.AddPath(enterPoints, JoinType.jtMiter, EndType.etClosedPolygon);
            _ClipperOffset.Execute(ref _ExitExtendPoints, distance);

            if (_ExitExtendPoints[0].Count != extendedPoints.Length)
                extendedPoints = new Vector2[_ExitExtendPoints[0].Count];

            for (int i = 0; i < _ExitExtendPoints[0].Count; ++i)
                extendedPoints[i] = ParseToVector2(_ExitExtendPoints[0][i]);
        }

        #endregion Public Methods

        #region Private Methods

        private void MarkObstaclesDirty(object sender, EventArgs eventArgs)
        {
            _AreObstaclesDirty = true;
        }

        private Vector2[] GetHexagonPoints(float radius)
        {
            const int   hexagonPoints = 6;
            const float sqrt3         = 1.73f;
            Vector2     point         = Vector2.Up * (2f * radius / sqrt3);
            Vector2[]   result        = new Vector2[hexagonPoints];
            point     = Vector2.Rotate(30f * MathUtils.DegToRad, point);
            result[0] = point;

            for (int i = 1; i < hexagonPoints; ++i)
            {
                point     = Vector2.Rotate(60f * MathUtils.DegToRad, point);
                result[i] = point;
            }

            return result;
        }

        #endregion Private Methods
    }
}