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

using System.Collections.Generic;
using DeusaldSharp;

namespace DeusaldNav2D
{
    public class Nav2D
    {
        #region Variables

        private Vector2        _LeftBottomMapCorner;
        private Vector2        _RightUpperMapCorner;
        
        private readonly List<Obstacle> _Obstacles;

        #endregion Variables

        #region Properties

        public float      AgentRadius { get; }
        public Obstacle[] Obstacles   => _Obstacles.ToArray();

        #endregion Properties

        #region Init Methods

        public Nav2D(Vector2 leftBottomMapCorner, Vector2 rightUpperMapCorner, float agentRadius)
        {
            _LeftBottomMapCorner = leftBottomMapCorner;
            _RightUpperMapCorner = rightUpperMapCorner;
            AgentRadius          = agentRadius;
            _Obstacles           = new List<Obstacle>();
        }

        #endregion Init Methods

        #region Public Methods

        public Obstacle AddObstacle(Vector2[] points, Vector2 position, float rotation, float extraOffset = 0f)
        {
            Obstacle newObstacle = new Obstacle(points, position, rotation, extraOffset, this);
            _Obstacles.Add(newObstacle);
            return newObstacle;
        }

        #endregion Public Methods
    }
}