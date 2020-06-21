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
using ClipperLib;
using DeusaldSharp;

namespace DeusaldNav2D
{
    internal class NavShape
    {
        #region Properties

        public Vector2[]       Points   { get; }
        public bool            Hole     { get; }
        public List<NavShape>  Children { get; }
        public NavShape        Parent   { get; }
        public NavElement.Type NavType  { get; }

        #endregion Properties

        #region Init Methods

        public NavShape(Nav2D nav2D, List<IntPoint> points, bool hole, List<PolyNode> children, NavShape parent, NavElement.Type navType)
        {
            Points = new Vector2[points.Count];

            for (int i = 0; i < Points.Length; ++i)
                Points[i] = nav2D.ParseToVector2(points[i]);

            Hole    = hole;
            NavType = navType;
            Parent  = parent;

            if (children == null || children.Count == 0)
            {
                Children = null;
                return;
            }

            Children = new List<NavShape>();

            foreach (var child in children)
                Children.Add(new NavShape(nav2D, child.Contour, child.IsHole, child.Childs, this, navType));
        }

        public NavShape(Vector2[] points, NavElement.Type navType)
        {
            Points   = points;
            Hole     = false;
            Children = null;
            Parent   = null;
            NavType  = navType;
        }

        #endregion Init Methods
    }
}