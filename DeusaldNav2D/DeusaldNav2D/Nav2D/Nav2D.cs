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
using System.Drawing;
using ClipperLib;
using DeusaldSharp;
using QuadTrees;

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

        public class ConnectionData { }

        #endregion Types

        #region Variables

        internal readonly Dictionary<uint, ElementsGroup> elementsGroups;
        internal readonly List<uint>                      elementsGroupToRebuild;
        internal readonly Clipper                         clipper;
        internal readonly PolyTree                        polyTree;

        private bool                 _AreObstaclesDirty;
        private bool                 _AreSurfacesDirty;
        private List<List<IntPoint>> _ExitExtendPoints;
        private uint                 _NextGroupId;
        private uint                 _ConnectionOrderId;

        private readonly Vector2                                               _LeftBottomMapCorner;
        private readonly Vector2                                               _RightUpperMapCorner;
        private readonly List<NavElement>                                      _Obstacles;
        private readonly List<NavElement>                                      _Surfaces;
        private readonly Accuracy                                              _Accuracy;
        private readonly ClipperOffset                                         _ClipperOffset;
        private readonly Queue<NavElement>                                     _ElementsToRebuildGroups;
        private readonly HashSet<NavElement>                                   _ElementsOnRebuildGroupsQueue;
        private readonly List<NavPoint>                                        _NavPoints;
        private readonly Dictionary<Tuple<NavPoint, NavPoint>, ConnectionData> _Connections;

        #endregion Variables

        #region Properties

        #if DEBUG

        public List<NavElement> DebugObstacles => _Obstacles;
        public List<NavElement> DebugSurfaces  => _Surfaces;
        public List<NavPoint>   DebugNavPoints => _NavPoints;

        #endif

        public float        AgentRadius         { get; }
        public Vector2      LeftBottomMapCorner => _LeftBottomMapCorner;
        public Vector2      RightUpperMapCorner => _RightUpperMapCorner;
        public NavElement[] Obstacles           => _Obstacles.ToArray();
        public NavElement[] Surfaces            => _Surfaces.ToArray();

        internal QuadTreeRectF<NavElement> QuadTree    { get; }
        internal uint                      NextGroupId => _NextGroupId++;

        #endregion Properties

        #region Init Methods

        public Nav2D(Vector2 leftBottomMapCorner, Vector2 rightUpperMapCorner, float agentRadius, Accuracy accuracy)
        {
            _LeftBottomMapCorner          = leftBottomMapCorner;
            _RightUpperMapCorner          = rightUpperMapCorner;
            AgentRadius                   = agentRadius;
            _NextGroupId                  = 1;
            _Obstacles                    = new List<NavElement>();
            _Surfaces                     = new List<NavElement>();
            _ClipperOffset                = new ClipperOffset();
            _ExitExtendPoints             = new List<List<IntPoint>>();
            elementsGroups                = new Dictionary<uint, ElementsGroup>();
            _ElementsToRebuildGroups      = new Queue<NavElement>();
            _ElementsOnRebuildGroupsQueue = new HashSet<NavElement>();
            _Connections                  = new Dictionary<Tuple<NavPoint, NavPoint>, ConnectionData>();
            elementsGroupToRebuild        = new List<uint>();
            _NavPoints                    = new List<NavPoint>();
            clipper                       = new Clipper();
            polyTree                      = new PolyTree();
            QuadTree                      = new QuadTreeRectF<NavElement>(GetRectFromMinMax(leftBottomMapCorner, rightUpperMapCorner));
            _Accuracy                     = accuracy;
        }

        #endregion Init Methods

        #region Public Methods

        public void Update()
        {
            if (!_AreObstaclesDirty && !_AreSurfacesDirty) return;

            if (_AreObstaclesDirty)
            {
                foreach (var obstacle in _Obstacles)
                    obstacle.RefreshNavElement();
            }

            if (_AreSurfacesDirty)
            {
                foreach (var surface in _Surfaces)
                    surface.RefreshNavElement();
            }

            while (_ElementsToRebuildGroups.Count != 0)
                _ElementsToRebuildGroups.Dequeue().RebuildTheElementGroup();

            _ElementsOnRebuildGroupsQueue.Clear();

            foreach (var id in elementsGroupToRebuild)
            {
                if (!elementsGroups.ContainsKey(id)) continue;
                elementsGroups[id].Rebuild();
            }

            elementsGroupToRebuild.Clear();

            RebuildNavPoints();
        }

        public NavElement AddObstacle(Vector2[] points, Vector2 position, float rotation, float extraOffset = 0f)
        {
            NavElement newObstacle = new NavElement(points, position, rotation, extraOffset, this, NavElement.Type.Obstacle, 1f);
            _Obstacles.Add(newObstacle);
            newObstacle.DirtyFlagEnabled += MarkObstaclesDirty;
            return newObstacle;
        }

        public NavElement AddObstacle(float radius, Vector2 position, float extraOffset = 0f)
        {
            Vector2[]  points      = GetHexagonPoints(radius);
            NavElement newObstacle = new NavElement(points, position, 0f, extraOffset, this, NavElement.Type.Obstacle, 1f);
            _Obstacles.Add(newObstacle);
            newObstacle.DirtyFlagEnabled += MarkObstaclesDirty;
            return newObstacle;
        }

        public NavElement AddSurface(Vector2[] points, Vector2 position, float rotation, float cost, float extraOffset = 0f)
        {
            NavElement newSurface = new NavElement(points, position, rotation, extraOffset, this, NavElement.Type.Surface, cost);
            _Surfaces.Add(newSurface);
            newSurface.DirtyFlagEnabled += MarkSurfacesDirty;
            return newSurface;
        }

        public NavElement AddSurface(float radius, Vector2 position, float cost, float extraOffset = 0f)
        {
            Vector2[]  points     = GetHexagonPoints(radius);
            NavElement newSurface = new NavElement(points, position, 0f, extraOffset, this, NavElement.Type.Surface, cost);
            _Surfaces.Add(newSurface);
            newSurface.DirtyFlagEnabled += MarkSurfacesDirty;
            return newSurface;
        }

        public void RemoveNavElement(NavElement navElement)
        {
            if (navElement.NavType == NavElement.Type.Obstacle)
            {
                navElement.DirtyFlagEnabled -= MarkObstaclesDirty;
                elementsGroups[navElement.ElementGroupId].DismantleGroup(true);
                _Obstacles.Remove(navElement);
            }
            else if (navElement.NavType == NavElement.Type.Surface)
            {
                navElement.DirtyFlagEnabled -= MarkSurfacesDirty;
                elementsGroups[navElement.ElementGroupId].DismantleGroup(true);
                _Surfaces.Remove(navElement);
            }
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

        internal RectangleF GetRectFromMinMax(Vector2 leftBottom, Vector2 rightTop)
        {
            float x      = (leftBottom.x + rightTop.x) / 2f;
            float y      = (leftBottom.y + rightTop.y) / 2f;
            float width  = MathF.Abs(leftBottom.x - rightTop.x);
            float height = MathF.Abs(leftBottom.y - rightTop.y);
            return new RectangleF(x, y, width, height);
        }

        internal void AddElementOnRebuildGroupsQueue(NavElement element)
        {
            if (_ElementsOnRebuildGroupsQueue.Contains(element)) return;
            _ElementsOnRebuildGroupsQueue.Add(element);
            _ElementsToRebuildGroups.Enqueue(element);
        }

        #endregion Public Methods

        #region Private Methods

        private void MarkObstaclesDirty(object sender, EventArgs eventArgs)
        {
            _AreObstaclesDirty = true;
        }

        private void MarkSurfacesDirty(object sender, EventArgs eventArgs)
        {
            _AreSurfacesDirty = true;
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

        private void AddToConnectionDictionary(NavPoint first, NavPoint second)
        {
            if (first.Id < second.Id)
                _Connections.Add(new Tuple<NavPoint, NavPoint>(first, second), new ConnectionData());
            else
                _Connections.Add(new Tuple<NavPoint, NavPoint>(second, first), new ConnectionData());
        }

        private void RebuildNavPoints()
        {
            _NavPoints.Clear();
            _Connections.Clear();
            _ConnectionOrderId = 0;
            CreateEdgePoints();
        }

        private void CreateEdgePoints()
        {
            foreach (var element in elementsGroups.Values)
            {
                for (var i = 0; i < element.NavSurfaces.Count; ++i)
                {
                    NavShape          surface             = element.NavSurfaces[i];
                    HashSet<NavPoint> forbiddenConnection = new HashSet<NavPoint>();

                    FillTheEdgePoints(ref surface, ref forbiddenConnection);
                }

                Queue<NavShape>                         obstacles                 = new Queue<NavShape>(element.NavObstacles);
                Dictionary<NavShape, HashSet<NavPoint>> parentForbiddenConnection = new Dictionary<NavShape, HashSet<NavPoint>>();

                while (obstacles.Count != 0)
                {
                    NavShape          obstacle            = obstacles.Dequeue();
                    HashSet<NavPoint> forbiddenConnection = obstacle.Hole ? parentForbiddenConnection[obstacle.Parent] : new HashSet<NavPoint>();
                    
                    FillTheEdgePoints(ref obstacle, ref forbiddenConnection);
                    
                    if (!obstacle.Hole)
                        parentForbiddenConnection.Add(obstacle, forbiddenConnection);

                    if (obstacle.Children == null) continue;
                    
                    foreach (var child in obstacle.Children)
                        obstacles.Enqueue(child);
                }
            }
        }

        private void FillTheEdgePoints(ref NavShape navShape, ref HashSet<NavPoint> forbiddenConnection)
        {
            NavPoint lastPoint     = new NavPoint(_ConnectionOrderId++, navShape.Points[navShape.Points.Length - 1], forbiddenConnection);
            NavPoint previousPoint = lastPoint;

            for (int i = 0; i < navShape.Points.Length - 1; ++i)
            {
                NavPoint point = new NavPoint(_ConnectionOrderId++, navShape.Points[i], forbiddenConnection);

                point.Neighbours.Add(previousPoint);
                previousPoint.Neighbours.Add(point);
                AddToConnectionDictionary(previousPoint, point);
                forbiddenConnection.Add(point);
                previousPoint = point;
            }

            previousPoint.Neighbours.Add(lastPoint);
            lastPoint.Neighbours.Add(previousPoint);
            AddToConnectionDictionary(previousPoint, lastPoint);
            forbiddenConnection.Add(lastPoint);
        }

        #endregion Private Methods
    }
}