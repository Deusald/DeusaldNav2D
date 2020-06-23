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
using QuadTree;

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

        #if DEBUG
        #pragma warning disable 67

        public Action<string> DebugLog;

        #pragma warning restore 67
        #endif

        internal readonly List<ElementsGroup>  elementsGroups;
        internal readonly Clipper              clipper;
        internal readonly PolyTree             polyTree;
        internal readonly Stack<ElementsGroup> elementsGroupPull;

        private bool                 _AreObstaclesDirty;
        private bool                 _AreSurfacesDirty;
        private List<List<IntPoint>> _ExitExtendPoints;
        private uint                 _ConnectionOrderId;
        private uint                 _NextElementsGroupId;

        private readonly Vector2                                               _LeftBottomMapCorner;
        private readonly Vector2                                               _RightUpperMapCorner;
        private readonly List<NavElement>                                      _Obstacles;
        private readonly List<NavElement>                                      _Surfaces;
        private readonly Accuracy                                              _Accuracy;
        private readonly ClipperOffset                                         _ClipperOffset;
        private readonly List<NavPoint>                                        _NavPoints;
        private readonly Dictionary<Tuple<NavPoint, NavPoint>, ConnectionData> _Connections;
        private readonly HashSet<NavElement>                                   _TmpGroupedElements;
        private readonly List<NavElement>                                      _TmpLocalGroupedElements;
        private readonly Queue<NavElement>                                     _TmpCollidedElements;
        private readonly List<NavElement>                                      _TmpSearchList;
        private readonly Dictionary<uint, ElementsGroup>                       _TmpElementsGroups;

        private const float _MinWorldAreaSize = 1f;

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

        internal QuadTree<NavElement> QuadTree            { get; }
        internal uint                 NextElementsGroupId => _NextElementsGroupId++;

        #endregion Properties

        #region Init Methods

        public Nav2D(Vector2 leftBottomMapCorner, Vector2 rightUpperMapCorner, float agentRadius, Accuracy accuracy)
        {
            if (new Quad(leftBottomMapCorner, rightUpperMapCorner).Area() < _MinWorldAreaSize)
                throw new Exception("World is too small! The world minimum area is 1 m^2");

            _LeftBottomMapCorner     = leftBottomMapCorner;
            _RightUpperMapCorner     = rightUpperMapCorner;
            AgentRadius              = agentRadius;
            _Obstacles               = new List<NavElement>();
            _Surfaces                = new List<NavElement>();
            _ClipperOffset           = new ClipperOffset();
            _ExitExtendPoints        = new List<List<IntPoint>>();
            elementsGroups           = new List<ElementsGroup>();
            _Connections             = new Dictionary<Tuple<NavPoint, NavPoint>, ConnectionData>();
            _NavPoints               = new List<NavPoint>();
            _TmpGroupedElements      = new HashSet<NavElement>();
            _TmpLocalGroupedElements = new List<NavElement>();
            _TmpCollidedElements     = new Queue<NavElement>();
            _TmpSearchList           = new List<NavElement>();
            _TmpElementsGroups       = new Dictionary<uint, ElementsGroup>();
            elementsGroupPull        = new Stack<ElementsGroup>();
            clipper                  = new Clipper();
            polyTree                 = new PolyTree();
            QuadTree                 = new QuadTree<NavElement>(10, 6, GetQuadTreeBounds(_LeftBottomMapCorner, _RightUpperMapCorner));
            _Accuracy                = accuracy;
            _NextElementsGroupId     = 1;
        }

        #endregion Init Methods

        #region Public Methods

        public void Update()
        {
            if (!_AreObstaclesDirty && !_AreSurfacesDirty) return;
            InnerUpdate(false);
        }

        public NavElement AddObstacle(Vector2[] points, Vector2 position, float rotation, float extraOffset = 0f)
        {
            NavElement newObstacle = new NavElement(points, position, rotation, extraOffset, this, NavElement.Type.Obstacle, 1f);
            _Obstacles.Add(newObstacle);
            newObstacle.DirtyFlagEnabled += MarkObstaclesDirty;
            _AreObstaclesDirty           =  true;
            return newObstacle;
        }

        public NavElement AddObstacle(float radius, Vector2 position, float extraOffset = 0f)
        {
            Vector2[]  points      = GetHexagonPoints(radius);
            NavElement newObstacle = new NavElement(points, position, 0f, extraOffset, this, NavElement.Type.Obstacle, 1f);
            _Obstacles.Add(newObstacle);
            newObstacle.DirtyFlagEnabled += MarkObstaclesDirty;
            _AreObstaclesDirty           =  true;
            return newObstacle;
        }

        public NavElement AddSurface(Vector2[] points, Vector2 position, float rotation, float cost, float extraOffset = 0f)
        {
            NavElement newSurface = new NavElement(points, position, rotation, extraOffset, this, NavElement.Type.Surface, cost);
            _Surfaces.Add(newSurface);
            newSurface.DirtyFlagEnabled += MarkSurfacesDirty;
            _AreSurfacesDirty           =  true;
            return newSurface;
        }

        public NavElement AddSurface(float radius, Vector2 position, float cost, float extraOffset = 0f)
        {
            Vector2[]  points     = GetHexagonPoints(radius);
            NavElement newSurface = new NavElement(points, position, 0f, extraOffset, this, NavElement.Type.Surface, cost);
            _Surfaces.Add(newSurface);
            newSurface.DirtyFlagEnabled += MarkSurfacesDirty;
            _AreSurfacesDirty           =  true;
            return newSurface;
        }

        public void RemoveNavElement(NavElement navElement)
        {
            if (navElement.NavType == NavElement.Type.Obstacle)
            {
                navElement.DirtyFlagEnabled -= MarkObstaclesDirty;
                _Obstacles.Remove(navElement);
            }
            else if (navElement.NavType == NavElement.Type.Surface)
            {
                navElement.DirtyFlagEnabled -= MarkSurfacesDirty;
                _Surfaces.Remove(navElement);
            }

            navElement.SetDirtyElementGroup();
            InnerUpdate(true);
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

        private Quad GetQuadTreeBounds(Vector2 leftBottom, Vector2 rightTop)
        {
            const float rootQuadTreeMultiplier = 2f;

            float x      = (leftBottom.x + rightTop.x) / 2f;
            float y      = (leftBottom.y + rightTop.y) / 2f;
            float width  = MathF.Abs(leftBottom.x - rightTop.x);
            float height = MathF.Abs(leftBottom.y - rightTop.y);

            width  *= rootQuadTreeMultiplier;
            height *= rootQuadTreeMultiplier;

            float halfWidth  = width / 2f;
            float halfHeight = height / 2f;

            return new Quad(x - halfWidth, y - halfHeight, x + halfWidth, y + halfHeight);
        }

        private void InnerUpdate(bool skipRefreshNavElement)
        {
            QuadTree.Clear();
            RefreshNavElements(skipRefreshNavElement);
            RebuildElementGroup();
            RebuildNavPoints();
        }

        private void RefreshNavElements(bool skipRefreshNavElement)
        {
            foreach (var obstacle in _Obstacles)
            {
                if (_AreObstaclesDirty && !skipRefreshNavElement)
                    obstacle.RefreshNavElement();

                QuadTree.Insert(obstacle, obstacle.Bounds);
            }

            _AreObstaclesDirty = false;

            foreach (var surface in _Surfaces)
            {
                if (_AreSurfacesDirty && !skipRefreshNavElement)
                    surface.RefreshNavElement();

                QuadTree.Insert(surface, surface.Bounds);
            }

            _AreSurfacesDirty = false;
        }

        private void RebuildElementGroup()
        {
            _TmpGroupedElements.Clear();

            foreach (var elementsGroup in elementsGroups)
                _TmpElementsGroups.Add(elementsGroup.Id, elementsGroup);

            elementsGroups.Clear();

            foreach (var obstacle in _Obstacles)
            {
                if (_TmpGroupedElements.Contains(obstacle)) continue;
                obstacle.RebuildTheElementGroup(_TmpGroupedElements, _TmpLocalGroupedElements, _TmpCollidedElements, _TmpSearchList, _TmpElementsGroups);
            }

            foreach (var surface in _Surfaces)
            {
                if (_TmpGroupedElements.Contains(surface)) continue;
                surface.RebuildTheElementGroup(_TmpGroupedElements, _TmpLocalGroupedElements, _TmpCollidedElements, _TmpSearchList, _TmpElementsGroups);
            }

            foreach (var unusedElementsGroup in _TmpElementsGroups.Values)
            {
                unusedElementsGroup.Clear();
                elementsGroupPull.Push(unusedElementsGroup);
            }

            _TmpElementsGroups.Clear();
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
            foreach (var element in elementsGroups)
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
                _NavPoints.Add(point);

                point.Neighbours.Add(previousPoint);
                previousPoint.Neighbours.Add(point);
                AddToConnectionDictionary(previousPoint, point);
                forbiddenConnection.Add(point);
                previousPoint = point;
            }

            _NavPoints.Add(lastPoint);
            previousPoint.Neighbours.Add(lastPoint);
            lastPoint.Neighbours.Add(previousPoint);
            AddToConnectionDictionary(previousPoint, lastPoint);
            forbiddenConnection.Add(lastPoint);
        }

        private void AddToConnectionDictionary(NavPoint first, NavPoint second)
        {
            if (first.Id < second.Id)
                _Connections.Add(new Tuple<NavPoint, NavPoint>(first, second), new ConnectionData());
            else
                _Connections.Add(new Tuple<NavPoint, NavPoint>(second, first), new ConnectionData());
        }

        #endregion Private Methods
    }
}