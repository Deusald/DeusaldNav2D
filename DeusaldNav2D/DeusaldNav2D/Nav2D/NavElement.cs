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
    public class NavElement
    {
        #region Types

        public enum Type
        {
            Obstacle,
            Surface
        }

        internal readonly struct CostChangedData
        {
            public readonly float previousCost;
            public readonly float newCost;

            public CostChangedData(float previousCost, float newCost)
            {
                this.previousCost = previousCost;
                this.newCost      = newCost;
            }
        }

        #endregion Types

        #region Variables

        public event EventHandler NavElementPointsRefreshed;

        internal event EventHandler                  DirtyFlagEnabled;
        internal event EventHandler<CostChangedData> CostChanged;

        internal Vector2[] navElementPoints;
        internal uint      elementsGroupId;

        internal readonly List<IntPoint> intNavElementPoints;

        private Vector2        _Position;
        private float          _Rotation;
        private float          _ExtraOffset;
        private bool           _IsDirty;
        private bool           _IsExtendDirty;
        private Vector2[]      _ExtendedPoints;
        private List<IntPoint> _EnterExtendPoints;
        private float          _Cost;
        private Quad           _Bounds;

        private readonly Vector2[] _OriginalPoints;
        private readonly Nav2D     _Nav2D;

        #endregion Variables

        #region Properties

        public bool IsDirty
        {
            get => _IsDirty;

            private set
            {
                _IsDirty = value;

                if (value)
                    DirtyFlagEnabled?.Invoke(this, EventArgs.Empty);
            }
        }

        public Vector2[] NavElementPoints => (Vector2[]) navElementPoints.Clone();

        public Vector2 Position
        {
            get => _Position;
            set
            {
                _Position = value;
                IsDirty   = true;
            }
        }

        public float Rotation
        {
            get => _Rotation;
            set
            {
                _Rotation = value;
                IsDirty   = true;
            }
        }

        public float ExtraOffset
        {
            get => _ExtraOffset;
            set
            {
                _ExtraOffset   = value;
                _IsExtendDirty = true;
                IsDirty        = true;
            }
        }

        public float Cost
        {
            get => _Cost;
            set
            {
                CostChanged?.Invoke(this, new CostChangedData(_Cost, value));
                _Cost = value;
            }
        }

        public Vector2 BottomBoundingBox { get; }
        public Vector2 TopBoundingBox    { get; }
        public Type    NavType           { get; }
        public Quad    Bounds            => _Bounds;

        #endregion Properties

        #region Init Methods

        internal NavElement(Vector2[] originalPoints, Vector2 position, float rotation, float extraOffset, Nav2D nav2D, Type type, float cost)
        {
            _OriginalPoints     = originalPoints;
            _ExtendedPoints     = new Vector2[originalPoints.Length];
            navElementPoints    = new Vector2[originalPoints.Length];
            _EnterExtendPoints  = new List<IntPoint>(_OriginalPoints.Length);
            intNavElementPoints = new List<IntPoint>();
            _Bounds             = new Quad();
            _Nav2D              = nav2D;
            _Position           = position;
            _Rotation           = rotation;
            _ExtraOffset        = extraOffset;
            _Cost               = cost;
            NavType             = type;
            _IsDirty            = true;
            _IsExtendDirty      = true;
            BottomBoundingBox   = Vector2.Zero;
            TopBoundingBox      = Vector2.Zero;
            elementsGroupId     = 0;

            if (originalPoints.Length < 3)
                throw new Exception("Can't create polygon shape with less than 3 vertex!");

            CheckIfCounterClockWise();
            CheckIfConvex();
            TransformExtendPoints();
            RefreshNavElement();
        }

        #endregion Init Methods

        #region Public Methods

        internal void RefreshNavElement()
        {
            if (!_IsDirty) return;

            if (_IsExtendDirty)
                RebuildExtendPoints();

            RebuildNavElementPoints();
        }

        internal void RebuildTheElementGroup(HashSet<NavElement> groupedElements, List<NavElement> localGroupedElements, Queue<NavElement> collidedElements,
            List<NavElement> searchList, Dictionary<uint, ElementsGroup> oldElementsGroup)
        {
            localGroupedElements.Clear();
            collidedElements.Clear();

            groupedElements.Add(this);
            localGroupedElements.Add(this);
            collidedElements.Enqueue(this);

            while (collidedElements.Count != 0)
            {
                NavElement element = collidedElements.Dequeue();
                _Nav2D.QuadTree.SearchArea(element.Bounds, ref searchList);

                foreach (var otherElement in searchList)
                {
                    if (groupedElements.Contains(otherElement)) continue;

                    localGroupedElements.Add(otherElement);
                    groupedElements.Add(otherElement);
                    collidedElements.Enqueue(otherElement);
                }
            }

            if (elementsGroupId == 0 || oldElementsGroup[elementsGroupId].isDirty || oldElementsGroup[elementsGroupId].NumberOfElements != localGroupedElements.Count)
            {
                ElementsGroup newGroup = _Nav2D.elementsGroupPull.Count == 0 ? new ElementsGroup(_Nav2D) : _Nav2D.elementsGroupPull.Pop();
                newGroup.AddElements(localGroupedElements);
                _Nav2D.elementsGroups.Add(newGroup);
                newGroup.Build();
            }
            else
            {
                _Nav2D.elementsGroups.Add(oldElementsGroup[elementsGroupId]);
                oldElementsGroup.Remove(elementsGroupId);
            }
        }

        internal void SetDirtyElementGroup()
        {
            foreach (var elementsGroup in _Nav2D.elementsGroups)
            {
                if (elementsGroup.Id != elementsGroupId) continue;
                elementsGroup.isDirty = true;
                return;
            }
        }

        #endregion Public Methods

        #region Private Methods

        private void TransformExtendPoints()
        {
            foreach (var point in _OriginalPoints)
                _EnterExtendPoints.Add(_Nav2D.ParseToIntPoint(point));
        }

        private void CheckIfCounterClockWise()
        {
            Vector2 vector1 = _OriginalPoints[1] - _OriginalPoints[0];
            Vector2 vector2 = _OriginalPoints[2] - _OriginalPoints[1];

            float cross = Vector2.Cross(vector1, vector2);

            if (MathUtils.AreFloatsEquals(cross, 0f))
                throw new Exception("Polygon is wrongly build!");

            if (cross < 0)
                throw new Exception("The polygon vertex are in clockwise order! Change to counter-clockwise order!");
        }

        private void CheckIfConvex()
        {
            // For each set of three adjacent points A, B, C,
            // find the cross product AB · BC. If the sign of
            // all the cross products is the same, the angles
            // are all positive or negative (depending on the
            // order in which we visit them) so the polygon
            // is convex.
            bool gotNegative = false;
            bool gotPositive = false;
            for (int a = 0; a < _OriginalPoints.Length; a++)
            {
                int b = (a + 1) % _OriginalPoints.Length;
                int c = (b + 1) % _OriginalPoints.Length;

                float crossProduct = CrossProductLength(_OriginalPoints[a].x, _OriginalPoints[a].y,
                    _OriginalPoints[b].x, _OriginalPoints[b].y, _OriginalPoints[c].x,
                    _OriginalPoints[c].y);

                if (crossProduct < 0)
                    gotNegative = true;
                else if (crossProduct > 0)
                    gotPositive = true;

                if (gotNegative && gotPositive)
                    throw new Exception("The polygon is not convex!");
            }
        }

        private float CrossProductLength(float ax, float ay, float bx, float by, float cx, float cy)
        {
            float bAx = ax - bx;
            float bAy = ay - by;
            float bCx = cx - bx;
            float bCy = cy - by;
            return bAx * bCy - bAy * bCx;
        }

        private void RebuildExtendPoints()
        {
            _Nav2D.ExtendThePolygon(ref _EnterExtendPoints, ref _ExtendedPoints, _ExtraOffset);
            _IsExtendDirty = false;
        }

        private void RebuildNavElementPoints()
        {
            float minX = float.MaxValue;
            float maxX = float.MinValue;
            float minY = float.MaxValue;
            float maxY = float.MinValue;

            if (navElementPoints.Length != _ExtendedPoints.Length)
                navElementPoints = new Vector2[_ExtendedPoints.Length];

            for (int i = 0; i < _ExtendedPoints.Length; ++i)
            {
                Vector2 rotated    = Vector2.Rotate(_Rotation, _ExtendedPoints[i]);
                Vector2 translated = _Position + rotated;
                navElementPoints[i] = translated;
                minX                = MathF.Min(minX, translated.x);
                maxX                = MathF.Max(maxX, translated.x);
                minY                = MathF.Min(minY, translated.y);
                maxY                = MathF.Max(maxY, translated.y);
            }

            BottomBoundingBox.Set(minX, minY);
            TopBoundingBox.Set(maxX, maxY);
            _Bounds.Set(BottomBoundingBox, TopBoundingBox);

            intNavElementPoints.Clear();

            foreach (var point in navElementPoints)
                intNavElementPoints.Add(_Nav2D.ParseToIntPoint(point));

            _IsDirty = false;
            SetDirtyElementGroup();
            NavElementPointsRefreshed?.Invoke(this, EventArgs.Empty);
        }

        #endregion Private Methods
    }
}