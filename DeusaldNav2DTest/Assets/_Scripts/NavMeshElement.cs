using System;
using DeusaldNav2D;
using NaughtyAttributes;
using UnityEngine;

namespace DeusaldNav2DTest
{
    public class NavMeshElement : MonoBehaviour
    {
        #region Variables

        #pragma warning disable 0649

        [SerializeField] private float   _ExtraOffset;
        [SerializeField] private Vector2 _NewPosition;

        #pragma warning restore 0649

        private NavElement _NavElement;
        private float      _PreviousExtraOffset;
        private Vector2    _PreviousPosition;
        private float      _PreviousRotation;
        private Transform  _Transform;
        private MainNav2D  _MainNav;
        private Vector2[]  _Points;

        #endregion Variables

        #region Init Methods

        public void Init(NavElement navElement, MainNav2D mainNav2D)
        {
            _MainNav                             =  mainNav2D;
            _NavElement                          =  navElement;
            _PreviousExtraOffset                 =  0f;
            _PreviousPosition                    =  Vector2.zero;
            _PreviousRotation                    =  0f;
            navElement.NavElementPointsRefreshed += RefreshPoints;
            RefreshPoints(this, EventArgs.Empty);
        }

        #endregion Init Methods

        #region Special Methods

        private void Awake()
        {
            _Transform = transform;
        }

        private void Update()
        {
            if (_NavElement == null) return;

            if (!DeusaldSharp.MathUtils.AreFloatsEquals(_PreviousExtraOffset, _ExtraOffset))
            {
                _NavElement.ExtraOffset = _ExtraOffset;
                _PreviousExtraOffset    = _ExtraOffset;
            }

            float rotation = transform.rotation.eulerAngles.z * Mathf.Deg2Rad;

            if (!DeusaldSharp.MathUtils.AreFloatsEquals(_PreviousRotation, rotation))
            {
                _NavElement.Rotation = rotation;
                _PreviousRotation    = rotation;
            }

            Vector2 position = _Transform.position;

            if (_PreviousPosition != position)
            {
                _NavElement.Position = MainNav2D.ToVec2(position);
                _PreviousPosition    = position;
            }
        }

        private void OnDrawGizmos()
        {
            if (!_MainNav._ShowNavElements) return;
            if (_Points == null) return;

            Gizmos.color = _NavElement.NavType == NavElement.Type.Surface ? Color.green : Color.red;

            for (int i = 0; i < _Points.Length; ++i)
            {
                Vector2 begin = new Vector2(_Points[i].x, _Points[i].y);
                Vector2 end   = new Vector2(_Points[(i + 1) % _Points.Length].x, _Points[(i + 1) % _Points.Length].y);
                Gizmos.DrawLine(begin, end);
            }

            Gizmos.color = Color.white;
        }

        #endregion Special Methods

        #region Private Methods

        private void RefreshPoints(object sender, EventArgs eventArgs)
        {
            DeusaldSharp.Vector2[] points = _NavElement.NavElementPoints;
            _Points = new Vector2[points.Length];

            for (int i = 0; i < _Points.Length; ++i)
                _Points[i] = MainNav2D.ToVec2(points[i]);
        }

        [Button]
        private void Remove()
        {
            _NavElement.NavElementPointsRefreshed -= RefreshPoints;
            _MainNav.Nav2D.RemoveNavElement(_NavElement);
            Destroy(gameObject);
        }

        [Button]
        private void ApplyNewPosition()
        {
            _Transform.position = _NewPosition;
        }

        #endregion Private Methods
    }
}