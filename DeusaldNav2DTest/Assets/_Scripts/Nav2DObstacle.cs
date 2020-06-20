using DeusaldNav2D;
using UnityEngine;

namespace DeusaldNav2DTest
{
    public class Nav2DObstacle : MonoBehaviour
    {
        #region Variables

        [SerializeField] private MainNav2D _MainNav2D;
        [SerializeField] private Vector2[] _Points;
        [SerializeField] private float     _Radius;
        [SerializeField] private float     _ExtraOffset;
        [SerializeField] private Vector2   _Position;
        [SerializeField] private float     _Rotation;

        private Obstacle _Obstacle;
        private float    _PreviousExtraOffset;
        private Vector2  _PreviousPosition;
        private float    _PreviousRotation;

        #endregion Variables

        #region Special Methods

        private void Start()
        {
            DeusaldSharp.Vector2[] points = new DeusaldSharp.Vector2[_Points.Length];

            for (int i = 0; i < points.Length; ++i)
                points[i] = new DeusaldSharp.Vector2(_Points[i].x, _Points[i].y);

            if (_Points.Length == 0)
                _Obstacle = _MainNav2D.Nav2D.AddObstacle(_Radius, new DeusaldSharp.Vector2(_Position.x, _Position.y));
            else
                _Obstacle = _MainNav2D.Nav2D.AddObstacle(points, new DeusaldSharp.Vector2(_Position.x, _Position.y),
                    _Rotation, _ExtraOffset);

            _PreviousExtraOffset = _ExtraOffset;
            _PreviousPosition    = _Position;
            _PreviousRotation    = _Rotation;
        }

        private void Update()
        {
            if (!DeusaldSharp.MathUtils.AreFloatsEquals(_PreviousExtraOffset, _ExtraOffset))
            {
                _Obstacle.ExtraOffset = _ExtraOffset;
                _PreviousExtraOffset  = _ExtraOffset;
            }

            if (!DeusaldSharp.MathUtils.AreFloatsEquals(_PreviousRotation, _Rotation))
            {
                _Obstacle.Rotation = _Rotation;
                _PreviousRotation  = _Rotation;
            }

            if (_PreviousPosition != _Position)
            {
                _Obstacle.Position = new DeusaldSharp.Vector2(_Position.x, _Position.y);
                _PreviousPosition  = _Position;
            }
        }

        private void OnDrawGizmos()
        {
            if (_Obstacle == null) return;

            DeusaldSharp.Vector2[] points = _Obstacle.ObstaclePoints;

            for (int i = 0; i < points.Length; ++i)
            {
                Gizmos.color = Color.green;
                Vector2 begin = new Vector2(points[i].x, points[i].y);
                Vector2 end   = new Vector2(points[(i + 1) % points.Length].x, points[(i + 1) % points.Length].y);
                Gizmos.DrawLine(begin, end);
                Gizmos.color = Color.white;
            }
        }

        #endregion Special Methods
    }
}