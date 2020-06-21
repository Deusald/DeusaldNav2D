using System.Collections.Generic;
using DeusaldNav2D;
using NaughtyAttributes;
using UnityEngine;

namespace DeusaldNav2DTest
{
    public class MainNav2D : MonoBehaviour
    {
        #region Variables

        [ShowIf(nameof(IsPlayModeOn)), BoxGroup(_VisibilityGroup)]
        public bool _ShowNavElements;

        #pragma warning disable 0649

        [SerializeField, HideIf(nameof(IsPlayModeOn))]
        private Nav2D.Accuracy _Accuracy;

        [SerializeField, HideIf(nameof(IsPlayModeOn))]
        private Vector2 _LeftBottomMapCorner;

        [SerializeField, HideIf(nameof(IsPlayModeOn))]
        private Vector2 _RightUpperMapCorner;

        [SerializeField, HideIf(nameof(IsPlayModeOn))]
        private float _AgentRadius;

        [SerializeField, HideIf(nameof(IsPlayModeOn))]
        private GameObject _NavMeshElementObject;

        [SerializeField, ShowIf(nameof(IsPlayModeOn)), BoxGroup(_CreateNavElementGroup)]
        private bool _IsObstacle;

        [SerializeField, ShowIf(nameof(IsPlayModeOnAndNotObstacle)), BoxGroup(_CreateNavElementGroup)]
        private float _Cost;

        [SerializeField, ShowIf(nameof(IsPlayModeOn)), BoxGroup(_CreateNavElementGroup)]
        private float _Width;

        [SerializeField, ShowIf(nameof(IsPlayModeOn)), BoxGroup(_CreateNavElementGroup)]
        private float _Height;

        [SerializeField, ShowIf(nameof(IsPlayModeOn)), BoxGroup(_CreateNavElementGroup)]
        private float _Radius;

        [SerializeField, ShowIf(nameof(IsPlayModeOn)), BoxGroup(_CreateNavElementGroup)]
        private Vector2[] _Points;

        private const string _VisibilityGroup       = "Visibility";
        private const string _CreateNavElementGroup = "Create Nav Element";

        #pragma warning restore 0649

        #endregion Variables

        #region Properties

        public Nav2D Nav2D { get; private set; }

        #endregion Properties

        #region Special Methods

        private void Awake()
        {
            Nav2D = new Nav2D(ToVec2(_LeftBottomMapCorner), ToVec2(_RightUpperMapCorner), _AgentRadius, _Accuracy);
        }

        private void Update()
        {
            Nav2D.Update();
        }

        private void OnDrawGizmos()
        {
            // World Boundary
            Gizmos.color = Color.magenta;
            Vector2 leftUpperMapCorner   = new Vector2(_LeftBottomMapCorner.x, _RightUpperMapCorner.y);
            Vector2 rightBottomMapCorner = new Vector2(_RightUpperMapCorner.x, _LeftBottomMapCorner.y);
            Gizmos.DrawLine(leftUpperMapCorner, _RightUpperMapCorner);
            Gizmos.DrawLine(_RightUpperMapCorner, rightBottomMapCorner);
            Gizmos.DrawLine(rightBottomMapCorner, _LeftBottomMapCorner);
            Gizmos.DrawLine(_LeftBottomMapCorner, leftUpperMapCorner);
            Gizmos.color = Color.white;
            
            if (Nav2D == null) return;
            
            //Nav Points
            Gizmos.color = Color.cyan;
            
            foreach (var navPoint in Nav2D.DebugNavPoints)
                Gizmos.DrawSphere(ToVec2(navPoint.Position), 0.1f);

            Gizmos.color = Color.white;
        }

        #endregion Special Methods

        #region Public Methods

        public static DeusaldSharp.Vector2 ToVec2(Vector2 vector2)
        {
            return new DeusaldSharp.Vector2(vector2.x, vector2.y);
        }

        public static Vector2 ToVec2(DeusaldSharp.Vector2 vector2)
        {
            return new Vector2(vector2.x, vector2.y);
        }

        #endregion Public Methods

        #region Private Methods

        private bool IsPlayModeOn()
        {
            return Application.isPlaying;
        }

        private bool IsPlayModeOnAndNotObstacle()
        {
            return Application.isPlaying && !_IsObstacle;
        }

        [Button, ShowIf(nameof(IsPlayModeOn))]
        private void CreateBox()
        {
            List<DeusaldSharp.Vector2> points = new List<DeusaldSharp.Vector2>();

            float halfWidth  = _Width / 2f;
            float halfHeight = _Height / 2f;

            if (halfWidth < 0f || halfHeight < 0f)
            {
                Debug.LogError("Width and Height can't be less than 0!");
                return;
            }

            points.Add(new DeusaldSharp.Vector2(-halfWidth, halfHeight));
            points.Add(new DeusaldSharp.Vector2(-halfWidth, -halfHeight));
            points.Add(new DeusaldSharp.Vector2(halfWidth, -halfHeight));
            points.Add(new DeusaldSharp.Vector2(halfWidth, halfHeight));

            if (_IsObstacle)
            {
                NavElement element       = Nav2D.AddObstacle(points.ToArray(), DeusaldSharp.Vector2.Zero, 0f);
                GameObject elementObject = Instantiate(_NavMeshElementObject, Vector3.zero, Quaternion.identity);
                elementObject.GetComponent<NavMeshElement>().Init(element, this);
            }
            else
            {
                if (_Cost <= 0f)
                {
                    Debug.LogError("Cost can't be less than 0");
                    return;
                }

                NavElement element       = Nav2D.AddSurface(points.ToArray(), DeusaldSharp.Vector2.Zero, 0f, _Cost);
                GameObject elementObject = Instantiate(_NavMeshElementObject, Vector3.zero, Quaternion.identity);
                elementObject.GetComponent<NavMeshElement>().Init(element, this);
            }
        }

        [Button, ShowIf(nameof(IsPlayModeOn))]
        private void CreateCircle()
        {
            if (_Radius < 0f)
            {
                Debug.LogError("Radius can't be less than 0!");
                return;
            }

            if (_IsObstacle)
            {
                NavElement element       = Nav2D.AddObstacle(_Radius, DeusaldSharp.Vector2.Zero, 0f);
                GameObject elementObject = Instantiate(_NavMeshElementObject, Vector3.zero, Quaternion.identity);
                elementObject.GetComponent<NavMeshElement>().Init(element, this);
            }
            else
            {
                if (_Cost <= 0f)
                {
                    Debug.LogError("Cost can't be less than 0");
                    return;
                }

                NavElement element       = Nav2D.AddSurface(_Radius, DeusaldSharp.Vector2.Zero, 0f, _Cost);
                GameObject elementObject = Instantiate(_NavMeshElementObject, Vector3.zero, Quaternion.identity);
                elementObject.GetComponent<NavMeshElement>().Init(element, this);
            }
        }

        [Button, ShowIf(nameof(IsPlayModeOn))]
        private void CreatePolygon()
        {
            DeusaldSharp.Vector2[] points = new DeusaldSharp.Vector2[_Points.Length];

            for (int i = 0; i < points.Length; ++i)
                points[i] = ToVec2(_Points[i]);

            if (_IsObstacle)
            {
                NavElement element       = Nav2D.AddObstacle(points, DeusaldSharp.Vector2.Zero, 0f);
                GameObject elementObject = Instantiate(_NavMeshElementObject, Vector3.zero, Quaternion.identity);
                elementObject.GetComponent<NavMeshElement>().Init(element, this);
            }
            else
            {
                if (_Cost <= 0f)
                {
                    Debug.LogError("Cost can't be less than 0");
                    return;
                }

                NavElement element       = Nav2D.AddSurface(points, DeusaldSharp.Vector2.Zero, 0f, _Cost);
                GameObject elementObject = Instantiate(_NavMeshElementObject, Vector3.zero, Quaternion.identity);
                elementObject.GetComponent<NavMeshElement>().Init(element, this);
            }
        }

        #endregion Private Methods
    }
}