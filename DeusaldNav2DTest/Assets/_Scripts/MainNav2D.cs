using DeusaldNav2D;
using UnityEngine;

namespace DeusaldNav2DTest
{
    public class MainNav2D : MonoBehaviour
    {
        #region Properties

        public Nav2D Nav2D { get; private set; }

        #endregion Properties

        #region Special Methods

        private void Awake()
        {
            Nav2D = new Nav2D(new DeusaldSharp.Vector2(0f, 0f),
                new DeusaldSharp.Vector2(0f, 0f), 0f, Nav2D.Accuracy.TwoPoints);
        }

        private void Update()
        {
            Nav2D.Update();
        }

        #endregion Special Methods
    }
}