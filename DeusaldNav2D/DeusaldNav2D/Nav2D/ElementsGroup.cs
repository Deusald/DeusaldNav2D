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

namespace DeusaldNav2D
{
    internal class ElementsGroup
    {
        #region Variables

        private readonly HashSet<NavElement> _Obstacles;
        private readonly HashSet<NavElement> _Surfaces;
        private readonly Nav2D               _Nav2D;
        private readonly uint                _Id;

        #endregion Variables

        #region Properties

        public uint Id => _Id;

        #endregion Properties

        #region Init Methods

        internal ElementsGroup(Nav2D nav2D, uint id)
        {
            _Nav2D     = nav2D;
            _Id        = id;
            _Obstacles = new HashSet<NavElement>();
            _Surfaces  = new HashSet<NavElement>();
        }

        #endregion Init Methods

        #region Public Methods

        public void AddElement(NavElement element)
        {
            if (element.NavType == NavElement.Type.Obstacle)
                _Obstacles.Add(element);
            else if (element.NavType == NavElement.Type.Surface)
                _Surfaces.Add(element);
        }

        public void RemoveNavElement(NavElement element)
        {
            if (element.NavType == NavElement.Type.Obstacle)
                _Obstacles.Remove(element);
            else if (element.NavType == NavElement.Type.Surface)
                _Surfaces.Remove(element);

            if (_Obstacles.Count != 0 || _Surfaces.Count != 0) return;
            _Nav2D.elementsGroups.Remove(Id);
        }

        public void DismantleGroup()
        {
            List<NavElement> obstacles = new List<NavElement>(_Obstacles);
            List<NavElement> surfaces  = new List<NavElement>(_Surfaces);

            foreach (var obstacle in obstacles)
                obstacle.ElementGroupId = 0;

            foreach (var surface in surfaces)
                surface.ElementGroupId = 0;
        }

        #endregion Public Methods
    }
}