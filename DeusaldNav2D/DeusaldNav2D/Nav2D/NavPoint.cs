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
    public class NavPoint
    {
        #region Properties

        #if DEBUG
        
        public List<NavPoint> DebugNeighbours => Neighbours;
        
        #endif
        
        internal uint              Id                   { get; }
        public   Vector2           Position             { get; }
        internal HashSet<NavPoint> ForbiddenConnections { get; }
        internal List<NavPoint>    Neighbours           { get; }

        #endregion Properties

        #region Init Methods

        internal NavPoint(uint id, Vector2 position, HashSet<NavPoint> forbiddenConnections)
        {
            Id                   = id;
            Position             = position;
            ForbiddenConnections = forbiddenConnections;
            Neighbours           = new List<NavPoint>();
        }

        #endregion Init Methods
    }
}