using System;
using UnityEngine;

namespace mehmetsrl.Extensions.VerletSystem
{

    [Serializable]
    public class RopeColorSection 
    {
        public float start;   // 0 ile 1 arasında (başlangıç yüzdesi)
        public float end;     // 0 ile 1 arasında (bitiş yüzdesi)
        public Color color;

        // Constructor (Kod içinden new'lemek isterseniz diye)
        public RopeColorSection(float start, float end, Color color)
        {
            this.start = start;
            this.end = end;
            this.color = color;
        }
    }
}