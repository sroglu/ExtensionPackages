using Unity.Mathematics;

namespace mehmetsrl.Physics.PPF.Core
{
    // Lightweight spatial hashing utility (re-usable). Deterministic integer hash.
    public readonly struct SpatialGrid
    {
        public readonly float CellSize;
        public readonly float CellReciprocal;

        public SpatialGrid(float cellSize)
        {
            CellSize = cellSize;
            CellReciprocal = cellSize > PPFConstants.Epsilon ? 1.0f / cellSize : 0.0f;
        }

        public int3 GetCellCoordinate(float3 position)
        {
            return (int3)math.floor(position * CellReciprocal);
        }

        public int GetHash(int3 cell)
        {
            const int p1 = 73856093;
            const int p2 = 19349663;
            const int p3 = 83492791;
            return (cell.x * p1) ^ (cell.y * p2) ^ (cell.z * p3);
        }

        public int GetHashFromPosition(float3 position)
        {
            return GetHash(GetCellCoordinate(position));
        }
    }
}