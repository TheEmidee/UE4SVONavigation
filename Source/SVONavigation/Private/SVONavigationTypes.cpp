#include "SVONavigationTypes.h"


#include "SVOPathCostCalculator.h"
#include "SVOPathHeuristicCalculator.h"


#include <libmorton/morton.h>

bool FSVOOctreeLeaf::GetSubNodeAt( uint_fast32_t X, uint_fast32_t Y, uint_fast32_t Z ) const
{
    const uint_fast64_t MortonCode = 0;
    morton3D_64_decode( MortonCode, X, Y, Z );
    return ( SubNodes & 1ULL << morton3D_64_encode( X, Y, Z ) ) != 0;
}

void FSVOOctreeLeaf::SetSubNodeAt( uint_fast32_t X, uint_fast32_t Y, uint_fast32_t Z )
{
    const uint_fast64_t MortonCode = 0;
    morton3D_64_decode( MortonCode, X, Y, Z );
    SubNodes |= 1ULL << morton3D_64_encode( X, Y, Z );
}

void FSVOOctreeData::Reset()
{
    NodesByLayers.Reset();
    Leaves.Reset();
}

FSVONavigationQueryFilterSettings::FSVONavigationQueryFilterSettings()
{
    PathCostCalculator = USVOPathCostCalculator_Distance::StaticClass();
    PathHeuristicCalculator = USVOPathHeuristicCalculator_Manhattan::StaticClass();
    HeuristicScale = 1.0f;
}
