#include "SVONavigationData.h"

#include "SVONavDataRenderingComponent.h"
#include "SVONavigationSettings.h"
#include "SVONavigationSystem.h"

#include <libmorton/morton.h>

static const FIntVector NeighborDirections[ 6 ] = {
    { 1, 0, 0 },
    { -1, 0, 0 },
    { 0, 1, 0 },
    { 0, -1, 0 },
    { 0, 0, 1 },
    { 0, 0, -1 }
};

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

FVector FSVONavigationBoundsData::GetNodePosition( uint8 layer_index, MortonCode morton_code ) const
{
    const auto voxel_size = GetLayerVoxelSize( layer_index );
    uint_fast32_t x, y, z;
    morton3D_64_decode( morton_code, x, y, z );
    return Box.GetCenter() - Box.GetExtent() + FVector( x, y, z ) * voxel_size + FVector( voxel_size * 0.5f );
}

FVector FSVONavigationBoundsData::GetNodePositionFromLink( const FSVOOctreeLink & link ) const
{
    const auto & node = OctreeData.NodesByLayers[ link.LayerIndex ][ link.NodeIndex ];
    auto position = GetNodePosition( link.LayerIndex, node.MortonCode );

    if ( link.LayerIndex == 0 && node.FirstChild.IsValid() )
    {
        const float Size = GetLayerVoxelSize( 0 );
        uint_fast32_t X, Y, Z;
        morton3D_64_decode( link.SubNodeIndex, X, Y, Z );
        position += FVector( X * Size / 4, Y * Size / 4, Z * Size / 4 ) - FVector( Size * 0.375f );
    }

    return position;
}

void FSVONavigationBoundsData::ComputeDataFromNavigationBounds( const FSVONavigationBounds & navigation_bounds, const FSVODataConfig & config )
{
    Config = config.AsShared();

    VolumeBox = navigation_bounds.AreaBox;

    const auto * settings = GetDefault< USVONavigationSettings >();

    const auto box_max_size = VolumeBox.GetSize().GetAbsMax();
    VoxelExponent = FMath::RoundToInt( FMath::Log2( box_max_size / ( settings->VoxelSize * 4 ) ) );
    LayerCount = VoxelExponent + 1;

    const auto corrected_box_size = FMath::Pow( 2, VoxelExponent ) * ( settings->VoxelSize * 4 );
    const auto corrected_box_extent = corrected_box_size * 0.5f;

    Box = FBox::BuildAABB( VolumeBox.GetCenter(), FVector( corrected_box_extent ) );

    BlockedIndices.Reset();
    OctreeData.Reset();

    LayerNodeCount.Reset( LayerCount );
    LayerVoxelSizes.Reset( LayerCount );
    LayerVoxelHalfSizes.Reset( LayerCount );

    for ( LayerIndex layer_index = 0; layer_index < LayerCount; ++layer_index )
    {
        LayerNodeCount.Add( FMath::Pow( FMath::Pow( 2, ( VoxelExponent - layer_index ) ), 3 ) );

        const auto layer_voxel_size = Box.GetExtent().X / FMath::Pow( 2, VoxelExponent ) * FMath::Pow( 2.0f, layer_index + 1 );
        LayerVoxelSizes.Add( layer_voxel_size );
        LayerVoxelHalfSizes.Add( layer_voxel_size * 0.5f );
    }

    if ( LayerCount < 2 )
    {
        return;
    }

    FirstPassRasterization();

    AllocateLeafNodes();

    RasterizeInitialLayer();

    for ( LayerIndex layer_index = 1; layer_index < LayerCount; ++layer_index )
    {
        RasterizeLayer( layer_index );
    }

    for ( LayerIndex layer_index = LayerCount - 2; layer_index != static_cast< LayerIndex >( -1 ); --layer_index )
    {
        BuildNeighborLinks( layer_index );
    }
}

bool FSVONavigationBoundsData::IsPositionOccluded( const FVector & position, float box_size ) const
{
    auto shared_ptr_config = Config.Pin();

    return shared_ptr_config->World->OverlapBlockingTestByChannel(
        position,
        FQuat::Identity,
        shared_ptr_config->CollisionChannel,
        FCollisionShape::MakeBox( FVector( box_size + shared_ptr_config->Clearance ) ),
        shared_ptr_config->CollisionQueryParameters );
}

void FSVONavigationBoundsData::FirstPassRasterization()
{
    {
        auto & layer_blocked_indices = BlockedIndices.Emplace_GetRef();
        const auto node_count = GetLayerNodeCount( 1 );

        for ( MortonCode node_index = 0; node_index < node_count; ++node_index )
        {
            const auto position = GetNodePosition( 1, node_index );
            if ( IsPositionOccluded( position, GetLayerVoxelHalfSize( 1 ) ) )
            {
                layer_blocked_indices.Add( node_index );
            }
        }
    }

    {
        for ( int32 voxel_index = 0; voxel_index < VoxelExponent; voxel_index++ )
        {
            auto & layer_blocked_indices = BlockedIndices.Emplace_GetRef();
            for ( MortonCode morton_code : BlockedIndices[ voxel_index ] )
            {
                layer_blocked_indices.Add( morton_code >> 3 );
            }
        }
    }
}

void FSVONavigationBoundsData::AllocateLeafNodes()
{
    OctreeData.Leaves.AddDefaulted( BlockedIndices[ 0 ].Num() * 8 * 0.25f );
}

void FSVONavigationBoundsData::RasterizeLeaf( const FVector & node_position, int32 leaf_index )
{
    const auto layer_voxel_half_size = GetLayerVoxelHalfSize( 0 );
    const FVector Location = node_position - layer_voxel_half_size;
    const auto leaf_voxel_size = layer_voxel_half_size * 0.5f;
    const auto leaf_occlusion_voxel_size = leaf_voxel_size * 0.5f;

    for ( int32 subnode_index = 0; subnode_index < 64; subnode_index++ )
    {
        uint_fast32_t X, Y, Z;
        morton3D_64_decode( subnode_index, X, Y, Z );
        const FVector voxel_location = Location + FVector( X * leaf_voxel_size, Y * leaf_voxel_size, Z * leaf_voxel_size ) + leaf_voxel_size * 0.5f;

        if ( leaf_index >= OctreeData.Leaves.Num() - 1 )
        {
            OctreeData.Leaves.AddDefaulted( 1 );
        }

        if ( IsPositionOccluded( voxel_location, leaf_occlusion_voxel_size ) )
        {
            OctreeData.Leaves[ leaf_index ].SetSubNode( subnode_index );
        }
    }
}

void FSVONavigationBoundsData::RasterizeInitialLayer()
{
    auto & layer_nodes = OctreeData.NodesByLayers.Emplace_GetRef();
    int32 leaf_index = 0;

    OctreeData.Leaves.Reserve( BlockedIndices[ 0 ].Num() * 8 );
    layer_nodes.Reserve( BlockedIndices[ 0 ].Num() * 8 );

    const auto layer_node_count = GetLayerNodeCount( 0 );

    for ( uint32 node_index = 0; node_index < layer_node_count; node_index++ )
    {
        // If we know this node needs to be added, from the low res first pass
        if ( !BlockedIndices[ 0 ].Contains( node_index >> 3 ) )
        {
            continue;
        }

        auto & octree_node = layer_nodes.Emplace_GetRef();
        octree_node.MortonCode = node_index;

        const auto node_position = GetNodePosition( 0, node_index );

        // Now check if we have any blocking, and search leaf nodes
        if ( IsPositionOccluded( node_position, GetLayerVoxelHalfSize( 0 ) ) )
        {
            RasterizeLeaf( node_position, leaf_index );
            octree_node.FirstChild.LayerIndex = 0;
            octree_node.FirstChild.NodeIndex = leaf_index;
            octree_node.FirstChild.SubNodeIndex = 0;
            leaf_index++;
        }
        else
        {
            OctreeData.Leaves.AddDefaulted( 1 );
            leaf_index++;
            octree_node.FirstChild.Invalidate();
        }
    }
}

void FSVONavigationBoundsData::RasterizeLayer( const LayerIndex layer_index )
{
    auto & layer_nodes = OctreeData.NodesByLayers.Emplace_GetRef();

    checkf( layer_index > 0 && layer_index < LayerCount, TEXT( "layer_index is out of bounds" ) );

    layer_nodes.Reserve( BlockedIndices[ layer_index ].Num() * 8 );

    const auto node_count = GetLayerNodeCount( layer_index );

    for ( uint32 node_index = 0; node_index < node_count; node_index++ )
    {
        if ( !BlockedIndices[ layer_index ].Contains( node_index >> 3 ) )
        {
            continue;
        }

        const auto new_node_index = layer_nodes.Emplace();

        auto & new_octree_node = layer_nodes[ new_node_index ];
        new_octree_node.MortonCode = node_index;

        const auto child_index_from_code = GetNodeIndexFromMortonCode( layer_index - 1, new_octree_node.MortonCode << 3 );
        if ( child_index_from_code.IsSet() )
        {
            // Set parent->child links
            new_octree_node.FirstChild.LayerIndex = layer_index - 1;
            new_octree_node.FirstChild.NodeIndex = child_index_from_code.GetValue();

            // Set child->parent links
            for ( auto child_index = 0; child_index < 8; ++child_index )
            {
                auto & child_node = GetLayerNodes( new_octree_node.FirstChild.LayerIndex )[ new_octree_node.FirstChild.NodeIndex + child_index ];
                child_node.Parent.LayerIndex = layer_index;
                child_node.Parent.NodeIndex = new_node_index;
            }
        }
        else
        {
            new_octree_node.FirstChild.Invalidate();
        }
    }
}

TOptional< NodeIndex > FSVONavigationBoundsData::GetNodeIndexFromMortonCode( const LayerIndex layer_index, const MortonCode morton_code ) const
{
    const auto & layer_nodes = GetLayerNodes( layer_index );
    auto start = 0;
    auto end = layer_nodes.Num() - 1;
    auto mean = ( start + end ) * 0.5f;

    // Binary search by Morton code
    while ( start <= end )
    {
        if ( layer_nodes[ mean ].MortonCode < morton_code )
        {
            start = mean + 1;
        }
        else if ( layer_nodes[ mean ].MortonCode == morton_code )
        {
            return mean;
        }
        else
        {
            end = mean - 1;
        }

        mean = ( start + end ) * 0.5f;
    }

    return TOptional< NodeIndex >();
}

void FSVONavigationBoundsData::BuildNeighborLinks( LayerIndex layer_index )
{
    auto & layer_nodes = GetLayerNodes( layer_index );

    for ( NodeIndex layer_node_index = 0; layer_node_index < layer_nodes.Num(); layer_node_index++ )
    {
        auto & node = layer_nodes[ layer_node_index ];
        FVector node_position = GetNodePosition( layer_index, node.MortonCode );

        for ( NeighborDirection direction = 0; direction < 6; direction++ )
        {
            NodeIndex node_index = layer_node_index;

            FSVOOctreeLink & link = node.Neighbors[ direction ];

            LayerIndex current_layer = layer_index;

            while ( !FindNeighborInDirection( link, current_layer, node_index, direction, node_position ) && current_layer < LayerCount - 2 )
            {
                auto & parent_node = GetLayerNodes( current_layer )[ node_index ].Parent;
                if ( parent_node.IsValid() )
                {
                    node_index = parent_node.NodeIndex;
                    current_layer = parent_node.LayerIndex;
                }
                else
                {
                    current_layer++;
                    node_index = GetNodeIndexFromMortonCode( current_layer, node.MortonCode >> 3 ).Get( 0 );
                }
            }
        }
    }
}

bool FSVONavigationBoundsData::FindNeighborInDirection( FSVOOctreeLink & link, const LayerIndex layer_index, const NodeIndex node_index, const NeighborDirection direction, const FVector & node_position )
{
    const auto max_coordinates = GetLayerMaxNodeCount( layer_index );
    const auto & layer_nodes = GetLayerNodes( layer_index );
    const auto & target_node = layer_nodes[ node_index ];

    uint_fast32_t x, y, z;
    morton3D_64_decode( target_node.MortonCode, x, y, z );

    FIntVector neighbor_coords( static_cast< int32 >( x ), static_cast< int32 >( y ), static_cast< int32 >( z ) );
    neighbor_coords += NeighborDirections[ direction ];

    if ( neighbor_coords.X < 0 || neighbor_coords.X >= max_coordinates ||
         neighbor_coords.Y < 0 || neighbor_coords.Y >= max_coordinates ||
         neighbor_coords.Z < 0 || neighbor_coords.Z >= max_coordinates )
    {
        link.Invalidate();
        return true;
    }

    x = neighbor_coords.X;
    y = neighbor_coords.Y;
    z = neighbor_coords.Z;

    const MortonCode neighbor_code = morton3D_64_encode( x, y, z );

    int32 stop_index = layer_nodes.Num();
    int32 increment = 1;

    if ( neighbor_code < target_node.MortonCode )
    {
        increment = -1;
        stop_index = -1;
    }

    for ( int32 neighbor_node_index = node_index + increment; neighbor_node_index != stop_index; neighbor_node_index += increment )
    {
        auto & node = layer_nodes[ neighbor_node_index ];

        if ( node.MortonCode == neighbor_code )
        {
            if ( layer_index == 0 &&
                 node.HasChildren() &&
                 OctreeData.Leaves[ node.FirstChild.NodeIndex ].IsOccluded() )
            {

                link.Invalidate();
                return true;
            }

            link.LayerIndex = layer_index;

            if ( neighbor_node_index >= layer_nodes.Num() || neighbor_node_index < 0 )
            {
                break;
            }

            link.NodeIndex = neighbor_node_index;

            /*FVector AdjacentLocation;
            GetNodeLocation( LayerIndex, AdjacentCode, AdjacentLocation );*/

            return true;
        }

        // If we've passed the code we're looking for, it's not on this layer
        if ( increment == -1 && node.MortonCode < neighbor_code || increment == 1 && node.MortonCode > neighbor_code )
        {
            return false;
        }
    }
    return false;
}

ASVONavigationData::ASVONavigationData()
{
    PrimaryActorTick.bCanEverTick = false;

    RenderingComponent = CreateDefaultSubobject< USVONavDataRenderingComponent >( TEXT( "RenderingComponent" ) );
    RootComponent = RenderingComponent;

    Config = MakeShared< FSVODataConfig >();

    Config->CollisionQueryParameters.bFindInitialOverlaps = true;
    Config->CollisionQueryParameters.bTraceComplex = false;
    Config->CollisionQueryParameters.TraceTag = "SVONavigationRasterize";
}

void ASVONavigationData::PostRegisterAllComponents()
{
    Super::PostRegisterAllComponents();

    if ( auto * settings = GetDefault< USVONavigationSettings >() )
    {
        Config->Clearance = settings->Clearance;
        Config->CollisionChannel = settings->CollisionChannel;
        Config->World = GetWorld();
    }
}

void ASVONavigationData::Serialize( FArchive & archive )
{
    Super::Serialize( archive );
    archive << NavigationBoundsData;
    archive << DebugInfos;
}

void ASVONavigationData::AddNavigationBounds( const FSVONavigationBounds & navigation_bounds )
{
    const auto bounds_box = navigation_bounds.AreaBox;
    auto must_add_new_entry = true;

    // If we already have a volume with the same box, update the key. Not ideal but we can't use the unique id of the actor as it's new each time we load the level
    for ( auto & key_pair : NavigationBoundsData )
    {
        if ( key_pair.Value.GetVolumeBox() == bounds_box )
        {
            key_pair.Key = navigation_bounds.UniqueID;
            must_add_new_entry = false;
            break;
        }
    }

    if ( must_add_new_entry )
    {
        auto & data = NavigationBoundsData.Emplace( navigation_bounds.UniqueID );
        data.ComputeDataFromNavigationBounds( navigation_bounds, *Config );
    }

    MarkComponentsRenderStateDirty();
}

void ASVONavigationData::UpdateNavigationBounds( const FSVONavigationBounds & navigation_bounds )
{
    if ( auto * data = NavigationBoundsData.Find( navigation_bounds.UniqueID ) )
    {
        data->ComputeDataFromNavigationBounds( navigation_bounds, *Config );
        MarkComponentsRenderStateDirty();
    }
}

void ASVONavigationData::RemoveNavigationBounds( const FSVONavigationBounds & navigation_bounds )
{
    NavigationBoundsData.Remove( navigation_bounds.UniqueID );
    MarkComponentsRenderStateDirty();
}

FSVOPathFindingResult ASVONavigationData::FindPath( const FSVOPathFindingQuery & path_finding_query ) const
{
    return FSVOPathFindingResult( ENavigationQueryResult::Error );
}
