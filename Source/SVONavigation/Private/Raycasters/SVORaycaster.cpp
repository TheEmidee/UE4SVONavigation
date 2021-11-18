#include "Raycasters/SVORayCaster.h"

#include "SVONavigationData.h"
#include "SVOVolumeNavigationData.h"

#include <NavigationSystem.h>

FSVORayCasterObserver_GenerateDebugInfos::FSVORayCasterObserver_GenerateDebugInfos( FSVORayCasterDebugInfos & debug_infos ) :
    DebugInfos( debug_infos )
{
}

void FSVORayCasterObserver_GenerateDebugInfos::Initialize( const FSVOVolumeNavigationData * navigation_data, const FVector from, const FVector to )
{
    DebugInfos.TraversedNodes.Reset();
    DebugInfos.TraversedLeafNodes.Reset();
    DebugInfos.TraversedLeafSubNodes.Reset();
    DebugInfos.RayCastStartLocation = from;
    DebugInfos.RayCastEndLocation = to;
    DebugInfos.NavigationData = navigation_data;
}

void FSVORayCasterObserver_GenerateDebugInfos::AddTraversedNode( FSVONodeAddress node_address, bool is_occluded )
{
    DebugInfos.TraversedNodes.Emplace( node_address, is_occluded );
}

void FSVORayCasterObserver_GenerateDebugInfos::AddTraversedLeafNode( FSVONodeAddress node_address, bool is_occluded )
{
    DebugInfos.TraversedLeafNodes.Emplace( node_address, is_occluded );
}

void FSVORayCasterObserver_GenerateDebugInfos::AddTraversedLeafSubNode( FSVONodeAddress node_address, bool is_occluded )
{
    DebugInfos.TraversedLeafSubNodes.Emplace( node_address, is_occluded );
}

bool USVORayCaster::HasLineOfSight( UObject * world_context, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    if ( UNavigationSystemV1 * navigation_system = UNavigationSystemV1::GetCurrent( world_context->GetWorld() ) )
    {
        if ( auto * navigation_data = navigation_system->GetNavDataForProps( nav_agent_properties ) )
        {
            if ( const auto * svo_navigation_data = Cast< ASVONavigationData >( navigation_data ) )
            {
                if ( const auto * volume_navigation_data = svo_navigation_data->GetVolumeNavigationDataContainingPoints( { from, to } ) )
                {
                    return HasLineOfSight( world_context, *volume_navigation_data, from, to, nav_agent_properties );
                }
            }
        }
    }

    return false;
}

bool USVORayCaster::HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FSVONodeAddress from, const FSVONodeAddress to, const FNavAgentProperties & nav_agent_properties ) const
{
    const auto from_position = volume_navigation_data.GetNodePositionFromAddress( from );
    const auto to_position = volume_navigation_data.GetNodePositionFromAddress( to );

    return HasLineOfSight( world_context, volume_navigation_data, from_position, to_position, nav_agent_properties );
}

bool USVORayCaster::HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    if ( Observer.IsValid() )
    {
        Observer->Initialize( &volume_navigation_data, from, to );
    }
    return HasLineOfSightInternal( world_context, volume_navigation_data, from, to, nav_agent_properties );
}

void USVORayCaster::SetObserver( const TSharedPtr< FSVORayCasterObserver > observer )
{
    Observer = observer;
}

bool USVORayCaster::HasLineOfSightInternal( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    return false;
}
