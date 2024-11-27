#include "Raycasters/SVORaycasterTest.h"

#include "Raycasters/SVORayCaster.h"
#include "Raycasters/SVORaycaster_OctreeTraversal.h"
#include "SVONavigationData.h"

#include <Components/SphereComponent.h>
#include <Debug/DebugDrawService.h>
#include <NavigationSystem.h>

void FSVORayCasterSceneProxyData::GatherData( const ASVORaycasterTest & ray_caster_test )
{
    DebugInfos = ray_caster_test.GetDebugInfos();
}

FSVORayCasterSceneProxy::FSVORayCasterSceneProxy( const UPrimitiveComponent & component, const FSVORayCasterSceneProxyData & proxy_data ) :
    FDebugRenderSceneProxy( &component )
{
    DrawType = EDrawType::SolidAndWireMeshes;
    RayCasterTest = Cast< ASVORaycasterTest >( component.GetOwner() );

    const auto & debug_draw_options = RayCasterTest->GetDebugDrawOptions();

    if ( proxy_data.DebugInfos.NavigationData == nullptr )
    {
        return;
    }

    if ( !debug_draw_options.bEnableDebugDraw )
    {
        return;
    }

    Lines.Emplace( proxy_data.DebugInfos.RayCastStartLocation, proxy_data.DebugInfos.RayCastEndLocation, proxy_data.DebugInfos.Result ? FColor::Red : FColor::Green, 5.0f );

    const auto layer_count = proxy_data.DebugInfos.NavigationData->GetData().GetLayerCount();
    const auto corrected_layer_index = FMath::Clamp( static_cast< int >( debug_draw_options.LayerIndexToDraw ), 0, layer_count - 1 );

    const auto draw_morton_coords = [ &debug_draw_options, this ]( const FVector & location, const FSVONodeAddress node_address ) {
        if ( debug_draw_options.bDrawMortonCode )
        {
            Texts.Emplace( FString::Printf( TEXT( "%i:%i:%i" ), node_address.LayerIndex, node_address.NodeIndex, node_address.SubNodeIndex ), location + FVector( 0.0f, 0.0f, 40.0f ), FLinearColor::Black );
        }
    };

    if ( debug_draw_options.bDrawLayerNodes )
    {
        for ( const auto & traversed_node : proxy_data.DebugInfos.TraversedNodes )
        {
            if ( traversed_node.NodeAddress.LayerIndex != corrected_layer_index )
            {
                continue;
            }

            const auto node_position = proxy_data.DebugInfos.NavigationData->GetNodePositionFromAddress( traversed_node.NodeAddress, false );
            const auto node_extent = proxy_data.DebugInfos.NavigationData->GetData().GetLayer( traversed_node.NodeAddress.LayerIndex ).GetNodeExtent();

            Boxes.Emplace( FBox::BuildAABB( node_position, FVector( node_extent ) ), traversed_node.bIsOccluded ? FColor::Orange : FColor::Green );
            draw_morton_coords( node_position, traversed_node.NodeAddress );
        }
    }

    if ( debug_draw_options.bDrawSubNodes )
    {
        for ( const auto & traversed_leaf_sub_node : proxy_data.DebugInfos.TraversedLeafSubNodes )
        {
            const auto node_position = proxy_data.DebugInfos.NavigationData->GetNodePositionFromAddress( traversed_leaf_sub_node.NodeAddress, true );
            const auto node_extent = proxy_data.DebugInfos.NavigationData->GetData().GetLeafNodes().GetLeafSubNodeExtent();

            Boxes.Emplace( FBox::BuildAABB( node_position, FVector( node_extent ) ), traversed_leaf_sub_node.bIsOccluded ? FColor::Orange : FColor::Green );
            draw_morton_coords( node_position, traversed_leaf_sub_node.NodeAddress );
        }
    }
}

SIZE_T FSVORayCasterSceneProxy::GetTypeHash() const
{
    static size_t UniquePointer;
    return reinterpret_cast< size_t >( &UniquePointer );
}

FPrimitiveViewRelevance FSVORayCasterSceneProxy::GetViewRelevance( const FSceneView * view ) const
{
    FPrimitiveViewRelevance Result;
    Result.bDrawRelevance = /*view->Family->EngineShowFlags.GetSingleFlag(ViewFlagIndex) &&*/ IsShown( view ) /* && ( !DebugDrawOptions.bDrawOnlyWhenSelected || SafeIsActorSelected() ) */;
    Result.bDynamicRelevance = true;
    // ideally the TranslucencyRelevance should be filled out by the material, here we do it conservative
    Result.bSeparateTranslucency = Result.bNormalTranslucency = IsShown( view );
    return Result;
}

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST

void FSVORayCasterDebugDrawDelegateHelper::InitDelegateHelper( const FSVORayCasterSceneProxy * scene_proxy )
{
    Super::InitDelegateHelper( scene_proxy );
}

void FSVORayCasterDebugDrawDelegateHelper::RegisterDebugDrawDelegateInternal()
{
    ensureMsgf( State != RegisteredState, TEXT( "RegisterDebugDrawDelgate is already Registered!" ) );
    if ( State == InitializedState )
    {
        DebugTextDrawingDelegate = FDebugDrawDelegate::CreateRaw( this, &FSVORayCasterDebugDrawDelegateHelper::DrawDebugLabels );
        DebugTextDrawingDelegateHandle = UDebugDrawService::Register( TEXT( "Navigation" ), DebugTextDrawingDelegate );
        State = RegisteredState;
    }
}

void FSVORayCasterDebugDrawDelegateHelper::UnregisterDebugDrawDelegate()
{
    ensureMsgf( State != InitializedState, TEXT( "UnegisterDebugDrawDelgate is in an invalid State: %i !" ), State );
    if ( State == RegisteredState )
    {
        check( DebugTextDrawingDelegate.IsBound() );
        UDebugDrawService::Unregister( DebugTextDrawingDelegateHandle );
        State = InitializedState;
    }
}
#endif

ASVORaycasterTest * USVORayCasterRenderingComponent::GetRayCasterTest() const
{
    return Cast< ASVORaycasterTest >( GetOwner() );
}

void USVORayCasterRenderingComponent::CreateRenderState_Concurrent( FRegisterComponentContext * context )
{
    Super::CreateRenderState_Concurrent( context );

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    DebugDrawDelegateManager.RequestRegisterDebugDrawDelegate( context );
#endif
}

void USVORayCasterRenderingComponent::DestroyRenderState_Concurrent()
{
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    DebugDrawDelegateManager.UnregisterDebugDrawDelegate();
#endif

    Super::DestroyRenderState_Concurrent();
}

FPrimitiveSceneProxy * USVORayCasterRenderingComponent::CreateSceneProxy()
{
    FSVORayCasterSceneProxyData proxy_data;
    proxy_data.GatherData( *GetRayCasterTest() );

    if ( FSVORayCasterSceneProxy * new_scene_proxy = new FSVORayCasterSceneProxy( *this, proxy_data ) )
    {
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
        DebugDrawDelegateManager.InitDelegateHelper( new_scene_proxy );
        DebugDrawDelegateManager.ReregisterDebugDrawDelegate();
#endif

        return new_scene_proxy;
    }

    return nullptr;
}

FBoxSphereBounds USVORayCasterRenderingComponent::CalcBounds( const FTransform & local_to_world ) const
{
    const FBoxSphereBounds result = Super::CalcBounds( local_to_world );

    if ( const auto * owner = GetRayCasterTest() )
    {
        return owner->GetBoundingBoxContainingOtherActorAndMe();
    }

    return result;
}

ASVORaycasterTest::ASVORaycasterTest()
{
    SphereComponent = CreateDefaultSubobject< USphereComponent >( TEXT( "SphereComponent" ) );
    SphereComponent->InitSphereRadius( 100.0f );
    RootComponent = SphereComponent;

#if WITH_EDITORONLY_DATA
    RenderingComponent = CreateEditorOnlyDefaultSubobject< USVORayCasterRenderingComponent >( TEXT( "RenderingComponent" ) );
    if ( RenderingComponent != nullptr )
    {
        RenderingComponent->SetCollisionEnabled( ECollisionEnabled::NoCollision );
    }
#endif

    PrimaryActorTick.bCanEverTick = false;

    Raycaster = NewObject< USVORayCaster_OctreeTraversal >();
    NavAgentProperties.PreferredNavData = ASVONavigationData::StaticClass();
    NavAgentProperties.AgentRadius = 100.0f;
    bUpdatePathAfterMoving = true;
}

FBoxSphereBounds ASVORaycasterTest::GetBoundingBoxContainingOtherActorAndMe() const
{
    TArray< FVector > points;
    points.Reserve( 2 );

    points.Add( GetActorLocation() );

    if ( OtherActor != nullptr )
    {
        points.Add( OtherActor->GetActorLocation() );
    }

    return FBoxSphereBounds( FBox( points ) );
}

#if WITH_EDITOR
void ASVORaycasterTest::PreEditChange( FProperty * property_about_to_change )
{
    static const FName NAME_OtherActor = GET_MEMBER_NAME_CHECKED( ASVORaycasterTest, OtherActor );

    if ( property_about_to_change != nullptr && property_about_to_change->GetFName() == NAME_OtherActor && OtherActor != nullptr && OtherActor->OtherActor == this )
    {
        OtherActor->OtherActor = nullptr;
#if WITH_EDITORONLY_DATA
        OtherActor->RenderingComponent->MarkRenderStateDirty();
        RenderingComponent->MarkRenderStateDirty();
#endif
    }

    Super::PreEditChange( property_about_to_change );
}

void ASVORaycasterTest::PostEditChangeProperty( FPropertyChangedEvent & property_changed_event )
{
    static const FName NAME_OtherActor = GET_MEMBER_NAME_CHECKED( ASVORaycasterTest, OtherActor );
    static const FName NAME_UpdatePathAfterMoving = GET_MEMBER_NAME_CHECKED( ASVORaycasterTest, bUpdatePathAfterMoving );

    if ( property_changed_event.Property != nullptr )
    {
        const FName property_name = property_changed_event.MemberProperty->GetFName();
        if ( property_name == NAME_OtherActor )
        {
            if ( OtherActor != nullptr )
            {
                auto * other_actors_old_other_actor = OtherActor->OtherActor;

                OtherActor->OtherActor = this;

#if WITH_EDITORONLY_DATA
                RenderingComponent->MarkRenderStateDirty();
#endif

                if ( other_actors_old_other_actor != nullptr )
                {
                    other_actors_old_other_actor->OtherActor = nullptr;
#if WITH_EDITORONLY_DATA
                    other_actors_old_other_actor->RenderingComponent->MarkRenderStateDirty();
#endif
                }
            }
        }
        else if ( property_name == NAME_UpdatePathAfterMoving )
        {
            if ( bUpdatePathAfterMoving && OtherActor != nullptr )
            {
                OtherActor->bUpdatePathAfterMoving = false;
            }
        }
    }

    Super::PostEditChangeProperty( property_changed_event );
}

void ASVORaycasterTest::PostEditMove( const bool is_finished )
{
    Super::PostEditMove( is_finished );

    if ( OtherActor != nullptr )
    {
        if ( bUpdatePathAfterMoving )
        {
            DoRaycast();
        }
        else if ( OtherActor->bUpdatePathAfterMoving )
        {
            OtherActor->DoRaycast();
        }
    }
}
#endif

void ASVORaycasterTest::BeginDestroy()
{
    if ( OtherActor != nullptr && OtherActor->OtherActor == this )
    {
        OtherActor->OtherActor = nullptr;
    }

    Super::BeginDestroy();
}

void ASVORaycasterTest::UpdateDrawing()
{
#if WITH_EDITORONLY_DATA
    if ( HasAnyFlags( RF_ClassDefaultObject ) )
    {
        return;
    }

    if ( RenderingComponent != nullptr && RenderingComponent->GetVisibleFlag() )
    {
        RenderingComponent->MarkRenderStateDirty();

#if WITH_EDITOR
        if ( GEditor != NULL )
        {
            GEditor->RedrawLevelEditingViewports();
        }
#endif // WITH_EDITOR
    }
#endif // WITH_EDITORONLY_DATA
}

void ASVORaycasterTest::DoRaycast()
{
    if ( OtherActor == nullptr )
    {
        return;
    }

    if ( Raycaster == nullptr )
    {
        return;
    }

    if ( auto * navigation_system = UNavigationSystemV1::GetCurrent( this ) )
    {
        if ( auto * navigation_data = navigation_system->GetNavDataForProps( NavAgentProperties ) )
        {
            if ( const auto * svo_navigation_data = Cast< ASVONavigationData >( navigation_data ) )
            {
                const auto from = GetActorLocation();
                const auto to = OtherActor->GetActorLocation();

                if ( const auto * volume_navigation_data = svo_navigation_data->GetVolumeNavigationDataContainingPoints( { from, to } ) )
                {
                    Raycaster->SetObserver( MakeShared< FSVORayCasterObserver_GenerateDebugInfos >( RayCasterDebugInfos ) );
                    Raycaster->Trace( *volume_navigation_data, from, to );
                }
            }
        }
    }

    UpdateDrawing();
}