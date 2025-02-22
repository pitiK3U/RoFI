#include "simplesim/module_states.hpp"


using namespace rofi::configuration;
using namespace rofi::simplesim;


auto removeConnection( const rofi::configuration::Component & roficom )
        -> std::optional< detail::ConfigurationUpdateEvents::ConnectionChanged >
{
    using CUE = detail::ConfigurationUpdateEvents;

    assert( roficom.type == ComponentType::Roficom );
    assert( roficom.parent != nullptr );
    assert( roficom.parent->parent != nullptr );

    Rofibot & rofibot = *roficom.parent->parent;

    auto thisConnector = Connector{ .moduleId = roficom.parent->getId(),
                                    .connIdx = roficom.getIndexInParent() };

    const auto & connections = rofibot.roficomConnections();
    for ( auto connIt = connections.begin(); connIt != connections.end(); ++connIt ) {
        auto sourceConnector = Connector{ .moduleId = connIt->getSourceModule( rofibot ).getId(),
                                          .connIdx = connIt->sourceConnector };
        auto destConnector = Connector{ .moduleId = connIt->getDestModule( rofibot ).getId(),
                                        .connIdx = connIt->destConnector };

        if ( thisConnector == sourceConnector || thisConnector == destConnector ) {
            rofibot.disconnect( connIt.get_handle() );
            return CUE::ConnectionChanged{ .lhs = sourceConnector,
                                           .rhs = destConnector,
                                           .orientation = std::nullopt };
        }
    }
    return std::nullopt;
}


std::optional< ModuleStates::RofiDescription > ModuleStates::getDescription(
        ModuleId moduleId ) const
{
    if ( auto * moduleInnerState = detail::getModuleInnerState( _moduleInnerStates, moduleId ) ) {
        RofiDescription description;
        size_t jointCount = moduleInnerState->joints().size();
        size_t connectorCount = moduleInnerState->connectors().size();
        assert( jointCount <= INT_MAX );
        assert( connectorCount <= INT_MAX );
        description.set_jointcount( static_cast< int >( jointCount ) );
        description.set_connectorcount( static_cast< int >( connectorCount ) );
        return description;
    }
    return {};
}

// Returns joint position limits if module with moduleId exists and has given joint
std::optional< std::pair< float, float > > ModuleStates::getJointPositionLimits( Joint joint ) const
{
    auto currentConfig = currentConfiguration();
    assert( currentConfig );

    if ( auto jointConfig = detail::getJointConfig( *currentConfig, joint ) ) {
        std::span jointLimits = jointConfig->jointLimits();
        if ( !jointLimits.empty() ) {
            assert( jointLimits.size() == 1 );
            return jointLimits.front();
        }
    }
    return {};
}

std::optional< ModuleStates::JointCapabilities > ModuleStates::getJointCapabilities(
        Joint joint ) const
{
    if ( auto jointLimits = getJointPositionLimits( joint ) ) {
        JointCapabilities capabilities;
        capabilities.set_minposition( jointLimits->first );
        capabilities.set_maxposition( jointLimits->second );

        // TODO set max speed on per module/per joint bases
        capabilities.set_maxspeed( ModuleStates::maxJointSpeed );

        return capabilities;
    }
    return {};
}

std::optional< float > ModuleStates::getJointPosition( Joint joint ) const
{
    auto currentConfig = currentConfiguration();
    assert( currentConfig );

    if ( auto jointConfig = detail::getJointConfig( *currentConfig, joint ) ) {
        std::span positions = jointConfig->positions();
        if ( !positions.empty() ) {
            assert( positions.size() == 1 );
            return positions.front();
        }
    }
    return {};
}

std::optional< float > ModuleStates::getJointVelocity( Joint joint ) const
{
    if ( auto * jointInnerState = detail::getJointInnerState( _moduleInnerStates, joint ) ) {
        return jointInnerState->velocity();
    }
    return std::nullopt;
}

bool ModuleStates::setPositionControl( Joint joint,
                                       ModuleStates::JointPositionControl positionControl )
{
    if ( auto * jointInnerState = detail::getJointInnerState( _moduleInnerStates, joint ) ) {
        jointInnerState->setPositionControl( std::move( positionControl ) );
        return true;
    }
    return false;
}

bool ModuleStates::setVelocityControl( Joint joint,
                                       ModuleStates::JointVelocityControl velocityControl )
{
    if ( auto * jointInnerState = detail::getJointInnerState( _moduleInnerStates, joint ) ) {
        jointInnerState->setVelocityControl( std::move( velocityControl ) );
        return true;
    }
    return false;
}

std::optional< rofi::messages::ConnectorState > ModuleStates::getConnectorState(
        Connector connector ) const
{
    if ( auto * connInnerState = detail::getConnectorInnerState( _moduleInnerStates, connector ) ) {
        return connInnerState->connectorState();
    }
    return {};
}

std::optional< ModuleStates::ConnectedTo > ModuleStates::getConnectedTo( Connector connector ) const
{
    if ( auto * connInnerState = detail::getConnectorInnerState( _moduleInnerStates, connector ) ) {
        return connInnerState->connectedTo();
    }
    return {};
}

bool ModuleStates::extendConnector( Connector connector )
{
    if ( auto * connInnerState = detail::getConnectorInnerState( _moduleInnerStates, connector ) ) {
        connInnerState->setExtending();
        return true;
    }
    return false;
}

std::optional< ModuleStates::ConnectedTo > ModuleStates::retractConnector( Connector connector )
{
    if ( auto * connInnerState = detail::getConnectorInnerState( _moduleInnerStates, connector ) ) {
        auto connectedTo = connInnerState->resetConnectedTo();
        connInnerState->setRetracting();

        if ( connectedTo ) {
            if ( auto * otherConnInner = detail::getConnectorInnerState( _moduleInnerStates,
                                                                         connectedTo->connector ) )
            {
                [[maybe_unused]] auto otherConnectedTo = otherConnInner->resetConnectedTo();

                assert( otherConnectedTo );
                assert( otherConnectedTo->connector == connector );
                assert( otherConnectedTo->orientation == connectedTo->orientation );
            } else {
                std::cerr << "Inconsistent state (couldn't find module "
                          << connectedTo->connector.moduleId << ", connector "
                          << connectedTo->connector.connIdx << ")\n";
            }
        }
        return { connectedTo };
    }
    return std::nullopt;
}

bool ModuleStates::setConnectorPower( Connector connector, ConnectorLine line, bool connect )
{
    using ConnectorCmd = rofi::messages::ConnectorCmd;

    if ( auto * connInnerState = detail::getConnectorInnerState( _moduleInnerStates, connector ) ) {
        switch ( line ) {
            case ConnectorCmd::INT_LINE:
                connInnerState->internal() = connect;
                return true;
            case ConnectorCmd::EXT_LINE:
                connInnerState->external() = connect;
                return true;
            default:
                return false;
        }
    }
    return false;
}

constexpr auto enumerated()
{
    return std::views::transform( [ i = 0UL ]< typename T >( T && value ) mutable {
        return std::tuple< T, size_t >( std::forward< T >( value ), i++ );
    } );
}

auto updateJointPositions( Rofibot & configuration,
                           std::chrono::duration< float > simStepTime,
                           const detail::ModuleInnerStates & moduleInnerStates )
        -> std::vector< detail::ConfigurationUpdateEvents::PositionReached >
{
    using PositionReached = detail::ConfigurationUpdateEvents::PositionReached;

    auto positionsReached = std::vector< PositionReached >();
    for ( auto & moduleInfo : configuration.modules() ) {
        assert( moduleInfo.module.get() );
        auto & module_ = *moduleInfo.module;
        assert( module_.parent == &configuration );

        auto * moduleInnerState = detail::getModuleInnerState( moduleInnerStates, module_.getId() );
        assert( moduleInnerState );

        std::span jointInnerStates = moduleInnerState->joints();
        std::ranges::view auto jointConfigurations = module_.configurableJoints();
        assert( std::ssize( jointInnerStates ) == std::ranges::distance( jointConfigurations ) );

        for ( auto [ jointConfiguration, i ] : jointConfigurations | enumerated() ) {
            static_assert( std::is_same_v< decltype( jointConfiguration ),
                                           rofi::configuration::Joint & > );
            const auto & jointInnerState = jointInnerStates[ i ];

            assert( jointConfiguration.positions().size() == 1 );
            assert( jointConfiguration.jointLimits().size() == 1 );
            auto currentPosition = jointConfiguration.positions().front();
            auto jointLimits = jointConfiguration.jointLimits().front();

            auto [ posReached, newPosition ] = jointInnerState.computeNewPosition( currentPosition,
                                                                                   simStepTime );
            if ( posReached ) {
                assert( i < INT_MAX );
                positionsReached.push_back(
                        PositionReached{ .joint = { .moduleId = module_.getId(),
                                                    .jointIdx = static_cast< int >( i ) },
                                         .position = newPosition } );
            }

            assert( jointLimits.first <= jointLimits.second );
            auto clampedNewPosition = std::clamp( newPosition,
                                                  jointLimits.first,
                                                  jointLimits.second );

            jointConfiguration.setPositions( std::array{ clampedNewPosition } );
        }
    }
    return positionsReached;
}

auto updateConnectorStates( Rofibot & configuration,
                            const detail::ModuleInnerStates & moduleInnerStates )
{
    using ConnectionChanged = detail::ConfigurationUpdateEvents::ConnectionChanged;
    struct ConnectorUpdateEvents {
        std::vector< Connector > connectorsToFinalizePosition;
        std::vector< ConnectionChanged > connectionsChanged;
    };

    auto connectorUpdateEvents = ConnectorUpdateEvents();
    for ( auto & moduleInfo : configuration.modules() ) {
        assert( moduleInfo.module.get() );
        auto & module_ = *moduleInfo.module;
        assert( module_.parent == &configuration );

        auto * moduleInnerState = detail::getModuleInnerState( moduleInnerStates, module_.getId() );
        assert( moduleInnerState );

        std::span connectorInnerStates = moduleInnerState->connectors();
        std::span connectorConfigurations = module_.connectors();
        assert( connectorInnerStates.size() == connectorConfigurations.size() );

        for ( auto [ connectorInnerState, i ] : connectorInnerStates | enumerated() ) {
            assert( connectorConfigurations[ i ].parent == &module_ );
            switch ( connectorInnerState.position() ) {
                case ConnectorInnerState::Position::Retracted:
                case ConnectorInnerState::Position::Extended:
                    break;
                case ConnectorInnerState::Position::Retracting:
                {
                    assert( i < INT_MAX );
                    connectorUpdateEvents.connectorsToFinalizePosition.push_back(
                            { .moduleId = module_.getId(), .connIdx = static_cast< int >( i ) } );

                    if ( auto removedConnection = removeConnection( connectorConfigurations[ i ] ) )
                    {
                        connectorUpdateEvents.connectionsChanged.push_back( *removedConnection );
                    }
                    break;
                }
                case ConnectorInnerState::Position::Extending:
                {
                    assert( i < INT_MAX );
                    connectorUpdateEvents.connectorsToFinalizePosition.push_back(
                            { .moduleId = module_.getId(), .connIdx = static_cast< int >( i ) } );

                    const auto & connConfiguration = connectorConfigurations[ i ];
                    assert( connConfiguration.parent == &module_ );
                    if ( auto connection = detail::connectToNearbyConnector( connConfiguration ) ) {
                        connectorUpdateEvents.connectionsChanged.push_back( *connection );
                    }
                    break;
                }
            }

            // Workaround for a bug in configuration (not reseting and requiring the prepared flag)
            if ( !configuration.isPrepared() ) {
                configuration.prepare();
            }
        }
    }

    return connectorUpdateEvents;
}

auto ModuleStates::computeNextIteration( std::chrono::duration< float > simStepTime ) const
        -> std::pair< RofibotConfigurationPtr, detail::ConfigurationUpdateEvents >
{
    using CUE = detail::ConfigurationUpdateEvents;

    auto currentConfig = currentConfiguration();
    assert( currentConfig );
    auto newConfiguration = std::make_shared< Rofibot >( *currentConfig );
    assert( newConfiguration );

    // Workaround for a bug in configuration (not setting component parent properly)
    for ( auto & moduleInfo : newConfiguration->modules() ) {
        for ( auto & component : moduleInfo.module->components() ) {
            component.parent = moduleInfo.module.get();
        }
    }

    CUE updateEvents;
    updateEvents.positionsReached = updateJointPositions( *newConfiguration,
                                                          simStepTime,
                                                          _moduleInnerStates );

    // Workaround for a bug in configuration (not setting the prepared flag properly)
    newConfiguration->prepare();

    if ( auto [ ok, err ] = newConfiguration->validate( SimpleCollision{} ); !ok ) {
        std::cerr << "Error after joint update: '" << err << "'\n";
        throw std::runtime_error( std::move( err ) );
    }

    auto connectorUpdateEvents = updateConnectorStates( *newConfiguration, _moduleInnerStates );
    updateEvents.connectorsToFinalizePosition = std::move(
            connectorUpdateEvents.connectorsToFinalizePosition );
    updateEvents.connectionsChanged = std::move( connectorUpdateEvents.connectionsChanged );

    if ( auto [ ok, err ] = newConfiguration->validate( SimpleCollision{} ); !ok ) {
        std::cerr << "Error after connector update: '" << err << "'\n";
        throw std::runtime_error( std::move( err ) );
    }
    return std::pair( newConfiguration, std::move( updateEvents ) );
}

auto ModuleStates::initInnerStatesFromConfiguration( const Rofibot & rofibotConfiguration,
                                                     bool verbose )
        -> std::map< ModuleId, ModuleInnerState >
{
    auto innerStates = std::map< ModuleId, ModuleInnerState >();
    for ( const auto & moduleInfo : rofibotConfiguration.modules() ) {
        const auto & _module = *moduleInfo.module;

        auto joints = std::ranges::distance( _module.configurableJoints() );
        auto connectors = _module.connectors().size();
        assert( joints < INT_MAX );
        assert( connectors < INT_MAX );
        auto moduleInnerState = ModuleInnerState( static_cast< int >( joints ),
                                                  static_cast< int >( connectors ) );

        if ( verbose ) {
            std::cerr << "Module id: " << _module.getId()
                      << ", joints: " << moduleInnerState.joints().size()
                      << ", connectors: " << moduleInnerState.connectors().size() << std::endl;
        }

        auto [ it, success ] = innerStates.emplace( _module.getId(),
                                                    std::move( moduleInnerState ) );
        if ( !success ) {
            throw std::runtime_error( "Multiple same module ids in configuration" );
        }
    }

    for ( auto & connection : rofibotConfiguration.roficomConnections() ) {
        auto sourceConnector =
                Connector{ .moduleId = connection.getSourceModule( rofibotConfiguration ).getId(),
                           .connIdx = connection.sourceConnector };
        auto destConnector =
                Connector{ .moduleId = connection.getDestModule( rofibotConfiguration ).getId(),
                           .connIdx = connection.destConnector };

        auto * sourceConnState = detail::getConnectorInnerState( innerStates, sourceConnector );
        assert( sourceConnState );
        auto * destConnState = detail::getConnectorInnerState( innerStates, destConnector );
        assert( destConnState );

        assert( !sourceConnState->connectedTo().has_value() );
        if ( sourceConnState->position() != ConnectorInnerState::Position::Extended ) {
            std::cerr << fmt::format(
                    "Connector {} of module {} connected but not extended. Setting as extended.\n",
                    sourceConnector.moduleId,
                    sourceConnector.connIdx );
            sourceConnState->setExtending();
            sourceConnState->finalizePosition();
        }
        sourceConnState->setConnectedTo( destConnector, connection.orientation );

        assert( !destConnState->connectedTo().has_value() );
        if ( destConnState->position() != ConnectorInnerState::Position::Extended ) {
            std::cerr << fmt::format(
                    "Connector {} of module {} connected but not extended. Setting as extended.\n",
                    destConnector.moduleId,
                    destConnector.connIdx );
            destConnState->setExtending();
            destConnState->finalizePosition();
        }
        destConnState->setConnectedTo( sourceConnector, connection.orientation );
    }

    return innerStates;
}
