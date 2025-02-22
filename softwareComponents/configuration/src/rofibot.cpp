#include <configuration/rofibot.hpp>

#include <atoms/unreachable.hpp>

namespace rofi::configuration {

using namespace rofi::configuration::matrices;

double roficom::orientationToAngle( roficom::Orientation o ) {
    using namespace roficom;
    switch ( o ) {
        case Orientation::North:
            return M_PI;
        case Orientation::East:
            return - M_PI_2;
        case Orientation::South:
            return 0;
        case Orientation::West:
            return M_PI_2;
    }
    ROFI_UNREACHABLE( "Orientation was modified" );
}

Matrix roficom::orientationToTransform( roficom::Orientation orientation ) {
    // the "default" roficom is A-X
    return translate( { -1, 0, 0 } ) * rotate( M_PI, { 0, 0, 1 } )
        * rotate( M_PI, { 1, 0, 0 } )
        * rotate( roficom::orientationToAngle( orientation ), { -1, 0, 0 } );
}

std::string roficom::orientationToString( Orientation o ) {
    switch ( o ) {
        case Orientation::North:
            return "North";
        case Orientation::East:
            return "East";
        case Orientation::South:
            return "South";
        case Orientation::West:
            return "West";
    }
    ROFI_UNREACHABLE( "Invalid orientation" );
}

roficom::Orientation roficom::stringToOrientation( const std::string& str ) {
    if ( str == "N" || str == "North" )
        return Orientation::North;
    else if ( str == "E" || str == "East" )
        return Orientation::East;
    else if ( str == "S" || str == "South" )
        return Orientation::South;
    else if ( str == "W" || str == "West" )
        return Orientation::West;
    else
        ROFI_UNREACHABLE( "String does not represent the orientation" );
}

int Component::getIndexInParent() const {
    assert( parent != nullptr );
    return parent->componentIdx( *this );
}

Matrix Component::getPosition() const {
    assert( parent != nullptr );
    assert( parent->parent != nullptr );
    auto moduleAbsPosition = parent->parent->getModulePosition( parent->getId() );
    return moduleAbsPosition * parent->getComponentRelativePosition( getIndexInParent() );
}

std::optional< std::pair< const Component&, roficom::Orientation > > Component::getNearConnector() const {
    assert( type == ComponentType::Roficom );
    assert( parent != nullptr );
    assert( parent->parent != nullptr );

    Rofibot& rofibot = *parent->parent;
    if ( !rofibot.isPrepared() )
        throw std::runtime_error( "rofibot is not prepared" );

    auto thisAbsPosition = getPosition();
    for ( auto& moduleInfo : rofibot.modules() ) {
        assert( moduleInfo.module );

        for ( const Component& nearConnector : moduleInfo.module->connectors() ) {
            assert( nearConnector.type == ComponentType::Roficom );

            static constexpr auto allOrientations = std::array{ roficom::Orientation::North,
                                                                roficom::Orientation::East,
                                                                roficom::Orientation::South,
                                                                roficom::Orientation::West };

            for ( roficom::Orientation o : allOrientations ) {
                if ( equals( thisAbsPosition * orientationToTransform( o ),
                             nearConnector.getPosition() ) ) {
                    return { { nearConnector, o } };
                }
            }
        }
    }

    return std::nullopt;
}

bool Module::setId( ModuleId newId ) {
    if ( parent ) {
        if ( parent->_idMapping.contains( newId ) )
            return false;
        parent->_idMapping[ newId ] = parent->_idMapping[ _id ];
        parent->_idMapping.erase( _id );
    }
    _id = newId;
    return true;
}

void Module::setJointPositions( int idx, std::span< const float > p ) {
    // Currently we invalidate all positions; ToDo: think if we can improve it
    assert( idx >= 0 );
    assert( to_unsigned( idx ) < _joints.size() );
    assert( _joints[ to_unsigned( idx ) ].joint->positions().size() == p.size() );
    _joints[ to_unsigned( idx ) ].joint->setPositions( p );
    _componentRelativePositions = std::nullopt;
    if ( parent )
        parent->onModuleMove();
}

void Module::clearComponentPositions() {
    _componentRelativePositions = std::nullopt;
    if ( parent )
        parent->onModuleMove();
}

void Rofibot::setSpaceJointPositions( SpaceJointHandle jointId, std::span< const float > p ) {
    assert( p.size() == _spaceJoints[ jointId ].joint->positions().size() );
    _spaceJoints[ jointId ].joint->setPositions( p );
    _prepared = false;
}

void Rofibot::prepare() {
    using namespace rofi::configuration::matrices;
    _clearModulePositions();

    // Setup position of space joints and extract roots
    std::set< ModuleInfoHandle > roots;
    for ( const SpaceJoint& j : _spaceJoints ) {
        Matrix jointPosition = translate( j.refPoint ) * j.joint->sourceToDest();
        ModuleInfo& mInfo = _modules[ j.destModule ];
        Matrix componentPosition = mInfo.module->getComponentRelativePosition( j.destComponent );
        // Reverse the comonentPosition to get position of the module origin
        Matrix modulePosition = jointPosition * arma::inv( componentPosition );
        if ( mInfo.absPosition ) {
            if ( !equals( mInfo.absPosition.value(), modulePosition ) )
                throw std::runtime_error(
                        fmt::format( "Inconsistent rooting of module {}", mInfo.module->_id ) );
        } else {
            mInfo.absPosition = componentPosition;
        }
        roots.insert( _idMapping[ mInfo.module->_id ] );
    }

    auto dfsTraverse = [&]( ModuleInfo& m, Matrix position, auto& self ) {
        if ( m.absPosition ) {
            if ( !equals( position, m.absPosition.value() ) )
                throw std::runtime_error(
                        fmt::format( "Inconsistent position of module {}", m.module->_id ) );
            return;
        }

        m.absPosition = position;
        // Traverse ignoring edge orientation
        std::vector< RoficomJointHandle > joints;
        std::copy( m.outJointsIdx.begin(), m.outJointsIdx.end(), std::back_inserter( joints ) );
        std::copy( m.inJointsIdx.begin(), m.inJointsIdx.end(), std::back_inserter( joints ) );
        for ( auto outJointIdx : joints ) {
            const RoficomJoint& j = _moduleJoints[ outJointIdx ];

            bool mIsSource = j.sourceModule == _idMapping[ m.module->_id ];
            Matrix jointTransf = mIsSource ? j.sourceToDest() : j.destToSource();
            Matrix jointRefPosition = position
                                    * m.module->getComponentRelativePosition( mIsSource
                                                                            ? j.sourceConnector
                                                                            : j.destConnector )
                                    * jointTransf;
            ModuleInfo& other = _modules[ mIsSource ? j.destModule : j.sourceModule ];
            Matrix otherConnectorPosition = other.module->getComponentRelativePosition( mIsSource
                                                                                      ? j.destConnector
                                                                                      : j.sourceConnector );
            // Reverse the comonentPosition to get position of the module origin
            Matrix otherPosition = jointRefPosition * arma::inv( otherConnectorPosition );
            self( other, otherPosition, self );
        }
    };

    for ( auto h : roots ) {
        ModuleInfo& m = _modules[ h ];
        auto pos = m.absPosition.value();
        m.absPosition.reset();
        dfsTraverse( m, pos, dfsTraverse );
    }

    for ( ModuleInfo& m : _modules ) {
        if ( !m.absPosition.has_value() )
            throw std::runtime_error(
                    fmt::format( "Not fixed position of module {}", m.module->_id ) );
    }

    _prepared = true;
}

void Rofibot::disconnect( RoficomJointHandle h ) {
    assert( _moduleJoints.contains( h ) );

    Rofibot::ModuleInfo& m1info = _modules[ _moduleJoints[ h ].sourceModule ];
    Rofibot::ModuleInfo& m2info = _modules[ _moduleJoints[ h ].destModule ];
    [[maybe_unused]] auto erased1 = std::erase( m1info.outJointsIdx, h );
    [[maybe_unused]] auto erased2 = std::erase( m2info.inJointsIdx, h );
    assert( erased1 == 1 );
    assert( erased2 == 1 );

    _moduleJoints.erase( h );
    _prepared = false;
}

void Rofibot::disconnect( SpaceJointHandle h ) {
    assert( _spaceJoints.contains( h ) );

    Rofibot::ModuleInfo& info = _modules[ _spaceJoints[ h ].destModule ];
    [[maybe_unused]] auto erased = std::erase( info.spaceJoints, h );
    assert( erased == 1 );

    _spaceJoints.erase( h );
    _prepared = false;
}

Rofibot::RoficomJointHandle connect( const Component& c1, const Component& c2, roficom::Orientation o ) {
    assert( c1.type == ComponentType::Roficom && c2.type == ComponentType::Roficom && "Components are not RoFICoMs" );

    if ( c1.parent->parent != c2.parent->parent )
        throw std::logic_error( "Components have to be in the same rofibot" );
    Rofibot& bot = *c1.parent->parent;
    Rofibot::ModuleInfo& m1info = bot._modules[ bot._idMapping[ c1.parent->getId() ] ];
    Rofibot::ModuleInfo& m2info = bot._modules[ bot._idMapping[ c2.parent->getId() ] ];

    auto jointHandle = bot._moduleJoints.insert( {
        o, bot._idMapping[ m1info.module->getId() ], bot._idMapping[ m2info.module->getId() ],
        m1info.module->componentIdx( c1 ), m2info.module->componentIdx( c2 )
    } );

    m1info.outJointsIdx.push_back( jointHandle );
    m2info.inJointsIdx.push_back( jointHandle );

    bot._prepared = false;
    return jointHandle;
}

} // namespace rofi::configuration
